/* Native USB1 CDC + VOFA FireWater. Commands are ASCII lines, never run in USB ISR. */
#include "MotorTune.h"
#include "MotorPID.h"
#include "Servo.h"
#include "zf_driver_usb_cdc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#define TUNE_RUN_TICKS 500U    // 每次启动最多10秒，便于带线架空调参
#define TUNE_STALL_DELAY 75U   // 起步1.5秒后检查堵转/编码器
#define TUNE_STALL_TICKS 25U   // 连续0.5秒低反馈时停止
typedef struct {
    float left, right, lp, li, ld, rp, ri, rd;
    uint8 run, reason; // 0就绪/手动停止，1超时，2左低速，3右低速，4两侧低速，5USB断开
    uint32 ticks, accepted, rejected;
} TuneConfig;
static volatile TuneConfig cfg;
static volatile uint32 tick20;
static volatile float measured_l, measured_r;
static int32 sum_l, sum_r;
static uint16 sample_count, elapsed, stall_l, stall_r;
static uint8 stable[4], debounce[4], selected;
static const gpio_pin_enum pins[4]={C15,C14,C13,C12};
static uint32 control_session, parser_session, sent_tick, shown_tick;
static char command[96];
static uint8 command_len, discard_line;

static uint32 lock(void) { uint32 m=__get_PRIMASK(); __disable_irq(); return m; }
static void unlock(uint32 mask) { __set_PRIMASK(mask); }
static void stop(uint8 reason)
{
    cfg.run=0; cfg.reason=reason;
    MotorPID_SetEnable(0);
}
static void set_run(uint8 run)
{
    if(!run) { stop(0); return; }
    if(!usb_cdc_ready()) { stop(5); return; }
    if(!cfg.run) { elapsed=stall_l=stall_r=0; cfg.run=1; cfg.reason=0; }
}
static uint8 valid_speed(float value)
{
    return isfinite(value) && (value==0.0f || (value>=70.0f && value<=240.0f));
}
static void defaults(void)
{
    stop(0);
    cfg.left=cfg.right=100.0f;
    cfg.lp=cfg.rp=MOTORPID_DEFAULT_KP;
    cfg.li=cfg.ri=MOTORPID_DEFAULT_KI;
    cfg.ld=cfg.rd=MOTORPID_DEFAULT_KD;
}

void MotorTune_Init(void)
{
    uint8 i;
    MotorPID_Init(); defaults(); Servo_Steer(0);
    key_init(10);
    for(i=0;i<4;i++) { stable[i]=(gpio_get_level(pins[i])==GPIO_LOW); debounce[i]=0; }
    control_session=parser_session=usb_cdc_session();
}

void MotorTune_Keys10ms(void)
{
    uint8 i, events=0;
    for(i=0;i<4;i++)
    {
        uint8 pressed=(gpio_get_level(pins[i])==GPIO_LOW);
        if(pressed==stable[i]) { debounce[i]=0; }
        else if(++debounce[i]>=3)
        {
            debounce[i]=0; stable[i]=pressed;
            if(pressed) { events|=(uint8)(1U<<i); }
        }
    }
    if(events&4U) { set_run(!cfg.run); return; }
    if(events&8U) { selected=(uint8)((selected+1U)%3U); return; }
    if((events&3U)==1U || (events&3U)==2U)
    {
        float delta=(events&2U) ? 5.0f : -5.0f;
        if(selected!=2)
        { float v=cfg.left+delta; if(v>=70 && v<=240) { cfg.left=v; } }
        if(selected!=1)
        { float v=cfg.right+delta; if(v>=70 && v<=240) { cfg.right=v; } }
    }
}

void MotorTune_Sample10ms(int left, int right)
{
    sum_l+=left; sum_r+=right; sample_count++;
}

void MotorTune_Control20ms(void)
{
    uint32 session=usb_cdc_session();
    if(sample_count)
    {
        measured_l=(float)sum_l/sample_count;
        measured_r=(float)sum_r/sample_count;
        sum_l=sum_r=0; sample_count=0;
    }
    if(!usb_cdc_ready() || session!=control_session) { stop(5); }
    control_session=session;
    Servo_Steer(0);
    if(cfg.run)
    {
        if(++elapsed>=TUNE_RUN_TICKS) { stop(1); }
        else
        {
            MotorPID_SetEnable(1);
            MotorPID_SetWheelGains(cfg.lp,cfg.li,cfg.ld,cfg.rp,cfg.ri,cfg.rd);
            MotorPID_SetWheelTargets(cfg.left,cfg.right);
            MotorPID_Update((int)(measured_l+0.5f),(int)(measured_r+0.5f));
            if(elapsed>TUNE_STALL_DELAY)
            {
                stall_l=(cfg.left>=70 && measured_l<3) ? (uint16)(stall_l+1) : 0;
                stall_r=(cfg.right>=70 && measured_r<3) ? (uint16)(stall_r+1) : 0;
                if(stall_l>=TUNE_STALL_TICKS || stall_r>=TUNE_STALL_TICKS)
                { stop((stall_l>=TUNE_STALL_TICKS && stall_r>=TUNE_STALL_TICKS) ? 4 : (stall_l>=TUNE_STALL_TICKS ? 2 : 3)); }
            }
        }
    }
    else if(MotorPID_IsEnabled()) { MotorPID_SetEnable(0); }
    cfg.ticks=elapsed;
    tick20++;
}

// Strict complete-line parser: malformed/split/oversized input cannot partly alter gains.
static int numbers(const char *text, float *values, int count)
{
    int i;
    char *end;
    for(i=0;i<count;i++)
    {
        values[i]=strtof(text,&end);
        if(end==text || !isfinite(values[i])) { return 0; }
        text=end;
        while(isspace((unsigned char)*text)) { text++; }
        if(i+1<count) { if(*text++!=',') { return 0; } }
    }
    return *text=='\0';
}
static void apply_line(char *line)
{
    char *eq=strchr(line,'=');
    float v[3];
    uint8 op=0;
    uint32 mask;
    if(eq) { *eq++='\0'; }
    // 浮点解析放在临界区外，避免串口命令延迟编码器/PID定时中断。
    if(!eq && !strcmp(line,"stop")) { op=1; }
    else if(!eq && !strcmp(line,"reset")) { op=2; }
    else if(eq)
    {
        if(!strcmp(line,"pid") || !strcmp(line,"lpid") || !strcmp(line,"rpid"))
        {
            if(numbers(eq,v,3) && v[0]>=0 && v[0]<=100 && v[1]>=0 && v[1]<=20 && v[2]>=0 && v[2]<=100)
            {
                op=(!strcmp(line,"pid")) ? 8 : ((!strcmp(line,"lpid")) ? 9 : 10);
            }
        }
        else if(numbers(eq,v,1))
        {
            if(!strcmp(line,"run") && (v[0]==0 || v[0]==1)) { op=3; }
            else if(valid_speed(v[0]))
            {
                if(!strcmp(line,"speed")) { op=4; }
                else if(!strcmp(line,"left")) { op=5; }
                else if(!strcmp(line,"right")) { op=6; }
            }
        }
    }
    mask=lock();
    switch(op)
    {
        case 1: stop(0); break;
        case 2: defaults(); break;
        case 3: set_run(v[0]==1); break;
        case 4: cfg.left=cfg.right=v[0]; break;
        case 5: cfg.left=v[0]; break;
        case 6: cfg.right=v[0]; break;
        case 8: case 9: case 10:
            if(op!=10) { cfg.lp=v[0]; cfg.li=v[1]; cfg.ld=v[2]; }
            if(op!=9) { cfg.rp=v[0]; cfg.ri=v[1]; cfg.rd=v[2]; }
            break;
        default: break;
    }
    if(op) { cfg.accepted++; } else { cfg.rejected++; }
    unlock(mask);
}

void MotorTune_Poll(void)
{
    uint8 c;
    uint16 receive_budget=1024;
    uint32 mask, tick, session=usb_cdc_session();
    TuneConfig s;
    float ml,mr,tl,tr;
    int ol,orr,n;
    char output[256];
    if(session!=parser_session) { command_len=discard_line=0; parser_session=session; }
    if(usb_cdc_rx_fault()) { command_len=0; discard_line=1; cfg.rejected++; }
    while(receive_budget-- && usb_cdc_read_byte(&c))
    {
        if(c=='\n' || c=='\r')
        {
            if(command_len && !discard_line) { command[command_len]='\0'; apply_line(command); }
            command_len=discard_line=0;
        }
        else if(c<32 || c>126)
        { if(!discard_line) { cfg.rejected++; } discard_line=1; command_len=0; }
        else if(!discard_line)
        {
            if(command_len<sizeof(command)-1) { command[command_len++]=(char)c; }
            else { command_len=0; discard_line=1; cfg.rejected++; }
        }
    }
    mask=lock();
    tick=tick20;
    s=cfg; ml=measured_l; mr=measured_r;
    tl=MotorPID_GetTargetLeft(); tr=MotorPID_GetTargetRight();
    ol=MotorPID_GetOutputLeft(); orr=MotorPID_GetOutputRight();
    unlock(mask);
    if(tick==sent_tick) { return; }
    // First six channels are the actual control setpoints, feedback and separate PWM outputs.
    n=snprintf(output,sizeof(output),"%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%lu,%lu\n",
        tl,tr,ml,mr,ol,orr,s.left,s.right,s.lp,s.li,s.ld,s.rp,s.ri,s.rd,
        (unsigned)s.run,(unsigned)s.reason,(unsigned long)s.accepted,(unsigned long)s.rejected);
    if(n>0 && n<(int)sizeof(output) && usb_cdc_try_write((uint8*)output,(uint32)n)) { sent_tick=tick; }
}

void MotorTune_Show(void)
{
    TuneConfig s;
    float ml,mr,tl,tr;
    int ol,orr;
    uint32 mask=lock(), tick=tick20;
    s=cfg; ml=measured_l; mr=measured_r;
    tl=MotorPID_GetTargetLeft(); tr=MotorPID_GetTargetRight();
    ol=MotorPID_GetOutputLeft(); orr=MotorPID_GetOutputRight();
    unlock(mask);
    if(tick-shown_tick<5) { return; } // 屏幕10Hz；通信和PID均为50Hz
    shown_tick=tick;
    ips200_show_string(0,8,"USB MOTOR PID / FIREWATER");
    ips200_show_string(0,32,usb_cdc_ready() ? "USB: READY   LIFT WHEELS " : "USB: WAIT    LIFT WHEELS ");
    ips200_show_string(0,56,s.run ? "RUN: ON  (MAX 10 SECONDS)" : "RUN: OFF                ");
    ips200_show_string(0,80,"REQUEST L/R:");
    ips200_show_int(104,80,(int)s.left,3); ips200_show_int(160,80,(int)s.right,3);
    ips200_show_string(0,104,"TARGET  L/R:");
    ips200_show_int(104,104,(int)tl,3); ips200_show_int(160,104,(int)tr,3);
    ips200_show_string(0,128,"ENCODER L/R:");
    ips200_show_int(104,128,(int)ml,4); ips200_show_int(160,128,(int)mr,4);
    ips200_show_string(0,152,"PWM OUT L/R:");
    ips200_show_int(104,152,ol,4); ips200_show_int(160,152,orr,4);
    ips200_show_string(0,176,"L P/I/D:");
    ips200_show_float(64,176,s.lp,3,2); ips200_show_float(120,176,s.li,2,2); ips200_show_float(168,176,s.ld,3,2);
    ips200_show_string(0,200,"R P/I/D:");
    ips200_show_float(64,200,s.rp,3,2); ips200_show_float(120,200,s.ri,2,2); ips200_show_float(168,200,s.rd,3,2);
    ips200_show_string(0,224,"RX OK/ERR:"); ips200_show_int(88,224,(int)s.accepted,4); ips200_show_int(144,224,(int)s.rejected,4);
    ips200_show_string(0,248,"STOP CODE:"); ips200_show_int(88,248,s.reason,1);
    ips200_show_string(128,248,selected==0 ? "BOTH " : (selected==1 ? "LEFT " : "RIGHT"));
    ips200_show_string(0,272,"C15:-5 C14:+5 C13:ON/OFF");
    ips200_show_string(0,296,"C12:SELECT L/R/BOTH");
}
