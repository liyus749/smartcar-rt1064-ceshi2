"""Exercise the real wireless parser, main command handler, 20ms ISR and MotorPID."""
import ast
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
project_dir=ROOT/'project/mdk'
main_path=ET.parse(project_dir/'rt1064.uvprojx').find(".//Files/File[FileName='main.c']/FilePath").text
main_source=(project_dir/main_path.replace('\\','/')).resolve()
code_dir=main_source.parents[2]/'code'
source=main_source.read_text(encoding='utf-8')
# Reuse the existing motor/servo hardware mock, without running its test program.
tree=ast.parse((ROOT/'tests/run_track_diff_integration_tests.py').read_text(encoding='utf-8'))
base={}
for node in tree.body:
    if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name):
        if node.targets[0].id in ('names','PREFIX','STUB'):
            base[node.targets[0].id]=ast.literal_eval(node.value)
names=base['names']+('SERVO_D_DEFAULT','USE_REMOTE_CONTROL','USE_RUN_KEYS','USE_LEGACY_GYRO','USE_SCREEN','RUN_AUTO_START')
config='\n'.join(re.search(r'^#define\s+'+name+r'\s+.*$',source,re.M).group(0) for name in names)
stub=base['STUB'].replace('typedef uint8_t uint8;', '''#include <stddef.h>
typedef uint32_t uint32;typedef int32_t int32;
uint32 __get_PRIMASK(void);void __disable_irq(void);void __set_PRIMASK(uint32);
int gpio_get_level(int);void uart_write_buffer(int,const uint8_t*,uint32_t);
typedef uint8_t uint8;''')
wireless_stub='''#include "zf_common_headfile.h"
#define WIRELESS_UART_INDEX 8
#define WIRELESS_UART_RTS_PIN 26
uint8 wireless_uart_init(void);
uint32 wireless_uart_rx_errors(void);
uint32 wireless_uart_read_buffer(uint8*,uint32);
'''
def block(name):
    return source.split('// BEGIN '+name+'\n',1)[1].split('// END '+name,1)[0]
controller='void pit_handler0(void)'+source.split('void pit_handler0(void)',1)[1].split(
    '//===================================================================================================================',1)[0]
sampler='void pit_handler1(void)'+source.split('void pit_handler1(void)',1)[1].split(
    '\n//-------------------------------------------------------------------------------------------------------------------',1)[0]
EXTRA=r'''
#include <string.h>
#include <stdlib.h>
#include "TrackRemote.h"
#include "zf_device_wireless_uart.h"
static float run_kp=8,run_ki=.15f,run_kd=0;
static unsigned irq,rx_errors,init_failed,rx_size,rx_pos,tx_size;
static int rts_busy;
static unsigned encoder_clears;
#define QTIMER1_ENCODER1 1
#define QTIMER1_ENCODER2 2
int encoder_get_count(int index) { assert(index==1 || index==2);return index==1 ? -27:31; }
void encoder_clear_count(int index) { assert(index==1 || index==2);encoder_clears++; }
static char rx[4096],transmitted[100000];
static char screen_motion[8];static uint8 screen_waiting;
void Screen_ShowRunState(uint8 moving,uint8 waiting) {
    strcpy(screen_motion,moving ? "RUN ":(waiting ? "WAIT":"STOP"));
    screen_waiting=waiting;
}
uint32 __get_PRIMASK(void) { return irq; }
void __disable_irq(void) { irq=1; }
void __set_PRIMASK(uint32 m) { irq=m; }
uint8 wireless_uart_init(void) { return (uint8)init_failed; }
uint32 wireless_uart_rx_errors(void) { return rx_errors; }
uint32 wireless_uart_read_buffer(uint8 *out,uint32 size) {
    unsigned n=rx_size-rx_pos;if(n>size)n=size;
    memcpy(out,rx+rx_pos,n);rx_pos+=n;return n;
}
int gpio_get_level(int pin) { assert(pin==26);return rts_busy; }
void uart_write_buffer(int port,const uint8 *s,uint32 n) {
    assert(port==8 && !rts_busy && !irq && n<=30);
    assert(tx_size+n<sizeof(transmitted));memcpy(transmitted+tx_size,s,n);tx_size+=n;
}
int Cross_GetState(void) { return Cross_Flag ? 2:0; }
int Roundabout_GetDirection(void) { return Island_State ? 1:0; }
void Roundabout_Retry(void) { round_pending=1; }
'''
TEST=r'''
static void near(float a,float b) { assert(fabsf(a-b)<.002f); }
static void feed(const char *s) {
    assert(rx_pos==rx_size);rx_pos=0;rx_size=(unsigned)strlen(s);memcpy(rx,s,rx_size);
    do { Run_RemoteService(); } while(rx_pos<rx_size);
    assert(!irq);
}
static void tick(int good,int n) {
    for(int i=0;i<n;i++) {
        if(good) { TrackRemote_Frame(1);Run_ErrorDiffFrame(ErrUse,ErrUse,run_switch>0.5f); }
        enc_l=(int)(MotorPID_GetTargetLeft()+.5f);enc_r=(int)(MotorPID_GetTargetRight()+.5f);
        pit_handler0();
    }
}
static void stopped(int why) {
    assert(!run_switch && !MotorPID_IsEnabled() && !pwm_l && !pwm_r && servo_duty==790);
    assert(TrackRemote_GetReason()==why);
}
int main(void) {
    assert(USE_RUN_KEYS==0);pit_handler1();assert(enc_l==27 && enc_r==31 && encoder_clears==2);
    MotorPID_Init();assert(!TrackRemote_Init());run_d=SERVO_D_DEFAULT;
    assert(RUN_AUTO_START==1 && SPEED_DEFAULT==180);near(run_speed,180);
    run_enable=0;Run_AutoStartTry(1);assert(!run_switch && run_auto_pending);
    run_enable=1;
    for(int i=0;i<30;i++) { Run_AutoStartTry(0);pit_handler0();assert(!run_switch); }
    TrackRemote_Frame(1);Run_AutoStartTry(1);assert(run_switch && !run_auto_pending);
    tick(1,80);near(MotorPID_GetTargetLeft(),180);near(MotorPID_GetTargetRight(),180);
    tick(0,25);stopped(REMOTE_REASON_CAMERA);
    TrackRemote_Frame(1);Run_AutoStartTry(1);assert(!run_switch);
    run_auto_pending=1;Run_AutoStartTry(1);feed("S1=0!");stopped(REMOTE_REASON_MANUAL);
    TrackRemote_Frame(1);Run_AutoStartTry(1);assert(!run_switch);
    run_auto_pending=1;run_enable=0;feed("S1=0!");assert(!run_auto_pending);
    run_enable=1;TrackRemote_Frame(1);Run_AutoStartTry(1);assert(!run_switch);
    // A radio initialization failure does not prevent camera-based automatic start.
    init_failed=1;assert(TrackRemote_Init());run_auto_pending=1;
    TrackRemote_Frame(1);Run_AutoStartTry(1);tick(1,80);assert(run_switch);
    near(MotorPID_GetTargetLeft(),180);Run_RemoteStop(REMOTE_REASON_MANUAL);
    init_failed=0;assert(!TrackRemote_Init());
    feed("S1=1!");assert(!run_switch && TrackRemote_GetRejected()==1);
    run_enable=0;TrackRemote_Frame(1);feed("S1=1!");assert(!run_switch);
    run_enable=1;feed("P1=1.80!D1=2.00!P2=8!I2=0.15!D2=0!V1=170!");
    near(run_p,1.8f);near(run_d,2);near(run_kp,8);near(run_ki,.15f);near(run_kd,0);near(run_speed,170);
    // One scalar command per gain preserves the other two gains.
    feed("P2=9.2!");near(run_kp,9.2f);near(run_ki,.15f);near(run_kd,0);
    feed("I2=0.21!");near(run_kp,9.2f);near(run_ki,.21f);near(run_kd,0);
    feed("D2=0.12!");near(run_kp,9.2f);near(run_ki,.21f);near(run_kd,.12f);
    feed("P2=8!I2=0.15!D2=0!");
    feed("S1=1.000000!");tick(1,500);assert(run_switch && pwm_l && pwm_r);
    assert(TrackRemote_GetCommandAgeMs()>=10000); // No heartbeat or recurring commands required.
    near(MotorPID_GetTargetLeft(),170);near(MotorPID_GetTargetRight(),170);
    // V1 changes the real wheel targets; ordinary speed ramp and 70 floor remain.
    feed("V1=200!");tick(1,80);near(MotorPID_GetTargetLeft(),200);near(MotorPID_GetTargetRight(),200);
    feed("V1=120!");tick(1,2);near(MotorPID_GetTargetLeft(),120);near(MotorPID_GetTargetRight(),120);
    feed("V1=70!");tick(1,2);near(MotorPID_GetTargetLeft(),70);near(MotorPID_GetTargetRight(),70);
    feed("V1=240!");tick(1,130);near(MotorPID_GetTargetLeft(),240);near(MotorPID_GetTargetRight(),240);
    feed("V1=170!");tick(1,2);near(run_speed,170);
    // A stop command takes effect in the receive loop, without waiting for a control tick.
    feed("S1=0.000000!");stopped(REMOTE_REASON_MANUAL);
    feed("P1=0!D1=50!");near(run_p,0);near(run_d,50);
    feed("P1=5!D1=0!");near(run_p,5);near(run_d,0);
    feed("P1=1.82\r\nD1=2.02\n");near(run_p,1.82f);near(run_d,2.02f);
    unsigned bad=TrackRemote_GetRejected();
    const char *invalid[]={"P1=nan!","D1=inf!","S1=0.5!","S1=1x!","P2=51!","I2=2.1!","D2=21!",
        "P1=1.7junk!","V1=69!","V1=241!","V1=0!","P1=6!","D1=51!","P2=8,.1!","S1=1=1!",
        "hb!","status!","threshold=15!","p+!","pid=8,.15,0!","stop!","run=1!"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++)feed(invalid[i]);
    assert(TrackRemote_GetRejected()==bad+sizeof(invalid)/sizeof(invalid[0]));assert(!run_switch);
    near(run_p,1.82f);near(run_d,2.02f);near(run_ki,.15f);
    // Fragmented UART packets do not apply before an explicit delimiter.
    TrackRemote_Frame(1);feed("s1");assert(!run_switch);feed("=1");assert(!run_switch);
    feed("!");assert(run_switch);tick(1,3);feed("s1=0!");stopped(REMOTE_REASON_MANUAL);
    // Expired partial start commands, oversized text and binary garbage cannot start a car.
    feed("S1=1");tick(1,52);feed("!");assert(!run_switch);
    char huge[160];const char *tail="S1=1!P1=1.80!";
    memset(huge,'x',sizeof(huge));memcpy(huge+125,tail,strlen(tail)+1);
    feed(huge);assert(!run_switch);near(run_p,1.8f);
    feed("\x80S1=1!D1=2.00!");assert(!run_switch);near(run_d,2);
    // Image-loss protection remains even though link/heartbeat expiry was removed.
    TrackRemote_Frame(1);feed("S1=1!");tick(0,24);assert(run_switch);
    tick(0,1);stopped(REMOTE_REASON_CAMERA);feed("S1=1!");assert(!run_switch);
    TrackRemote_Frame(1);assert(!run_switch);feed("S1=1!");
    for(int i=0;i<25;i++) { TrackRemote_Frame(0);feed("P1=1.80!");pit_handler0(); }
    stopped(REMOTE_REASON_CAMERA);
    // RX overflow discards the damaged command batch and stops immediately.
    TrackRemote_Frame(1);feed("S1=1!");tick(1,3);
    rx_errors++;feed("S1=1!P1=2!");stopped(REMOTE_REASON_RX);near(run_p,1.8f);
    feed("!P1=1.80!");assert(!run_switch);feed("S1=1!");assert(run_switch);
    // Existing roundabout fault/retry must still wait for a fresh processed image.
    tick(1,3);round_fault=1;pit_handler0();stopped(REMOTE_REASON_ROUNDABOUT);
    TrackRemote_Frame(1);feed("S1=1!");assert(round_pending);
    pit_handler0();assert(!pwm_l && !pwm_r);
    round_fault=round_pending=0;Island_State=0;TrackRemote_Frame(1);pit_handler0();assert(MotorPID_IsEnabled());
    // Busy radio transmission never blocks command reception or control.
    rts_busy=1;unsigned before=tx_size;tick(1,12);Run_RemoteService();assert(tx_size==before);
    feed("S1=0!");stopped(REMOTE_REASON_MANUAL);rts_busy=0;
    assert(!strcmp(screen_motion,"STOP") && !screen_waiting);
    // Drain any old row, then verify exactly four columns from distinct live values.
    for(int i=0;i<60;i++)Run_RemoteService();
    TrackRemote_Tick20ms(0);TrackRemote_Tick20ms(0);
    ErrUse=-7.25f;enc_l=123;enc_r=98;tx_size=0;feed("V1=180!");
    assert(TRACK_REMOTE_CHANNELS==4);
    for(int i=0;i<60;i++)Run_RemoteService();
    transmitted[tx_size]=0;char *start=transmitted,*last=NULL;
    for(char *p=transmitted;*p;p++)if(*p=='\n') { if(p>start)last=start;start=p+1; }
    assert(last);float values[TRACK_REMOTE_CHANNELS];char *p=last;
    for(int i=0;i<TRACK_REMOTE_CHANNELS;i++) {
        char *end;values[i]=strtof(p,&end);assert(end!=p && isfinite(values[i]));
        assert(*end==(i+1==TRACK_REMOTE_CHANNELS ? '\n':','));p=end+1;
    }
    near(values[0],-7.25f);near(values[1],123);near(values[2],98);near(values[3],180);
    assert(TrackRemote_GetAccepted()>0 && TrackRemote_GetRejected()>0 && TrackRemote_GetRxBytes()>0);
    // Initialization failure never permits a start, even with a valid camera frame.
    init_failed=1;assert(TrackRemote_Init());TrackRemote_Frame(1);assert(!TrackRemote_Arm());
    puts("PASS: auto180 after first usable frame, no startup image race, latched stops, radio-independent auto start; seven commands, independent gains, four live FireWater channels and simple screen state");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='rt1064_wireless_track_') as directory:
    build=Path(directory)
    (build/'zf_common_headfile.h').write_text(stub,encoding='utf-8')
    (build/'zf_device_wireless_uart.h').write_text(wireless_stub,encoding='utf-8')
    (build/'check.c').write_text(config+base['PREFIX']+EXTRA+block('RUN AUTO START')+block('WIRELESS TRACK CONTROL')+
        block('STEERING BOOST')+block('RUN CURVE CONTROL')+sampler+controller+TEST,encoding='utf-8')
    exe=build/'check.exe'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-I',str(build),
        '-I',str(code_dir),str(build/'check.c'),str(code_dir/'TrackRemote.c'),
        str(code_dir/'MotorPID.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
