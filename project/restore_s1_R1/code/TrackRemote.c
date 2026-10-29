/* Wireless UART8 tracking controls, VOFA FireWater telemetry. No screen/keys required. */
#include "TrackRemote.h"
#include "zf_device_wireless_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

static volatile uint16 command_age,frame_age;
static volatile uint32 ticks;
static volatile uint8 reason,image_usable;
static uint8 ready,line_length,discard_line,send_now;
static uint32 accepted,rejected,rx_bytes,last_overflow,last_sent,tx_tick,line_tick;
static char line[96],tx[512];
static uint16 tx_length,tx_offset;

static uint32 lock(void) { uint32 m=__get_PRIMASK();__disable_irq();return m; }
static void unlock(uint32 m) { __set_PRIMASK(m); }

uint8 TrackRemote_Init(void)
{
    ready=0;command_age=frame_age=65535U;ticks=0;image_usable=0;
    reason=REMOTE_REASON_READY;accepted=rejected=rx_bytes=0;
    line_length=discard_line=0;tx_length=tx_offset=0;last_sent=line_tick=0;send_now=1;
    if(wireless_uart_init()) { reason=REMOTE_REASON_INIT;return 1; }
    last_overflow=wireless_uart_rx_errors();ready=1;return 0;
}
void TrackRemote_Stop(uint8 why) { reason=why; }
uint8 TrackRemote_GetReason(void) { return reason; }
uint8 TrackRemote_ImageUsable(void) { return image_usable && frame_age<TRACK_REMOTE_CAMERA_TICKS; }
uint32 TrackRemote_GetAccepted(void) { return accepted; }
uint32 TrackRemote_GetRejected(void) { return rejected; }
uint32 TrackRemote_GetRxBytes(void) { return rx_bytes; }
uint32 TrackRemote_GetFrameAgeMs(void) { return (uint32)frame_age*20U; }
uint32 TrackRemote_GetCommandAgeMs(void) { return (uint32)command_age*20U; }
void TrackRemote_Frame(uint8 usable)
{
    uint32 m=lock();image_usable=usable;
    if(usable) { frame_age=0; }
    unlock(m);
}
uint8 TrackRemote_Arm(void)
{
    if(!ready || !TrackRemote_ImageUsable()) { return 0; }
    reason=REMOTE_REASON_READY;return 1;
}
uint8 TrackRemote_Tick20ms(uint8 moving)
{
    ticks++;
    if(command_age<65535U) { command_age++; }
    if(frame_age<65535U) { frame_age++; }
    if(!moving) { return 0; }
    if(frame_age>=TRACK_REMOTE_CAMERA_TICKS) { reason=REMOTE_REASON_CAMERA;return reason; }
    return 0;
}
static int numbers(const char *s,float *v,int n)
{
    int i;char *end;
    for(i=0;i<n;i++)
    {
        v[i]=strtof(s,&end);
        if(s==end || !isfinite(v[i])) { return 0; }
        s=end;while(isspace((unsigned char)*s)) { s++; }
        if(i+1<n) { if(*s!=',') { return 0; } s++; }
    }
    return !*s;
}
static uint8 apply(TrackRemote_Handler handler)
{
    float v[1]={0};char *eq=strchr(line,'=');TrackRemote_Command command;
    if(!eq) { return 0; }
    *eq++=0;
    if(!numbers(eq,v,1)) { return 0; }
    if(!strcmp(line,"s1") && (v[0]==0 || v[0]==1)) { command=v[0]==1 ? REMOTE_RUN:REMOTE_STOP; }
    else if(!strcmp(line,"p1") && v[0]>=0 && v[0]<=5) { command=REMOTE_P; }
    else if(!strcmp(line,"d1") && v[0]>=0 && v[0]<=50) { command=REMOTE_D; }
    else if(!strcmp(line,"p2") && v[0]>=0 && v[0]<=50) { command=REMOTE_MOTOR_P; }
    else if(!strcmp(line,"i2") && v[0]>=0 && v[0]<=2) { command=REMOTE_MOTOR_I; }
    else if(!strcmp(line,"d2") && v[0]>=0 && v[0]<=20) { command=REMOTE_MOTOR_D; }
    else if(!strcmp(line,"v1") && v[0]>=70 && v[0]<=240) { command=REMOTE_SPEED; }
    else { return 0; }
    return handler ? handler(command,v):0;
}
void TrackRemote_Poll(TrackRemote_Handler handler)
{
    uint8 buf[32];uint32 n,i,budget=256,overflow;
    if(!ready) { return; }
    if(line_length && ticks-line_tick>50U)
    { line_length=0;discard_line=1;rejected++; }
    overflow=wireless_uart_rx_errors();
    if(overflow!=last_overflow)
    {
        last_overflow=overflow;line_length=0;discard_line=1;rejected++;
        if(handler) { handler(REMOTE_RX_FAULT,NULL); }
        /* Drop the damaged receive batch, then require a fresh delimiter. */
        while(budget && (n=wireless_uart_read_buffer(buf,budget<sizeof(buf) ? budget:sizeof(buf)))!=0)
        { budget-=n;rx_bytes+=n; }
        return;
    }
    while(budget && (n=wireless_uart_read_buffer(buf,budget<sizeof(buf) ? budget:sizeof(buf)))!=0)
    {
        budget-=n;rx_bytes+=n;
        for(i=0;i<n;i++)
        {
            uint8 c=buf[i];
            if(c=='!' || c=='\r' || c=='\n')
            {
                if(!discard_line && line_length)
                {
                    line[line_length]=0;
                    if(apply(handler))
                    { uint32 m=lock();command_age=0;unlock(m);accepted++; }
                    else { rejected++; }
                }
                line_length=discard_line=0;
            }
            else if(!discard_line)
            {
                if(c<32 || c>126 || line_length>=sizeof(line)-1)
                { line_length=0;discard_line=1;rejected++; }
                else { line[line_length++]=(char)tolower(c);line_tick=ticks; }
            }
        }
    }
}
void TrackRemote_Send(const float *wave)
{
    uint32 now=ticks;uint16 chunk;int i,n;
    if(!ready || !wave) { return; }
    if(tx_offset<tx_length && now-tx_tick>10U)
    {
        /* A paused partial row needs a delimiter before the next complete row. */
        tx[0]='\n';tx_length=1;tx_offset=0;tx_tick=now;send_now=1;
    }
    if(tx_offset==tx_length && (send_now || now-last_sent>=TRACK_REMOTE_WAVE_TICKS))
    {
        tx_length=tx_offset=0;
        for(i=0;i<TRACK_REMOTE_CHANNELS;i++)
        {
            double v=wave[i];
            if(!isfinite(v) || v>1.0e9 || v< -1.0e9) { v=0; }
            n=snprintf(tx+tx_length,sizeof(tx)-tx_length,i+1==TRACK_REMOTE_CHANNELS ? "%.2f\n":"%.2f,",v);
            if(n<0 || n>=(int)(sizeof(tx)-tx_length)) { tx_length=0;return; }
            tx_length+=(uint16)n;
        }
        tx_tick=last_sent=now;send_now=0;
    }
    if(tx_offset>=tx_length || gpio_get_level(WIRELESS_UART_RTS_PIN)) { return; }
    chunk=(uint16)(tx_length-tx_offset);if(chunk>30) { chunk=30; }
    uart_write_buffer(WIRELESS_UART_INDEX,(const uint8 *)tx+tx_offset,chunk);
    tx_offset+=chunk;
}
