#ifndef TRACK_REMOTE_H
#define TRACK_REMOTE_H
#include "zf_common_headfile.h"

#define TRACK_REMOTE_CHANNELS        4 /* ErrUse, left encoder, right encoder, SPD */
#define TRACK_REMOTE_CAMERA_TICKS   25U /* 500ms without usable image */
#define TRACK_REMOTE_WAVE_TICKS      2U /* 25Hz, independent of camera */

typedef enum {
    REMOTE_STOP=0, REMOTE_RUN, REMOTE_P, REMOTE_D,
    REMOTE_MOTOR_P, REMOTE_MOTOR_I, REMOTE_MOTOR_D, REMOTE_SPEED, REMOTE_RX_FAULT
} TrackRemote_Command;
enum {
    REMOTE_REASON_READY=0, REMOTE_REASON_MANUAL=1, REMOTE_REASON_RESERVED=2,
    REMOTE_REASON_CAMERA=3, REMOTE_REASON_RX=4, REMOTE_REASON_ROUNDABOUT=5,
    REMOTE_REASON_INIT=6
};
typedef uint8 (*TrackRemote_Handler)(TrackRemote_Command command,const float *values);

uint8 TrackRemote_Init(void);
void TrackRemote_Poll(TrackRemote_Handler handler);
void TrackRemote_Send(const float *wave); /* Main loop only; <=30 bytes per call. */
uint8 TrackRemote_Tick20ms(uint8 moving); /* ISR; returns stop reason, or zero. */
void TrackRemote_Frame(uint8 usable);
uint8 TrackRemote_Arm(void); /* A new run command requires a recent usable image. */
void TrackRemote_Stop(uint8 reason);
uint8 TrackRemote_GetReason(void);
uint8 TrackRemote_ImageUsable(void);
uint32 TrackRemote_GetAccepted(void);
uint32 TrackRemote_GetRejected(void);
uint32 TrackRemote_GetRxBytes(void);
uint32 TrackRemote_GetFrameAgeMs(void);
uint32 TrackRemote_GetCommandAgeMs(void);
#endif
