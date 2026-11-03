/* Active control tuning. Vision teammates need not edit main.c for these values.
 * Speeds: encoder counts/10ms; steering offset: PWM counts, NOT degrees.
 * Calibration endpoints remain in Servo.h. Preserve them when tuning PD.
 */
#ifndef TRACK_CONTROL_H
#define TRACK_CONTROL_H

#define SERVO_P_DEFAULT     (1.80f)
#define SERVO_P_KEY_STEP    (0.02f)     // C15/C14 调P时每次增减0.02
#define SERVO_P_MIN         (0.0f)
#define SERVO_P_MAX         (5.0f)
#define SERVO_D_DEFAULT     (2.0f)      // 20ms偏差差分，需实车调参
#define SERVO_D_KEY_STEP    (0.02f)
#define SERVO_D_MIN         (0.0f)
#define SERVO_D_MAX         (50.0f)
#define SPEED_DEFAULT       (180.0f)    // 直道180，小弯160，大弯140，环岛100；按键仍可调低基础速度
#define SPEED_KEY_STEP      (5.0f)     // SPD按键每次加减5
#define SPEED_KEY_MIN       (70.0f)
#define SPEED_KEY_MAX       (240.0f)
#define SPEED_KEY_LONG_SCANS (100)     // 仅架空测试：每10ms扫描，长按约1秒
#define CROSS_SLOW_RATIO    (0.7f)     // 过十字时目标速度降到 70%
#define TURN_MEDIUM_LIMIT   (30.0f)    // 舵机输出超过此值开始弯道降速
#define TURN_SHARP_LIMIT    (60.0f)    // 舵机输出超过此值按急弯降速
#define TURN_SPEED_MEDIUM_MAX (160.0f) // 小弯基础目标上限；不再叠加比例减速
#define TURN_SPEED_SHARP_MAX  (140.0f) // 大弯基础目标上限
#define TURN_PREVIEW_START    (0.25f)  // 连续确认的前方弯道开始触发减速
#define TURN_PREVIEW_FULL     (1.50f)  // 预警强度到此值时应用完整入弯速度上限
#define TURN_RELEASE_HOLD_TICKS (10)  // 20ms 一拍，出弯保持低目标 200ms
#define TURN_RELEASE_STEP    (2.0f)   // 其后每拍最多增加 2，即每秒 100 计数/10ms
#define SERVO_U_LIMIT       (120.0f)
#define STEER_DEADBAND      (1.5f)
#define STEER_BOOST_START   (12.0f)
#define STEER_BOOST_K       (0.10f)
#define DIFF_STEER_DEADBAND   (10)     // 小打角不做差速，减少直道/S 弯附加摆动
#define ERROR_DIFF_ON         (12.0f)  // 两种偏差同向达到12像素，明显弯道及时确认
#define ERROR_DIFF_OFF        (8.0f)   // 回到8像素以内退出，保留迟滞
#define ERROR_DIFF_FULL       (24.0f)  // 24像素达到最大60%差速
#define ERROR_DIFF_MIN_LEVEL  (0.40f)  // 介入后至少请求满幅的40%，避免几乎看不出的微小差速
#define ERROR_DIFF_CONFIRM    (2)      // 连续2个新摄像头帧，不能用定时器重复计旧帧
#define ERROR_DIFF_MIN_ROWS   (20)
#define ERROR_DIFF_MAX_AGE     (8)      // 20ms控制周期；约160ms无新图像则撤销
#define ROUND_SPEED_MAX (100.0f) // 独立环岛基础速度上限
#define STEER_D_FILTER_ALPHA (0.5f) // D变化滤波，保持原值；越小越平滑但滞后越大

float Run_CurveTarget(float requested, float steering, float preview);
void Run_SpeedReset(void);
float Run_SpeedTarget(float requested, float steering, float preview);

#endif
