# 本工程协作约定

- 主工程是 `project/mdk/rt1064.uvprojx`，目标 `nor_sdram_zf_dtcm`。先核对工程XML，活动源码在 `project/restore_s1_R1/code/` 和 `project/restore_s1_R1/user/`。
- 用户要求尽量减少main.c改动，便于与队友合并。main主要保留初始化、调度与模块调用；新增控制逻辑放独立模块，必要的接口接入才改main。
- 常用速度、P/D、转向补偿、死区和D滤波参数集中在 `TrackControl.h`，速度策略与恢复状态在 `TrackControl.c`。不要在main重新复制一套参数。
- 转向执行与标定在 `Servo.c/.h`，轮速PID在 `MotorPID.c/.h`，显示在 `Screen.c/.h`，视觉环岛在 `Roundabout.c/.h` 和 `island.c/.h`。
- 原 `Control.c/.h` 是未编入当前工程的旧电控，不能直接恢复其引脚、舵机中值或旧PID。普通 `project/code` 和 `project/user` 也是历史副本，勿改错目录。
- 当前基础速度是直道180、小弯160、大弯140、环岛100；有原差速、240单轮上限、软启动、停车保护。用户明确要求时再调整控制参数，模块拆分本身应保持行为等价。
- Ang是相对当前舵机中位的PWM计数，不是实测角度。正数左、负数右，标定端点在Servo.h。
- 显示、速度或控制变动，运行活动源码的按键、显示、舵机、控制集成、弯道及环岛测试；完整编译只构建，不自动烧录或发车。
- 当前协作修复分支为 `fix/ring-recognition-20261102-202412`。按用户授权同步该分支，不自行将未实车确认的改动合入main。
