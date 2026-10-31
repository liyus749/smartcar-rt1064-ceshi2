> **本目录已接入旧 island 工程一直调试的 VIS7 环岛算法。** 电控仍为下述 KEY RUN R3，按键、显示和无线配置未改。请使用本次重新编译生成的 `project/mdk/Objects/keys_R3/rt1064_keys_R3.axf`；下文 `firmware/` 内预编译文件和旧清单属于合并前底版。
>
> 本次修改与验证见 [环岛算法合并说明](docs/环岛算法合并说明.md)。后续修改均在本目录进行。
# RT1064 智能车：KEY RUN R3

当前默认版本改回逐飞主板按键控制，无线模块关闭。上电停车，C13启动/停止；C12循环选择SPD/P/D，C15减、C14加。基础SPD180、每次±5；舵机P1.80/D2.00，每次±0.02。参数断电恢复默认。

## 编译与烧录

Keil工程 `project/mdk/rt1064.uvprojx`，目标 `nor_sdram_zf_dtcm`，ARMCLANG 6.24。活动源码为 `project/restore_s1_R1/code/` 和 `project/restore_s1_R1/user/`，目录名沿用旧版本。编译输出 `project/mdk/Objects/keys_R3/rt1064_keys_R3.axf`。可直接烧录 [KEY RUN R3固件](firmware/rt1064_keys_R3.axf)，屏幕显示 `KEY RUN R3`。

屏幕显示二值图、ERR、SPD、舵机P/D、左右编码器实际/目标、舵机PWM、十字Cr/Cs、环岛Is/Dir、CAM及停车原因。弯道降速、差速和十字/环岛控制保留。编码器为计数/10ms，不是RPM；基础SPD不同于弯道中每轮的实际目标。

详细键位、标志位和操作见[逐飞主板按键说明](docs/逐飞主板_按键循迹R3.md)。构建与源文件校验见 `firmware/manifest.json`。

## 验证

- `python tests/run_key_control_tests.py`：真实按键代码、参数边界、消抖/长按、启停优先级、丢图停车锁定。
- `python tests/run_simple_display_tests.py`：真实显示代码的数据、选择标记、标志位和240×320布局边界。
- `python tests/run_track_diff_integration_tests.py`：按键/图像超时、控制中断和真实MotorPID集成。
- `python tests/run_curve_control_tests.py`、`python tests/run_steering_tests.py`：弯道/差速和舵机控制。

构建0错误、2个SDK原有浮点警告，尚未代替实车验证。旧AUTO180 R2固件会自动发车，当前请使用KEY RUN R3；旧无线说明仅供历史参考。
