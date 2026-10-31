# 本工程的修改范围

- 当前项目与后续修改目录为 `E:\ceshi\smartcar-rt1064-diankong`。
- `E:\ceshi\island` 是旧环岛工程参考；不要在该目录修改或把旧电控文件覆盖过来。
- 主 Keil 工程为 `project/mdk/rt1064.uvprojx`，目标 `nor_sdram_zf_dtcm`。
- 修改前从该工程 XML 核对活动源码路径。目前为 `project/restore_s1_R1/code/` 和 `project/restore_s1_R1/user/`；普通 `project/code/`、`project/user/` 不是主工程活动源码。
- 环岛实现为活动目录中的 `island.c/.h`，通过 `Roundabout.c/.h` 接入队友电控框架。保留纯视觉进出环，不引入角度强制打舵或正常出环。
- 电控以本目录队友版本为准。未经用户后续要求，不替换主程序、PID、舵机标定、引脚、基础巡线、十字、按键、无线配置或显示代码。
- 当前底版是 KEY RUN R3：上电停车，C13启停，C12选择SPD/P/D，C15减、C14加，无线默认关闭。不要把旧 VOFA 配置恢复过来。
- 验证要读取活动源码；优先运行按键、显示、控制集成、弯道、舵机、环岛与适配检查。`tests/run_vision_tests.py` 为历史测试，硬编码的普通 `project/code/` 不代表活动程序。
- 构建只编译，不自动烧录或发车。输出为 `project/mdk/Objects/keys_R3/rt1064_keys_R3.axf`；`firmware/` 中旧二进制不自动代表当前源码。
- 合并记录见 `docs/环岛算法合并说明.md`。原环岛模块备份在 `work/ring-transfer/backup/`。
