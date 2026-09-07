# 黑区检测主机回归

在提供 GCC 的终端中，从 `IntegratedRobot` 目录运行：

```powershell
gcc -std=c11 -O0 -I tests/host tests/host/test_black_targets.c -lm -o build-verify/test_black_targets.exe
./build-verify/test_black_targets.exe
```

输出目录 `build-verify` 需已存在。测试包含实际的 `main/ball_target_detector.c` 和 `main/ball_transport_controller.c`；桩头文件替代 ESP 平台设施，模拟底盘函数记录停车和速度命令，不参与固件编译，也不模拟实际电机运动。测试检查球和目标均无效时仍能凭车前黑色信号触发收尾动作：白球补推后停车，橙球直接停车，并检查等待场景阶段不会被该信号误判为运送完成。

最新流程测试还覆盖：白球单次 800 ms 补推及定时停车、原有后退时长、后退完成后请求橙球、约 30° 左转期间不提前结束、转完先停车再搜索，以及橙球直接停车并短后退结束。屏蔽测试验证橙球阶段恢复左侧并屏蔽右侧，检测器重置也不会恢复旧白球黑区。
