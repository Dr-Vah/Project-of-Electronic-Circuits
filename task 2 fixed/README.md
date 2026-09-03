# 修改摘要

## 巡线控制

- 摄像头在 ROI（左右各剔除 20%）内计算黑线质心，输出归一化误差 `line_error`（-1..+1，图像右为正）与暗像素计数 `line_pixels`（`camera_line_sensor.h/.c`）。
- `infrared_read()` 生成 `raw_error`（取反，保持“正=线偏左”的历史符号），经 `LINE_ERROR_FILTER_ALPHA` 低通滤波得到 `error`（`main.c`）。
- `track_visible_line()` 由四通道分档改为连续 PD 控制：`omega = -LINE_STEER_KP*error - LINE_STEER_KD*d_error`，并限幅 `LINE_STEER_MAX_OMEGA`、死区 `LINE_STEER_DEADBAND`。

## 避障回线判定

- `AVOID_STRAFE_RIGHT` 回线成功条件由「右侧外通道见线」改为「线在内缩 ROI 内且靠近中心」：`line_pixels != 0 && fabsf(raw_error) < LINE_RETURN_CENTER_ERROR`。

## 显示节流

- `analyse_rgb565()` 中屏幕刷新按 `CAMERA_DISPLAY_PERIOD_US`（100 ms）节流，避免 SPI 刷屏阻塞视觉流水线；图像分析、状态更新仍每帧执行。

## 参数变更

新增（`main.c`）：

- `LINE_STEER_KP 0.40f`
- `LINE_STEER_KD 0.08f`
- `LINE_STEER_MAX_OMEGA 0.20f`
- `LINE_STEER_DEADBAND 0.01f`
- `LINE_ERROR_FILTER_ALPHA 0.60f`
- `LINE_RETURN_CENTER_ERROR 0.60f`

新增（`camera_line_sensor.c`）：

- `CAMERA_DISPLAY_PERIOD_US 100000LL`

修改：

- `OBSTACLE_LEFT_EXTRA_TIME_MS` → 200
- `OBSTACLE_RETURN_SPEED_MPS` → 0.04f

移除：

- 四通道分档转向相关宏 `OUTER_ERROR_THRESHOLD`、`OPPOSITE_OUTER_SAMPLES` 及其分档逻辑。
