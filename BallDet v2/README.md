# White/Orange Ball + Black Target Detector

这是从 `BallGoalNav` 独立出来的 ESP32-S3 视觉检测与可视化模块，包含白球、橙球和黑块检测，以及屏幕像素叠加和网页 Canvas 叠加。不包含电机、导航、屏幕、Wi-Fi 或相机驱动。

模块完整保留当前工程中的检测流程：

1. 把输入帧等比例缩小到不超过 320 像素宽，并一次生成灰度、色差和 RGB 中间平面。
2. 用动态半径约束、圆周梯度、亮度和低色度条件搜索白球。
3. 白球连续两帧稳定后，使用 RGB 阈值和连通区域形状检测橙球，同时在画面下部搜索黑色连通区域。
4. 左右半区各保留一个黑块候选；只有同时检测到两个黑块时结果才有效，并选择左侧黑块作为 `target`。
5. 白球、橙球和左侧黑块均使用连续两帧确认及位置平滑；连续丢失两帧后失效。

## 文件

- `ball_target_detector.h`：公开数据结构、检测接口和可视化接口。
- `ball_target_detector.c`：共享预处理、三类检测、跨帧确认、RGB565 绘图、JSON 和网页的全部实现。

这两个源码文件之间没有其他项目内依赖。运行时只依赖 ESP-IDF 提供的 `esp_err.h`、`esp_heap_caps.h` 和 FreeRTOS。

## 输入格式

`bt_detector_process_rgb565()` 接收行优先、无行填充的 RGB565 图像。当前参数是按 `esp_jpeg` 解码并设置 `swap_color_bytes=1` 后的缓冲区标定的，因此函数会交换每个 `uint16_t` 的高低字节再解析颜色。

推荐输入分辨率为 320×240 或 640×480，画面比例应与标定图片一致。摄像头安装高度、俯角或场景照明明显变化时，需要重新标定源码顶部的半径关系和阈值。

## ESP-IDF 集成

把整个文件夹复制到自己的工程中，然后将 `.c` 文件加入组件，并把该目录加入头文件搜索路径。例如：

```cmake
idf_component_register(
    SRCS "main.c" "../WhiteBallBlackTargetDetector/ball_target_detector.c"
    INCLUDE_DIRS "." "../WhiteBallBlackTargetDetector"
    REQUIRES esp_psram
)
```

最小调用示例：

```c
#include "ball_target_detector.h"

static bt_detector_t detector;

void detector_start(void)
{
    bt_detector_init(&detector);
}

void process_camera_frame(const uint16_t *rgb565,
                          uint16_t width, uint16_t height)
{
    bt_detector_result_t result;
    esp_err_t err = bt_detector_process_rgb565(
        &detector, rgb565, width, height, &result);
    if (err != ESP_OK) {
        return;
    }

    if (result.ball_valid) {
        /* result.ball.x, result.ball.y, result.ball.radius */
    }
    if (result.orange_ball_valid) {
        /* result.orange_ball.x/y/radius 是稳定橙球。 */
    }
    if (result.target_valid) {
        /* result.target 是经过两帧确认和平滑后的左侧黑块。 */
    }
    if (result.target_count == 2) {
        /* result.targets[0] 和 [1] 是本帧的左右黑块。 */
    }

    /* 可选：直接在字节交换的 RGB565 显示缓冲区上画标记。 */
    bt_visualize_rgb565(display_rgb565, display_width, display_height, &result);
}
```

`bt_detector_t` 必须在帧与帧之间保持不变，不要每帧重新初始化。接口不是线程安全的，同一个实例只能由一个检测任务顺序调用。

## 输出含义

- `ball_valid`：白球已经连续两帧确认。
- `ball`：稳定白球的圆心、半径、评分和圆周支持点数量。
- `orange_ball_valid`：橙球已经连续两帧确认。
- `orange_ball`：稳定橙球的圆心、等效半径、评分和颜色区域填充率。
- `target_count`：当前帧找到的黑块数量；数量为 2 时，`targets[0]`、`targets[1]` 依次为左、右黑块。数量为 1 时，该候选可能位于任一侧。
- `target_valid`：左侧黑块已经连续两帧确认，并且本检测阶段仍有效。
- `target`：稳定并平滑后的左侧黑块中心、边界框、面积、评分和平均灰度。

所有坐标和尺寸都映射回原始输入图像，而不是内部 320 像素宽的工作图。

## 可视化

### 屏幕 RGB565 叠加

```c
bt_visualize_rgb565(display_pixels, display_width, display_height, &result);
```

函数直接修改调用者提供的字节交换 RGB565 缓冲区。缓冲区可以是原始相机帧，也可以是缩放后的屏幕帧；函数会根据 `result.frame_width` 和 `result.frame_height` 自动缩放坐标。

标记与当前板端画面一致：

- 白球：绿色圆周、红色中心。
- 橙球：橙色圆周、黄色中心。
- 黑块：青色矩形框。
- 稳定左侧目标：从顶部 43% 横坐标处到目标中心的蓝线。
- 小车参考点：顶部 43% 横坐标处的紫色十字。

该函数只负责往像素缓冲区绘图。之后仍由使用者调用自己的 ST7735S 或其他屏幕驱动发送图像。

### 网页叠加

模块内置了当前风格的 HTML/Canvas 页面：

```c
const char *page = bt_detector_visualization_html();
```

将 `page` 作为网页根路径的响应内容。页面会轮询：

- `/frame.jpg`：当前相机 JPEG。
- `/detection`：当前检测 JSON。

JSON 可直接生成：

```c
char json[1200];
if (bt_detector_result_to_json(&result, json, sizeof(json)) == ESP_OK) {
    /* 将 json 作为 application/json 返回。 */
}
```

模块不启动 Wi-Fi 或 HTTP 服务器，使用者可以接入自己的网络框架。这样检测和绘图代码可直接复用，也不会绑定原项目的热点名称、密码或导航状态机。

## 内存与实时性

模块优先从 PSRAM 分配大缓冲区，失败时回退到普通堆。320×240 工作图在黑块阶段的峰值临时内存约为 768 KB，因此建议启用 PSRAM。每帧处理结束前会释放全部临时内存；`bt_detector_t` 只保存少量跨帧状态。

白球检测每遍历一个候选半径会调用一次 `vTaskDelay(1)` 喂看门狗，所以必须从正常 FreeRTOS 任务上下文调用，不能在中断中调用。

## 当前标定参数

- 工作宽度：最大 320 像素。
- 白球半径模型：在 320 宽工作图上约为 `r = 15 - 11*y/height`，搜索带宽 ±5 像素，总范围 6–20 像素。
- 白球稳定确认：2 帧，最大圆心跳变 24 个原图像素。
- 橙球颜色：`R >= 165`、`R-G >= 30`、`G-B >= 5`。
- 橙球连通区域：至少 18 像素，框宽高 5–60 像素，长宽比不超过 2:1，填充率至少 25%。
- 橙球稳定确认：2 帧，最大圆心跳变 32 个原图像素。
- 黑块阈值：灰度不高于 110。
- 黑块搜索区域：横向 3%–97%，纵向 50%–94%。
- 黑块最大框：工作图上 48×20 像素。
- 黑块稳定确认：2 帧，左侧目标最大圆心跳变 120 个原图像素。

这些数值与生成本文件夹时 `BallGoalNav` 中正在使用的版本一致。
