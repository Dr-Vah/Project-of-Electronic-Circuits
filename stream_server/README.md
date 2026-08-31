# ESP32-S3 摄像头 Wi-Fi 实时调试服务器

该模块将 ESP32-S3 从 USB UVC 摄像头收到的 MJPEG 帧实时发送到电脑。
ESP32-S3 会建立独立 Wi-Fi 热点并提供浏览器预览，因此不占用摄像头使用的
GPIO19/GPIO20，也不需要安装 Python、OpenCV 或桌面客户端。

## 使用方法

1. 将 `camera_stream_server.c` 和 `camera_stream_server.h` 放入 ESP-IDF
   工程的 `main` 目录。
2. 在 `main/CMakeLists.txt` 的 `SRCS` 中加入
   `camera_stream_server.c`，并在 `PRIV_REQUIRES` 中加入：

   ```cmake
   esp_wifi esp_event esp_netif esp_http_server nvs_flash esp_psram
   ```

3. 在程序初始化阶段调用：

   ```c
   ESP_ERROR_CHECK(camera_stream_server_init());
   ```

4. 每收到一帧完整的 MJPEG 数据后调用：

   ```c
   camera_stream_server_publish_jpeg(frame->data, frame->data_len);
   ```

   此函数会把 JPEG 复制到 PSRAM，网络发送由 HTTP 任务完成，不会在摄像头
   回调中等待浏览器。

5. 烧写并启动 ESP32-S3，电脑连接以下热点：

   - Wi-Fi：`ESP32-Camera-Debug`
   - 密码：`camera123`
   - 预览地址：`http://192.168.4.1/`

电脑提示“此网络无 Internet”属于正常现象，请保持热点连接。

## 工作方式

- 网页的 `/stream` 接口使用标准 MJPEG multipart 流。
- `/status` 返回累计帧数和最新 JPEG 大小。
- 始终只保留最新一帧；网络变慢时跳过旧帧，不反向阻塞 USB 摄像头。
- JPEG 缓冲区和每个浏览器客户端的发送副本均从 PSRAM 分配。
- 默认允许两台设备连接热点，但建议调试时只打开一个视频页面。

## 修改热点

热点名称、密码、信道和最大客户端数位于
`camera_stream_server.c` 文件开头。WPA2 密码不得少于 8 个字符。

## 环境要求

- ESP-IDF 5.4.x
- 带 PSRAM 的 ESP32-S3
- 摄像头输出格式为完整 JPEG/MJPEG 帧

本目录是可复用模块，不包含完整摄像头枚举程序。完整工程需要在 UVC 帧回调或
后续处理任务中调用上述发布接口。
