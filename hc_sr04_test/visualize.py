#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HC-SR04 测距实时可视化 (配合 ESP32-S3 hc_sr04_test 固件使用)

用法:
    python visualize.py <串口号> [波特率]

示例:
    python visualize.py COM5
    python visualize.py COM5 115200

依赖安装 (仅需一次):
    pip install pyserial matplotlib
"""
import sys
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial

WINDOW = 40        # 图上保留的采样点数 (滚动窗口)
BAUD = 115200      # 默认波特率, 与 ESP-IDF 串口监视器一致


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {port}: {e}")
        print("      请在设备管理器确认串口号(如 COM3), 并确认开发板已连接。")
        sys.exit(1)

    print(f"[信息] 已连接 {port} @ {baud} baud")
    print("[信息] 图表实时显示距离, 在模块前方挥手即可看到曲线起伏; Ctrl+C 退出。")

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.set_title("HC-SR04 Distance (ESP32-S3)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (cm)")
    line, = ax.plot([], [], "b.-", lw=1.5)
    timestamps, dists = [], []

    def animate(_frame):
        # 一次把所有缓冲数据读完
        while ser.in_waiting:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) < 2:
                continue
            try:
                ms, d = int(parts[0]), float(parts[1])
            except ValueError:
                continue
            if d < 0:
                print(f"[{ms} ms] 测量超时: 距离过远(>5m)或未收到回波")
                continue
            timestamps.append(ms / 1000.0)
            dists.append(d)
            print(f"[{ms} ms] 距离 = {d:.1f} cm")
            # 只保留最近 WINDOW 个点, 曲线向左滚动
            if len(timestamps) > WINDOW:
                del timestamps[:-WINDOW]
                del dists[:-WINDOW]

        line.set_data(timestamps, dists)
        if timestamps:
            ax.set_xlim(timestamps[0], timestamps[-1] + 0.5)
            ax.set_ylim(0, max(dists) * 1.3 + 5)
        return line,

    _ani = animation.FuncAnimation(fig, animate, interval=200, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()