#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wifi_receiver_bw.py —— 电脑端通过 WiFi 接收 ESP32-S3 的二值/灰度掩码 (BWImage) 并显示。

ESP32 端流程：get_mask() 得到 BWImage → wifi_stream_push_bw() 推到 /mask。
本脚本循环 GET http://192.168.4.1/mask，把返回的灰度 JPEG 解码显示。

依赖安装:
    pip install opencv-python numpy

用法:
    python wifi_receiver_bw.py                      # 默认 http://192.168.4.1/mask
    python wifi_receiver_bw.py --url http://192.168.4.1/mask

按 q 退出。
"""

import argparse
import time
from collections import deque
import http.client
from urllib.parse import urlsplit

import numpy as np
import cv2


class MaskFetcher:
    """通过一条 keep-alive 持久连接反复 GET /mask，避免每帧重建 TCP。"""

    def __init__(self, url):
        parts = urlsplit(url)
        self.host = parts.hostname or "192.168.4.1"
        self.port = parts.port or 80
        self.path = parts.path or "/"
        self.conn = None

    def get(self):
        """GET 一次，返回完整响应字节；失败或非 200 返回 None。"""
        try:
            if self.conn is None:
                self.conn = http.client.HTTPConnection(
                    self.host, self.port, timeout=3.0)
            self.conn.request("GET", self.path)
            resp = self.conn.getresponse()
            body = resp.read()
            if resp.status != 200:
                return None
            return body
        except Exception:
            self.close()
            return None

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None


def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 WiFi BW 掩码接收显示")
    ap.add_argument("--url", default="http://192.168.4.1/mask",
                    help="/mask 地址")
    args = ap.parse_args()

    print(f"[*] 正在从 {args.url} 拉取掩码...")

    fetcher = MaskFetcher(args.url)

    cv2.namedWindow("BW Mask", cv2.WINDOW_NORMAL)

    frame_times = deque()
    blank = None

    while True:
        body = fetcher.get()

        if body is None:
            # 还没数据或连接失败：显示提示，继续重试
            if blank is None:
                blank = np.zeros((240, 320), dtype=np.uint8)
                cv2.putText(blank, "waiting...", (60, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, 255, 2)
            cv2.imshow("BW Mask", blank)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
            continue

        # 灰度 JPEG，cv2.imdecode 自动识别格式并解码为单通道
        arr = np.frombuffer(body, dtype=np.uint8)
        mask = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            print("[!] 解码失败（可能不是有效 JPEG），已忽略")
            continue

        # 掩码约定：0=黑线(前景)，255=场地(背景)。
        # 若想白线黑底，取消下一行注释反相：
        # mask = 255 - mask

        now = time.time()
        frame_times.append(now)
        while frame_times and now - frame_times[0] > 1.0:
            frame_times.popleft()
        fps = 0.0
        if len(frame_times) >= 2:
            fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
        h, w = mask.shape

        disp = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(disp, f"{fps:5.1f} FPS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(disp, f"{w}x{h}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("BW Mask", disp)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    fetcher.close()
    cv2.destroyAllWindows()
    print("[*] 已退出")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n[*] 已退出")
