#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ble_receiver.py —— 电脑端 BLE 接收脚本
连接 ESP32-S3 (ESP32S3-CAM)，接收 MJPEG 帧分片 -> 重组 -> OpenCV 显示。

依赖安装:
    pip install bleak opencv-python numpy

用法:
    python ble_receiver.py                    # 自动扫描并连接 ESP32S3-CAM
    python ble_receiver.py --addr XX:XX:...   # 按 MAC 地址直连（更快）
    python ble_receiver.py --save frame.jpg   # 额外保存第一帧到文件
    python ble_receiver.py --no-mtu           # 禁用 MTU 协商（仅调试用）

按 q 退出。
"""

import asyncio
import argparse
import sys
import time
from collections import deque

import numpy as np
import cv2
from bleak import BleakClient, BleakScanner

# ==================== 协议常量（与 main/BLE.h / BLE.c 一致） ====================
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FRAME_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
CTRL_CHAR_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

DEVICE_NAME = "ESP32S3-CAM"

SYNC = 0x5A
FLAG_START = 0x01
FLAG_END = 0x02
HDR_SIZE = 6


def hex16_to_uuid(h):
    """0xFFE1 -> '0000ffe1-0000-1000-8000-00805f9b34fb'"""
    return f"0000{h:04x}-0000-1000-8000-00805f9b34fb"


class FrameAssembler:
    """把分片的通知数据重组为完整 JPEG 帧。"""

    def __init__(self):
        self.buf = bytearray()
        self.in_frame = False
        self.expected_seq = 0
        self.frames_received = 0
        self.bad_packets = 0

    def push(self, pkt: bytes):
        """处理一条通知，返回 (frame_bytes, stats) 或 (None, None)。"""
        if len(pkt) < HDR_SIZE:
            return None

        sync = pkt[0]
        flags = pkt[1]
        seq = pkt[2] | (pkt[3] << 8)
        plen = pkt[4] | (pkt[5] << 8)
        payload = pkt[HDR_SIZE: HDR_SIZE + plen]

        if sync != SYNC:
            self.bad_packets += 1
            return None

        if flags & FLAG_START:
            # 新的一帧开始
            self.buf = bytearray(payload)
            self.in_frame = True
            self.expected_seq = seq + 1
        elif self.in_frame:
            # 中间包：可做序号连续性校验（非强制）
            if seq != self.expected_seq:
                self.bad_packets += 1
                self.in_frame = False  # 序号错乱，丢弃本帧
                return None
            self.buf += payload
            self.expected_seq = seq + 1
        else:
            # 没收到 START 就来了中间包，丢弃
            return None

        if flags & FLAG_END:
            frame = bytes(self.buf)
            self.in_frame = False
            self.frames_received += 1
            return frame

        return None

    def create_trackbars(self):#定义创建滑条的函数
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars",0,179,lambda x: None)        
        cv2.createTrackbar("L - S", "Trackbars",0,255,lambda x: None)        
        cv2.createTrackbar("L - V", "Trackbars",0,255,lambda x: None)        
        cv2.createTrackbar("U - H", "Trackbars",179,179,lambda x: None)        
        cv2.createTrackbar("U - S", "Trackbars",255,255,lambda x: None)        
        cv2.createTrackbar("U - V", "Trackbars",255,255,lambda x: None)      



def make_notification_cb(assembler, save_path, stats):
    """构造 bleak 通知回调。"""

    def on_notify(_sender, data: bytearray):
        frame = assembler.push(bytes(data))
        if frame is None:
            return
        
        assembler.create_trackbars()
#--------------------------------------------------------------------------------------------------
        # 解码 JPEG 并显示
        arr = np.frombuffer(frame, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            print("[!] 解码失败（可能不是完整 JPEG），已忽略")
            return

        # 摄像头倒装导致图像颠倒，做 180° 旋转。
        # 若实际只是上下颠倒，改成 cv2.flip(img, 0)；左右镜像用 cv2.flip(img, 1)。
        img = cv2.flip(img, -1)

        now = time.time()
        stats["times"].append(now)
        while stats["times"] and now - stats["times"][0] > 1.0:
            stats["times"].popleft()
        stats["bytes"] += len(frame)

        if save_path and not stats["saved"]:
            with open(save_path, "wb") as f:
                f.write(frame)
            stats["saved"] = True
            print(f"[+] 已保存首帧到: {save_path}")

        # 实时 FPS：最近 1 秒滑动窗口内的平均帧率
        fps = 0.0
        if len(stats["times"]) >= 2:
            fps = (len(stats["times"]) - 1) / (stats["times"][-1] - stats["times"][0])
        cv2.putText(img, f"{fps:5.1f} FPS", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(img, f"{len(frame)} B", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#将BGR图相转换为HSV图像
        l_h = cv2.getTrackbarPos("L - H","Trackbars") 
        l_s = cv2.getTrackbarPos("L - S","Trackbars") 
        l_v = cv2.getTrackbarPos("L - V","Trackbars") 
        u_h = cv2.getTrackbarPos("U - H","Trackbars") 
        u_s = cv2.getTrackbarPos("U - S","Trackbars") 
        u_v = cv2.getTrackbarPos("U - V","Trackbars") 
        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv, lower, upper)   #根据阈值创建掩码
        result = cv2.bitwise_and(img, img, mask=mask)

        cv2.imshow("Original", img)      #显示原始图像
        cv2.imshow("Mask", mask)            #显示掩码图像，掩码可以认为是所需结果
        cv2.imshow("Result", result)        #等待结果图像
        cv2.waitKey(3)                      #等待3ms，刷新图像显示
        
    return on_notify


async def negotiate_mtu(client: BleakClient, mtu: int = 517):
    """跨平台 MTU 协商（Windows=WinRT, Linux=BlueZ, macOS=CoreBluetooth）。"""
    backend = client._backend
    try:
        if hasattr(backend, "_acquire_mtu"):          # Windows (WinRT)
            await backend._acquire_mtu(client.address, mtu)
        elif hasattr(backend, "set_mtu"):             # Linux (BlueZ)
            await backend.set_mtu(client.address, mtu)
        elif hasattr(backend, "setMtu"):              # macOS (CoreBluetooth)
            await backend.setMtu(client.address, mtu)
        else:
            print("[!] 无法识别后端，跳过 MTU 协商")
            return
        print(f"[+] 已请求 MTU = {mtu}")
    except Exception as e:
        print(f"[!] MTU 协商失败（继续用默认 23）: {e}")


async def find_device():
    """扫描并返回目标设备地址。"""
    print(f"[*] 正在扫描 BLE 设备 (查找 {DEVICE_NAME})...")
    devices = await BleakScanner.discover(timeout=8.0, return_adv=True)
    for addr, (dev, adv) in devices.items():
        name = dev.name or adv.local_name or ""
        if name and DEVICE_NAME.lower() in name.lower():
            print(f"[+] 找到: {name} ({addr})")
            return addr
        if DEVICE_NAME.lower() in str(addr).lower():  # 兜底
            return addr
    return None


async def main(args):
    assembler = FrameAssembler()
    stats = {"bytes": 0, "saved": False, "times": deque()}

    addr = args.addr
    if not addr:
        addr = await find_device()
        if not addr:
            print("[x] 未找到 ESP32S3-CAM，请确认设备已上电并开启广播")
            sys.exit(1)

    print(f"[*] 连接 {addr} ...")
    async with BleakClient(addr, timeout=20.0) as client:
        print("[+] 已连接")

        if not args.no_mtu:
            await negotiate_mtu(client)

        # 订阅帧数据特征通知
        cb = make_notification_cb(assembler, args.save, stats)
        await client.start_notify(FRAME_CHAR_UUID, cb)
        print("[+] 已订阅帧通知，等待图像...")

        # 可选：发送 START 命令到控制特征
        if args.start:
            try:
                await client.write_gatt_char(CTRL_CHAR_UUID, bytes([0x01]))
                print("[+] 已发送 START 命令")
            except Exception as e:
                print(f"[!] 发送 START 失败（控制特征可能未启用）: {e}")

        print("[*] 按 q 退出。")
        try:
            while True:
                await asyncio.sleep(0.05)
                # OpenCV 需要主线程不断处理窗口事件
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            await client.stop_notify(FRAME_CHAR_UUID)
            cv2.destroyAllWindows()

    print(f"[*] 共收到 {assembler.frames_received} 帧, "
          f"丢包/坏包 {assembler.bad_packets}, "
          f"总字节 {stats['bytes']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="ESP32-S3 BLE MJPEG 接收显示")
    ap.add_argument("--addr", help="设备 MAC 地址，如 AA:BB:CC:DD:EE:FF")
    ap.add_argument("--save", help="保存首帧到指定路径（如 frame.jpg）")
    ap.add_argument("--start", action="store_true", help="连接后发送 START 命令")
    ap.add_argument("--no-mtu", action="store_true", help="不协商 MTU")
    a = ap.parse_args()

    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("\n[*] 已退出")
