# -*- coding: utf-8 -*-
"""Project02 与 MuJoCo 仿真端之间的 UDP 通信客户端。

本模块只实现 Project02 侧的网络收发，不导入 Project01 的任何 Python 文件。
通信字段、Topic、端口及图像分包格式与 Project01 的新接口保持一致。
"""

import json
import math
import socket
import struct
import time
import uuid
import zlib


# 与 Project01 的 control_input.py、sensor_output.py 保持一致。
HOST = "127.0.0.1"
SENSOR_PORT = 18980
ACTIVE_PORT = 18981
PASSIVE_PORT = 18982
PROTOCOL_VERSION = 1

TOPIC_STATE = "dart/state"
TOPIC_IMU = "dart/imu"
TOPIC_IMAGE = "dart/image"
TOPIC_VISION_GT = "dart/vision_gt"
_ALL_TOPICS = (TOPIC_STATE, TOPIC_IMU, TOPIC_IMAGE, TOPIC_VISION_GT)

_IMAGE_HEADER = struct.Struct("!4sIHHHHd")
_IMAGE_MAGIC = b"RMIM"


class ForcePublisher:
    """向仿真端发送主动或被动六维力/力矩。"""

    def __init__(self, channel, host=HOST):
        if channel not in ("active", "passive"):
            raise ValueError("channel 必须为 active/passive")
        self.channel = channel
        self.address = (host, ACTIVE_PORT if channel == "active" else PASSIVE_PORT)
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.closed = False

    def send(self, force):
        """发送一组 [Fx, Fy, Fz, Mx, My, Mz]，返回递增序号。"""
        if self.closed:
            raise RuntimeError("ForcePublisher 已关闭")
        values = tuple(float(value) for value in force)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("必须发送六个有限数值")

        self.sequence += 1
        packet = {
            "version": PROTOCOL_VERSION,
            "channel": self.channel,
            "session": self.session_id,
            "sequence": self.sequence,
            "force": values,
            "sender_monotonic_s": time.monotonic(),
        }
        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(payload, self.address)
        return self.sequence

    def close(self):
        if not self.closed:
            self.closed = True
            self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


def send_force_loop(channel, values, hz=30.0):
    """按指定频率持续发送固定六维力/力矩；Ctrl+C 停止。"""
    hz = float(hz)
    if not math.isfinite(hz) or hz <= 0.0:
        raise ValueError("发送频率必须是大于零的有限数值")
    values = tuple(float(value) for value in values)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("必须发送六个有限数值")

    period = 1.0 / hz
    next_send = time.perf_counter()
    with ForcePublisher(channel) as publisher:
        print(f"[{channel}] 发送 {publisher.address[0]}:{publisher.address[1]}，频率 {hz:g} Hz，六维值 {values}")
        print("停止发送不会自动清零；需要归零时请主动发送六维零值。Ctrl+C 停止。")
        try:
            while True:
                publisher.send(values)
                print("[" + ",".join(str(value) for value in values) + "]", flush=True)
                next_send += period
                delay = next_send - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_send = time.perf_counter()
        except KeyboardInterrupt:
            print(f"\n[{channel}] 已停止发送；仿真端保持上一次收到的有效值")


class SensorSubscriber:
    """订阅仿真端状态、IMU、图像和视觉真值 UDP Topic。

    每个订阅客户端使用独立本地 UDP 端口。host/port 指仿真端地址，
    不依赖 Project01 与 Project02 的文件夹位置。
    """

    def __init__(self, topics=(TOPIC_IMAGE, TOPIC_IMU, TOPIC_STATE, TOPIC_VISION_GT),
                 fields=None, host=HOST, port=SENSOR_PORT):
        topics = tuple(topics)
        if not topics or any(topic not in _ALL_TOPICS for topic in topics):
            raise ValueError("topics 只能是 dart/image、dart/imu、dart/state、dart/vision_gt")

        self.topics = topics
        self.fields = list(fields) if fields is not None else None
        self.server = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.bind(("", 0))
        self.sock.settimeout(0.02)
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            try:
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass

        self._last_subscribe = -math.inf
        self._incomplete_frames = {}
        self._closed = False
        self._resubscribe(force=True)

    def _resubscribe(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_subscribe < 1.0:
            return
        payload = {
            "version": PROTOCOL_VERSION,
            "command": "subscribe",
            "topics": list(self.topics),
            "fields": self.fields,
        }
        self.sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self.server)
        self._last_subscribe = now

    def recv(self, timeout=0.05):
        """返回一个事件；超时返回 None。图像事件包含 image_rgb 字节串。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while not self._closed:
            self._resubscribe()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.sock.settimeout(min(remaining, 0.05))
            try:
                data, address = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self._closed:
                    return None
                raise

            if address != self.server:
                continue
            if data.startswith(_IMAGE_MAGIC):
                event = self._accept_image_chunk(data)
                if event is not None:
                    return event
                continue

            try:
                packet = json.loads(data.decode("utf-8"))
                if not isinstance(packet, dict) or packet.get("version") != PROTOCOL_VERSION:
                    continue
                topic = packet.get("topic")
                if topic in self.topics and topic in (TOPIC_STATE, TOPIC_IMU, TOPIC_VISION_GT):
                    return packet
            except (ValueError, TypeError, UnicodeError, AttributeError):
                continue
        return None

    def _accept_image_chunk(self, raw):
        if len(raw) < _IMAGE_HEADER.size or TOPIC_IMAGE not in self.topics:
            return None

        magic, frame_id, index, count, width, height, sim_time = _IMAGE_HEADER.unpack_from(raw)
        if (magic != _IMAGE_MAGIC or count < 1 or index >= count or count > 4096
                or width < 1 or height < 1 or width * height * 3 > 100_000_000):
            return None

        now = time.monotonic()
        for key, frame in list(self._incomplete_frames.items()):
            if now - frame["first_seen"] > 1.0:
                del self._incomplete_frames[key]

        frame = self._incomplete_frames.get(frame_id)
        if frame is None:
            if len(self._incomplete_frames) >= 3:
                oldest = min(self._incomplete_frames,
                             key=lambda key: self._incomplete_frames[key]["first_seen"])
                del self._incomplete_frames[oldest]
            frame = {
                "first_seen": now,
                "chunks": {},
                "count": count,
                "width": width,
                "height": height,
                "time_s": sim_time,
            }
            self._incomplete_frames[frame_id] = frame

        if (frame["count"] != count or frame["width"] != width
                or frame["height"] != height):
            del self._incomplete_frames[frame_id]
            return None

        frame["chunks"][index] = raw[_IMAGE_HEADER.size:]
        if len(frame["chunks"]) != count:
            return None

        del self._incomplete_frames[frame_id]
        try:
            compressed = b"".join(frame["chunks"][i] for i in range(count))
            image = zlib.decompress(compressed)
            if len(image) != width * height * 3:
                return None
        except (KeyError, zlib.error):
            return None

        return {
            "topic": TOPIC_IMAGE,
            "frame_id": frame_id,
            "time_s": frame["time_s"],
            "width": width,
            "height": height,
            "image_rgb": image,
        }

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.sock.sendto(json.dumps({
                "version": PROTOCOL_VERSION,
                "command": "unsubscribe",
            }).encode("utf-8"), self.server)
        except OSError:
            pass
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()
