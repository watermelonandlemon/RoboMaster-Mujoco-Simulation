# -*- coding: utf-8 -*-
"""Project02 的 v2.6 UDP v1 客户端，只订阅 dart/state；独立于 Project01。"""
import json
import math
import socket
import time
import uuid

HOST = "127.0.0.1"
SENSOR_PORT = 18980
ACTIVE_PORT = 18981
PASSIVE_PORT = 18982
PROTOCOL_VERSION = 1
TOPIC_STATE = "dart/state"


class ForcePublisher:
    """向仿真端发送主动或被动六维力/力矩。"""

    def __init__(self, channel, host=HOST):
        if channel not in ("active", "passive"):
            raise ValueError("channel 必须为 active/passive")
        self.channel = channel
        self.address = (host, ACTIVE_PORT if channel == "active" else PASSIVE_PORT)
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.last_send_monotonic_s = None
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
        sent_at = time.monotonic()
        packet = {
            "version": PROTOCOL_VERSION,
            "channel": self.channel,
            "session": self.session_id,
            "sequence": self.sequence,
            "force": values,
            "sender_monotonic_s": sent_at,
        }
        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(payload, self.address)
        self.last_send_monotonic_s = sent_at
        return self.sequence

    def close(self):
        if not self.closed:
            self.closed = True
            self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


class SensorSubscriber:
    """仅订阅 dart/state；兼容原版 passive.py 的 state 订阅。"""

    def __init__(self, topics=(TOPIC_STATE,), fields=None, host=HOST, port=SENSOR_PORT):
        if tuple(topics) != (TOPIC_STATE,):
            raise ValueError("本客户端只订阅 dart/state")
        self.topics = (TOPIC_STATE,)
        self.fields = list(fields) if fields is not None else None
        self.server = (socket.gethostbyname(host), int(port))
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
        """返回完整状态帧，超时返回 None。"""
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
            try:
                packet = json.loads(data.decode("utf-8"))
                if not isinstance(packet, dict) or packet.get("version") != PROTOCOL_VERSION:
                    continue
                topic = packet.get("topic")
                if topic == TOPIC_STATE:
                    return packet
            except (ValueError, TypeError, UnicodeError, AttributeError):
                continue
        return None

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


class FrameTracker:
    """区分新帧、重置、暂停、乱序；绝不将真实时间用于目标位置差分。"""
    def __init__(self):
        self.frame = self.sim = self.received = None

    def accept(self, event, now):
        frame, sim = event.get("frame_id"), event.get("time_s")
        if (type(frame) is not int or not 0 <= frame < 2**32 or
                type(sim) not in (int, float) or not math.isfinite(sim) or sim < 0):
            raise ValueError("无效的 frame_id/time_s")
        sim = float(sim)
        elapsed = None if self.received is None else now - self.received
        dt = None if self.sim is None else sim - self.sim
        delta = None if self.frame is None else (frame - self.frame) % 2**32
        if delta is None:
            status = "first"
        elif delta == 0:
            status = "restart" if elapsed > 1.0 and dt < -1e-8 else "duplicate"
        elif delta >= 2**31:
            status = "restart" if elapsed > 1.0 and dt <= 0 else "out_of_order"
        elif dt < -1e-8:
            status = "reset"
        elif abs(dt) <= 1e-8:
            status = "same_sim_time"
        else:
            status = "forward"
        accepted = status not in ("duplicate", "out_of_order")
        if accepted:
            self.frame, self.sim, self.received = frame, sim, now
        return status, {"receive_interval_s": elapsed, "sim_interval_s": dt,
                        "missing_frame_count": delta - 1 if delta is not None and
                        0 < delta < 2**31 and status not in ("reset", "restart") else None}, accepted
