# -*- coding: utf-8 -*-
"""RoboMaster MuJoCo 进程间通信，保留 v2.0 力/力矩协议。

传感输出 UDP 127.0.0.1:18980：注册式发布/订阅，多订阅者独立选择
  dart/state（纯数字 JSON）和 dart/image（分片 zlib-RGB 图像）。
每个订阅者绑定自己的临时 UDP 端口，向 18980 发送订阅请求；无需多个
程序争抢绑定同一端口。传感输出目标节拍仍由原版 lib.py 控制（30 Hz）。
主动输入 UDP 18981；被动输入 UDP 18982，后台每约 10ms 读取一次。
输入永不超时归零；多发布者最新有效包覆盖旧值，停止发送不自动清零。

为与之前替换包及原版 lib.py 兼容，仿真进程会安装两个简单适配：
before_step 使用持续保存的 UDP 输入；只有图像订阅者时才实际离屏渲染。
发送脚本 import 此文件时不会导入 MuJoCo。
"""

import json
import math
import queue
import socket
import struct
import threading
import time
import uuid
import zlib

HOST = "127.0.0.1"
SENSOR_PORT = 18980
ACTIVE_PORT = 18981
PASSIVE_PORT = 18982
# 为兼容旧代码保留常量；新发送端不再使用公共 30Hz 时钟。
ACTIVE_CLOCK_PORT = 18983
PASSIVE_CLOCK_PORT = 18984
INTERFACE_HZ = 30.0
INTERFACE_PERIOD_S = 1.0 / INTERFACE_HZ
INPUT_POLL_INTERVAL_S = 0.010  # 主动/被动后台检查周期 10ms
TOPIC_STATE = "dart/state"  # 全部数字真值（上帝视角，包含 IMU）
TOPIC_IMU = "dart/imu"        # 独立 IMU 输出
TOPIC_IMAGE = "dart/image"
SUBSCRIPTION_TTL_S = 5.0  # 只管理传感器订阅者；不是力输入超时机制
UDP_IMAGE_CHUNK_BYTES = 12000
# magic、frame_id、片序号、片总数、宽、高、仿真时刻
_IMAGE_HEADER = struct.Struct("!4sIHHHHd")
_IMAGE_MAGIC = b"RMIM"
PROTOCOL_VERSION = 1
ZERO_FORCE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
FORCE_KEYS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")

_active_bridge = None
_hook_installed = False


def _read_exact(sock, length):
    parts = bytearray()
    while len(parts) < length:
        chunk = sock.recv(length - len(parts))
        if not chunk:
            raise ConnectionError("远端连接已关闭")
        parts.extend(chunk)
    return bytes(parts)


def make_sensor_frame(header, rgb_bytes=None, image_width=320, image_height=240):
    """与 v2.0 相同：4B总长度 + 4B JSON长度 + JSON + zlib(RGB)。"""
    meta = dict(header)
    meta["version"] = PROTOCOL_VERSION
    if rgb_bytes is None:
        compressed = b""
        meta["image_encoding"] = "none"
        meta["image_width"] = 0
        meta["image_height"] = 0
    else:
        rgb_bytes = bytes(rgb_bytes)
        expected = image_width * image_height * 3
        if len(rgb_bytes) != expected:
            raise ValueError(f"RGB 长度应为 {expected}，实际为 {len(rgb_bytes)}")
        compressed = zlib.compress(rgb_bytes, level=1)
        meta["image_encoding"] = "rgb24-zlib"
        meta["image_width"] = int(image_width)
        meta["image_height"] = int(image_height)
    meta_bytes = json.dumps(meta, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    payload = struct.pack("!I", len(meta_bytes)) + meta_bytes + compressed
    return struct.pack("!I", len(payload)) + payload


def read_sensor_frame(sock):
    """独立传感器接收程序沿用，不需要任何修改。"""
    payload_size = struct.unpack("!I", _read_exact(sock, 4))[0]
    if payload_size < 4 or payload_size > 6_000_000:
        raise ValueError("非法传感数据包长度")
    payload = _read_exact(sock, payload_size)
    meta_len = struct.unpack("!I", payload[:4])[0]
    if not 2 <= meta_len <= payload_size - 4:
        raise ValueError("非法传感 JSON 长度")
    meta = json.loads(payload[4:4 + meta_len].decode("utf-8"))
    if meta.get("version") != PROTOCOL_VERSION:
        raise ValueError("接口协议版本不一致")
    encoded = payload[4 + meta_len:]
    if meta["image_encoding"] == "rgb24-zlib":
        image = zlib.decompress(encoded)
        expected = meta["image_width"] * meta["image_height"] * 3
        if len(image) != expected:
            raise ValueError("传感图像解码长度错误")
    elif meta["image_encoding"] == "none":
        image = None
    else:
        raise ValueError("未知图像编码")
    return meta, image


def encode_force_packet(channel, force, sequence, session_id):
    if channel not in ("active", "passive"):
        raise ValueError("channel 必须为 active/passive")
    force = tuple(float(v) for v in force)
    if len(force) != 6 or not all(math.isfinite(v) for v in force):
        raise ValueError("必须发送六个有限数值")
    message = {
        "version": PROTOCOL_VERSION,
        "channel": channel,
        "session": session_id,
        "sequence": int(sequence),
        "force": force,
        "sender_monotonic_s": time.monotonic(),
    }
    return json.dumps(message, separators=(",", ":")).encode("utf-8")


def _install_interface_hook(bridge):
    """兼容原版 lib.py：输入保值，并跳过无图像订阅者时的离屏渲染。"""
    global _active_bridge, _hook_installed
    import dart_interfaces

    _active_bridge = bridge
    if _hook_installed:
        return

    original_before_step = dart_interfaces.DartInterfaces.before_step
    original_capture = dart_interfaces.DartInterfaces.capture_image_if_due

    def before_step_with_latest_udp(interface):
        current = _active_bridge
        if current is not None:
            active, passive = current.poll_inputs()
            if interface.active_input.as_tuple() != active or interface.active_provider is not None:
                interface.set_active_input(*active)
            if interface.passive_input.as_tuple() != passive or interface.passive_provider is not None:
                interface.set_passive_input(*passive)
        original_before_step(interface)

    def capture_only_when_needed(interface, *args, **kwargs):
        current = _active_bridge
        if current is not None and not current.image_subscribed:
            return False
        return original_capture(interface, *args, **kwargs)

    dart_interfaces.DartInterfaces.before_step = before_step_with_latest_udp
    dart_interfaces.DartInterfaces.capture_image_if_due = capture_only_when_needed
    _hook_installed = True


class LocalDartBridge:
    """MuJoCo 仿真进程：UDP 输入监听 + UDP 传感器发布/订阅。"""

    def __init__(self, host=HOST, sensor_port=SENSOR_PORT,
                 active_port=ACTIVE_PORT, passive_port=PASSIVE_PORT,
                 active_clock_port=ACTIVE_CLOCK_PORT,
                 passive_clock_port=PASSIVE_CLOCK_PORT):
        self.host = host
        self.sensor_port = sensor_port
        self.ports = {"active": active_port, "passive": passive_port}
        self._inputs = {
            "active": {"force": ZERO_FORCE, "sequence_by_session": {}},
            "passive": {"force": ZERO_FORCE, "sequence_by_session": {}},
        }
        self._input_lock = threading.Lock()
        self._stop = threading.Event()
        self._sensor_queue = queue.Queue(maxsize=1)
        self._rx_sockets = {}
        self._subscriptions = {}
        self._subscriptions_lock = threading.Lock()
        self._sensor_socket = None
        self._input_thread = None
        self._subscriber_thread = None
        self._sender_thread = None

        try:
            for channel, port in self.ports.items():
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((host, port))
                sock.setblocking(False)
                self._rx_sockets[channel] = sock

            self._sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sensor_socket.bind((host, sensor_port))
            self._sensor_socket.settimeout(0.2)
            # 避免 Windows 上向已退出的订阅者发送后触发 UDP reset。
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                try:
                    self._sensor_socket.ioctl(socket.SIO_UDP_CONNRESET, False)
                except OSError:
                    pass

            _install_interface_hook(self)

            self._input_thread = threading.Thread(
                target=self._input_loop, daemon=True, name="DartInputUDP100Hz"
            )
            self._subscriber_thread = threading.Thread(
                target=self._subscription_loop, daemon=True, name="DartSensorSubscribersUDP"
            )
            self._sender_thread = threading.Thread(
                target=self._sensor_send_loop, daemon=True, name="DartSensorOutputUDP"
            )
            self._input_thread.start()
            self._subscriber_thread.start()
            self._sender_thread.start()
        except Exception:
            self.close()
            raise

    def _snapshot_subscriptions(self, topic=None):
        now = time.monotonic()
        with self._subscriptions_lock:
            for addr, entry in list(self._subscriptions.items()):
                if now - entry["last_seen"] > SUBSCRIPTION_TTL_S:
                    del self._subscriptions[addr]
            if topic is None:
                return list(self._subscriptions.items())
            return [(addr, entry) for addr, entry in self._subscriptions.items()
                    if topic in entry["topics"]]

    @property
    def sensor_connected(self):
        """兼容 lib.py 中的判断：至少一个传感器订阅者在线。"""
        return bool(self._snapshot_subscriptions())

    @property
    def image_subscribed(self):
        """没有图像订阅者时，禁止不必要的 GPU 离屏渲染。"""
        return bool(self._snapshot_subscriptions(TOPIC_IMAGE))

    def _subscription_loop(self):
        while not self._stop.is_set():
            try:
                raw, addr = self._sensor_socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            try:
                message = json.loads(raw.decode("utf-8"))
                if not isinstance(message, dict) or message.get("version") != PROTOCOL_VERSION:
                    continue
                command = message.get("command")
                if command == "unsubscribe":
                    with self._subscriptions_lock:
                        self._subscriptions.pop(addr, None)
                    continue
                if command != "subscribe":
                    continue
                topics = message.get("topics", [])
                if not isinstance(topics, list) or not topics:
                    continue
                topics = frozenset(t for t in topics if t in (TOPIC_STATE, TOPIC_IMU, TOPIC_IMAGE))
                if not topics:
                    continue
                fields = message.get("fields")
                if not isinstance(fields, list):
                    fields = None
                else:
                    fields = frozenset(f for f in fields if isinstance(f, str) and len(f) < 80)
                with self._subscriptions_lock:
                    self._subscriptions[addr] = {
                        "topics": topics, "fields": fields, "last_seen": time.monotonic()
                    }
            except (ValueError, TypeError, UnicodeError):
                continue

    def _input_loop(self):
        next_poll = time.perf_counter()
        while not self._stop.is_set():
            self._drain_input_packets()
            next_poll += INPUT_POLL_INTERVAL_S
            remaining = next_poll - time.perf_counter()
            if remaining > 0:
                self._stop.wait(remaining)
            else:
                next_poll = time.perf_counter()

    def _drain_input_packets(self):
        for channel, sock in self._rx_sockets.items():
            for _ in range(256):
                try:
                    raw, _ = sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                try:
                    packet = json.loads(raw.decode("utf-8"))
                    if not isinstance(packet, dict):
                        continue
                    if packet.get("version") != PROTOCOL_VERSION:
                        continue
                    if packet.get("channel") != channel:
                        continue
                    vals = tuple(float(v) for v in packet["force"])
                    if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
                        continue
                    session = packet["session"]
                    if not isinstance(session, str) or not session:
                        continue
                    sequence = int(packet["sequence"])
                    if sequence < 0:
                        continue
                    with self._input_lock:
                        state = self._inputs[channel]
                        by_sender = state["sequence_by_session"]
                        if sequence <= by_sender.get(session, -1):
                            continue
                        by_sender[session] = sequence
                        state["force"] = vals
                except (TypeError, KeyError, ValueError, OverflowError, UnicodeError):
                    continue

    def poll_inputs(self, wall_time=None):
        """无等待，最新值无限保持；参数保留以兼容原版 lib.py。"""
        with self._input_lock:
            return self._inputs["active"]["force"], self._inputs["passive"]["force"]

    def clear_inputs(self):
        """兼容原版重置操作：不清除网络保持值。"""
        return None

    def broadcast_tick(self, frame_id, sim_time):
        """兼容原版调用；输入端早已独立发送。"""
        return None

    def publish_sensor(self, meta, image_rgb=None):
        """只把最新帧放入后台队列，不在仿真 GUI 线程中压缩图片。"""
        if not self.sensor_connected:
            return False
        item = (dict(meta), image_rgb)
        try:
            self._sensor_queue.put_nowait(item)
        except queue.Full:
            try:
                self._sensor_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._sensor_queue.put_nowait(item)
            except queue.Full:
                pass
        return True

    def _sendto(self, data, addr):
        try:
            self._sensor_socket.sendto(data, addr)
            return True
        except OSError:
            return False

    def _sensor_send_loop(self):
        while not self._stop.is_set():
            try:
                meta, image_rgb = self._sensor_queue.get(timeout=0.15)
            except queue.Empty:
                continue

            states = self._snapshot_subscriptions(TOPIC_STATE)
            images = self._snapshot_subscriptions(TOPIC_IMAGE)
            imus = self._snapshot_subscriptions(TOPIC_IMU)
            if states or imus:
                # dart/state 发布完整数字真值；dart/imu 只发布 IMU 字段。
                header = {
                    "version": PROTOCOL_VERSION,
                    "frame_id": int(meta.get("frame_id", 0)),
                    "time_s": float(meta.get("time_s", 0.0)),
                }
                values = {
                    k: v for k, v in meta.items()
                    if k not in ("frame_id", "time_s")
                }
                imu_keys = {
                    "coordinate_frame",
                    "acceleration_body_mps2",
                    "angular_velocity_body_rad_s",
                    "acceleration_valid",
                }
                payload_cache = {}
                for topic, subscribers, source in (
                    (TOPIC_STATE, states, values),
                    (TOPIC_IMU, imus, {
                        k: v for k, v in values.items() if k in imu_keys
                    }),
                ):
                    for addr, entry in subscribers:
                        fields = entry["fields"]
                        cache_key = (
                            topic, tuple(fields) if fields is not None else None
                        )
                        if cache_key not in payload_cache:
                            selected = source if fields is None else {
                                k: v for k, v in source.items() if k in fields
                            }
                            packet = dict(header, topic=topic, data=selected)
                            payload_cache[cache_key] = json.dumps(
                                packet, ensure_ascii=False,
                                separators=(",", ":")
                            ).encode("utf-8")
                        self._sendto(payload_cache[cache_key], addr)

            if not images or image_rgb is None:
                continue
            try:
                height, width, channels = image_rgb.shape
                if channels != 3 or width > 65535 or height > 65535:
                    continue
                raw = image_rgb.tobytes()
                compressed = zlib.compress(raw, level=1)
                count = max(1, (len(compressed) + UDP_IMAGE_CHUNK_BYTES - 1)
                            // UDP_IMAGE_CHUNK_BYTES)
                if count > 65535:
                    continue
                frame_id = int(meta.get("frame_id", 0)) & 0xffffffff
                sim_time = float(meta.get("image_time_s") or meta.get("time_s", 0.0))
                # 只有已完成的数据帧会被交付；任意一片丢失就跳过该画面。
                for index in range(count):
                    chunk = compressed[
                        index * UDP_IMAGE_CHUNK_BYTES:(index + 1) * UDP_IMAGE_CHUNK_BYTES
                    ]
                    wire = _IMAGE_HEADER.pack(
                        _IMAGE_MAGIC, frame_id, index, count, width, height, sim_time
                    ) + chunk
                    for addr, _ in images:
                        self._sendto(wire, addr)
                    if index % 8 == 7:
                        time.sleep(0)  # 避免一次性灌满本机 UDP 接收缓冲
            except (TypeError, ValueError, OverflowError, zlib.error):
                continue

    def close(self):
        global _active_bridge
        self._stop.set()
        if _active_bridge is self:
            _active_bridge = None
        for sock in self._rx_sockets.values():
            try:
                sock.close()
            except OSError:
                pass
        if self._sensor_socket is not None:
            try:
                self._sensor_socket.close()
            except OSError:
                pass
        for worker in (self._input_thread, self._subscriber_thread, self._sender_thread):
            if worker is not None and worker.is_alive():
                worker.join(timeout=0.6)


class SensorSubscriber:
    """每个客户端独立订阅某些传感器话题，不与其他客户端竞争端口。"""

    def __init__(self, topics=(TOPIC_IMAGE, TOPIC_IMU, TOPIC_STATE), fields=None,
                 host=HOST, port=SENSOR_PORT):
        topics = tuple(topics)
        if not topics or any(t not in (TOPIC_STATE, TOPIC_IMU, TOPIC_IMAGE) for t in topics):
            raise ValueError("topics 只能是 dart/image、dart/imu 和 dart/state")
        self.topics = topics
        self.fields = list(fields) if fields is not None else None
        self.server = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.bind((host, 0))
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
        self.sock.sendto(json.dumps(payload, separators=(",", ":")).encode(), self.server)
        self._last_subscribe = now

    def recv(self, timeout=0.05):
        """None 或 {topic, frame_id, time_s, data/image_rgb, width, height}。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while not self._closed:
            self._resubscribe()
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            self.sock.settimeout(min(remain, 0.05))
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
                if packet.get("version") != PROTOCOL_VERSION:
                    continue
                if packet.get("topic") in self.topics and packet.get("topic") in (TOPIC_STATE, TOPIC_IMU):
                    return packet
            except (ValueError, TypeError, UnicodeError, AttributeError):
                continue
        return None

    def _accept_image_chunk(self, raw):
        if len(raw) < _IMAGE_HEADER.size or TOPIC_IMAGE not in self.topics:
            return None
        magic, frame_id, index, count, width, height, sim_time = _IMAGE_HEADER.unpack_from(raw)
        if (magic != _IMAGE_MAGIC or count < 1 or index >= count or
                count > 4096 or width < 1 or height < 1 or
                width * height * 3 > 100_000_000):
            return None
        now = time.monotonic()
        for fid, frame in list(self._incomplete_frames.items()):
            if now - frame["first_seen"] > 1.0:
                del self._incomplete_frames[fid]
        frame = self._incomplete_frames.get(frame_id)
        if frame is None:
            if len(self._incomplete_frames) >= 3:
                oldest = min(self._incomplete_frames, key=lambda k:
                             self._incomplete_frames[k]["first_seen"])
                del self._incomplete_frames[oldest]
            frame = {
                "first_seen": now, "chunks": {}, "count": count,
                "width": width, "height": height, "time_s": sim_time
            }
            self._incomplete_frames[frame_id] = frame
        if frame["count"] != count or frame["width"] != width or frame["height"] != height:
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
            "topic": TOPIC_IMAGE, "frame_id": frame_id,
            "time_s": frame["time_s"], "width": width,
            "height": height, "image_rgb": image
        }

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.sock.sendto(json.dumps({
                "version": PROTOCOL_VERSION, "command": "unsubscribe"
            }).encode(), self.server)
        except OSError:
            pass
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()


def send_force_loop(channel, values, hz=30.0):
    """各发布者独立按 hz 发送；停止时不自动发送清零包。"""
    if channel not in ("active", "passive"):
        raise ValueError("channel 必须为 active/passive")
    hz = float(hz)
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError("发送频率必须是大于零的有限数值")
    port = ACTIVE_PORT if channel == "active" else PASSIVE_PORT
    session = uuid.uuid4().hex
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / hz
    next_send = time.perf_counter()
    sequence = 0
    print(f"[{channel}] 发送 {HOST}:{port}，频率 {hz:g} Hz，六维值 {tuple(values)}")
    print("停止发送不会自动清零；要归零请主动发送六维零值。Ctrl+C 停止。")
    try:
        while True:
            sequence += 1
            force = tuple(values)
            sock.sendto(
                encode_force_packet(channel, force, sequence, session),
                (HOST, port),
            )
            # 每次发送后输出本次实际发送的六维数据
            print("[" + ",".join(str(float(v)) for v in force) + "]", flush=True)
            next_send += period
            delay = next_send - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_send = time.perf_counter()
    except KeyboardInterrupt:
        print(f"\n[{channel}] 已停止发送；MuJoCo 保持上一次收到的有效值")
    finally:
        sock.close()


def parse_force_arguments(description):
    """兼容原版示例：支持 --fx、--fy 等六个命令行参数。"""
    import argparse
    parser = argparse.ArgumentParser(description=description)
    for key in FORCE_KEYS:
        parser.add_argument(
            "--" + key.lower(), type=float, default=0.0,
            help=f"{key} ({'N' if key[0] == 'F' else 'N·m'})，默认0",
        )
    args = parser.parse_args()
    return tuple(getattr(args, key.lower()) for key in FORCE_KEYS)
