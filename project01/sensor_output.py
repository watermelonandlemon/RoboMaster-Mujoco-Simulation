# -*- coding: utf-8 -*-
import json
import math
import queue
import socket
import struct
import threading
import time
import zlib

HOST = "127.0.0.1"
SENSOR_PORT = 18980
INTERFACE_HZ = 30.0
INTERFACE_PERIOD_S = 1.0 / INTERFACE_HZ
TOPIC_STATE = "dart/state"
TOPIC_IMU = "dart/imu"
TOPIC_IMAGE = "dart/image"
TOPIC_VISION_GT = "dart/vision_gt"
_ALL_TOPICS = (TOPIC_STATE, TOPIC_IMU, TOPIC_IMAGE, TOPIC_VISION_GT)
SUBSCRIPTION_TTL_S = 5.0
UDP_IMAGE_CHUNK_BYTES = 12000
PROTOCOL_VERSION = 1
_IMAGE_HEADER = struct.Struct("!4sIHHHHd")
_IMAGE_MAGIC = b"RMIM"



class SensorPublisher:
    """仿真端 UDP 发布器，管理订阅者并异步发送四类输出话题。"""

    def __init__(self, host=HOST, sensor_port=SENSOR_PORT):
        self.host = host
        self.sensor_port = int(sensor_port)
        self._stop = threading.Event()
        self._sensor_queue = queue.Queue(maxsize=1)
        self._subscriptions = {}
        self._subscriptions_lock = threading.Lock()
        self._sensor_socket = None
        self._subscriber_thread = None
        self._sender_thread = None
        self._closed = False

        try:
            self._sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sensor_socket.bind((self.host, self.sensor_port))
            self._sensor_socket.settimeout(0.2)
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                try:
                    self._sensor_socket.ioctl(socket.SIO_UDP_CONNRESET, False)
                except OSError:
                    pass

            self._subscriber_thread = threading.Thread(
                target=self._subscription_loop,
                daemon=True,
                name="DartSensorSubscribersUDP",
            )
            self._sender_thread = threading.Thread(
                target=self._sensor_send_loop,
                daemon=True,
                name="DartSensorOutputUDP",
            )
            self._subscriber_thread.start()
            self._sender_thread.start()
        except Exception:
            self.close()
            raise

    def _snapshot_subscriptions(self, topic=None):
        now = time.monotonic()
        with self._subscriptions_lock:
            for address, entry in list(self._subscriptions.items()):
                if now - entry["last_seen"] > SUBSCRIPTION_TTL_S:
                    del self._subscriptions[address]
            if topic is None:
                return list(self._subscriptions.items())
            return [
                (address, entry)
                for address, entry in self._subscriptions.items()
                if topic in entry["topics"]
            ]

    @property
    def sensor_connected(self):
        return bool(self._snapshot_subscriptions())

    @property
    def image_subscribed(self):
        return bool(self._snapshot_subscriptions(TOPIC_IMAGE))

    @property
    def vision_subscribed(self):
        return bool(self._snapshot_subscriptions(TOPIC_VISION_GT))

    def _subscription_loop(self):
        while not self._stop.is_set():
            try:
                raw, address = self._sensor_socket.recvfrom(2048)
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
                        self._subscriptions.pop(address, None)
                    continue
                if command != "subscribe":
                    continue

                topics = message.get("topics", [])
                if not isinstance(topics, list) or not topics:
                    continue
                topics = frozenset(topic for topic in topics if topic in _ALL_TOPICS)
                if not topics:
                    continue

                fields = message.get("fields")
                if not isinstance(fields, list):
                    fields = None
                else:
                    fields = frozenset(
                        field for field in fields
                        if isinstance(field, str) and len(field) < 80
                    )

                with self._subscriptions_lock:
                    self._subscriptions[address] = {
                        "topics": topics,
                        "fields": fields,
                        "last_seen": time.monotonic(),
                    }
            except (ValueError, TypeError, UnicodeError):
                continue

    def publish_sensor(self, meta, image_rgb=None, vision_gt=None):
        """提交最新一帧；网络发送、JSON 编码和图像压缩均在后台线程完成。"""
        if not self.sensor_connected:
            return False
        item = (
            dict(meta),
            image_rgb,
            dict(vision_gt) if vision_gt is not None else None,
        )
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

    def _sendto(self, data, address):
        try:
            self._sensor_socket.sendto(data, address)
            return True
        except OSError:
            return False

    def _sensor_send_loop(self):
        while not self._stop.is_set():
            try:
                meta, image_rgb, vision_gt = self._sensor_queue.get(timeout=0.15)
            except queue.Empty:
                continue

            states = self._snapshot_subscriptions(TOPIC_STATE)
            images = self._snapshot_subscriptions(TOPIC_IMAGE)
            imus = self._snapshot_subscriptions(TOPIC_IMU)
            visions = self._snapshot_subscriptions(TOPIC_VISION_GT)

            if states or imus or (visions and vision_gt is not None):
                header = {
                    "version": PROTOCOL_VERSION,
                    "frame_id": int(meta.get("frame_id", 0)) & 0xffffffff,
                    "time_s": float(meta.get("time_s", 0.0)),
                }
                values = {
                    key: value for key, value in meta.items()
                    if key not in ("frame_id", "time_s")
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
                        key: value for key, value in values.items() if key in imu_keys
                    }),
                    (TOPIC_VISION_GT, visions if vision_gt is not None else [], vision_gt),
                ):
                    for address, entry in subscribers:
                        fields = entry["fields"]
                        cache_key = (topic, tuple(fields) if fields is not None else None)
                        if cache_key not in payload_cache:
                            selected = source if fields is None else {
                                key: value for key, value in source.items() if key in fields
                            }
                            packet = dict(header, topic=topic, data=selected)
                            payload_cache[cache_key] = json.dumps(
                                packet,
                                ensure_ascii=False,
                                separators=(",", ":"),
                            ).encode("utf-8")
                        self._sendto(payload_cache[cache_key], address)

            if not images or image_rgb is None:
                continue
            try:
                height, width, channels = image_rgb.shape
                if channels != 3 or width > 65535 or height > 65535:
                    continue
                compressed = zlib.compress(image_rgb.tobytes(), level=1)
                count = max(1, (len(compressed) + UDP_IMAGE_CHUNK_BYTES - 1)
                            // UDP_IMAGE_CHUNK_BYTES)
                if count > 65535:
                    continue

                frame_id = int(meta.get("frame_id", 0)) & 0xffffffff
                image_time = meta.get("image_time_s")
                sim_time = float(meta.get("time_s", 0.0) if image_time is None else image_time)
                for index in range(count):
                    chunk = compressed[
                        index * UDP_IMAGE_CHUNK_BYTES:(index + 1) * UDP_IMAGE_CHUNK_BYTES
                    ]
                    wire = _IMAGE_HEADER.pack(
                        _IMAGE_MAGIC, frame_id, index, count, width, height, sim_time
                    ) + chunk
                    for address, _entry in images:
                        self._sendto(wire, address)
                    if index % 8 == 7:
                        time.sleep(0)
            except (TypeError, ValueError, OverflowError, zlib.error):
                continue

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._sensor_socket is not None:
            try:
                self._sensor_socket.close()
            except OSError:
                pass
        for worker in (self._subscriber_thread, self._sender_thread):
            if worker is not None and worker.is_alive():
                worker.join(timeout=0.6)


class SensorSubscriber:
    """外部程序使用的订阅客户端；每个客户端拥有独立 UDP 端口。"""

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
        """返回一个事件；超时则返回 None。图像事件含 image_rgb 字节串。"""
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
                if packet.get("version") != PROTOCOL_VERSION:
                    continue
                if (packet.get("topic") in self.topics and
                        packet.get("topic") in (TOPIC_STATE, TOPIC_IMU, TOPIC_VISION_GT)):
                    return packet
            except (ValueError, TypeError, UnicodeError, AttributeError):
                continue
        return None

    def _accept_image_chunk(self, raw):
        if len(raw) < _IMAGE_HEADER.size or TOPIC_IMAGE not in self.topics:
            return None
        magic, frame_id, index, count, width, height, sim_time = _IMAGE_HEADER.unpack_from(raw)
        if (magic != _IMAGE_MAGIC or count < 1 or index >= count or count > 4096 or
                width < 1 or height < 1 or width * height * 3 > 100_000_000):
            return None

        now = time.monotonic()
        for frame_key, frame in list(self._incomplete_frames.items()):
            if now - frame["first_seen"] > 1.0:
                del self._incomplete_frames[frame_key]
        frame = self._incomplete_frames.get(frame_id)
        if frame is None:
            if len(self._incomplete_frames) >= 3:
                oldest = min(
                    self._incomplete_frames,
                    key=lambda key: self._incomplete_frames[key]["first_seen"],
                )
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
        if (frame["count"] != count or frame["width"] != width or
                frame["height"] != height):
            del self._incomplete_frames[frame_id]
            return None

        frame["chunks"][index] = raw[_IMAGE_HEADER.size:]
        if len(frame["chunks"]) != count:
            return None
        del self._incomplete_frames[frame_id]
        try:
            compressed = b"".join(frame["chunks"][chunk_index] for chunk_index in range(count))
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
            }).encode(), self.server)
        except OSError:
            pass
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()
