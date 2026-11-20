# -*- coding: utf-8 -*-
"""RoboMaster 飞镖三路进程间通信（仅监听 127.0.0.1）。

传感：TCP 18980，单订阅者，30Hz 目标节拍，RGB24 + zlib；
主动：UDP 18981，30Hz 发送六维力/力矩；
被动：UDP 18982，30Hz 发送六维力/力矩。

本模块不依赖 MuJoCo，也不创建场景。可供不同 Python 进程 import。
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
# 由仿真服务广播的公共 30Hz 周期时钟，输入端在收到 tick 后响应。
ACTIVE_CLOCK_PORT = 18983
PASSIVE_CLOCK_PORT = 18984
INTERFACE_HZ = 30.0
INTERFACE_PERIOD_S = 1.0 / INTERFACE_HZ
STALE_INPUT_S = 0.20
PROTOCOL_VERSION = 1
ZERO_FORCE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
FORCE_KEYS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")


def _read_exact(sock, length):
    parts = bytearray()
    while len(parts) < length:
        chunk = sock.recv(length - len(parts))
        if not chunk:
            raise ConnectionError("远端连接已关闭")
        parts.extend(chunk)
    return bytes(parts)


def make_sensor_frame(header, rgb_bytes=None, image_width=320, image_height=240):
    """构造长度前缀消息：4B总长度 + 4B JSON 长度 + JSON + zlib(RGB)。"""
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
    """独立进程中的传感客户端调用，返回 (metadata, RGB bytes 或 None)。"""
    payload_size = struct.unpack("!I", _read_exact(sock, 4))[0]
    if payload_size < 4 or payload_size > 6_000_000:
        raise ValueError("非法传感数据包长度")
    payload = _read_exact(sock, payload_size)
    meta_len = struct.unpack("!I", payload[:4])[0]
    if not 2 <= meta_len <= payload_size - 4:
        raise ValueError("非法传感 JSON 长度")
    meta = json.loads(payload[4:4+meta_len].decode("utf-8"))
    if meta.get("version") != PROTOCOL_VERSION:
        raise ValueError("接口协议版本不一致")
    encoded = payload[4+meta_len:]
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
    """主动、被动发送端共用；force 是 6 个机体系分量。"""
    if channel not in ("active", "passive"):
        raise ValueError("channel 必须为 active/passive")
    force = tuple(float(v) for v in force)
    if len(force) != 6 or not all(math.isfinite(v) for v in force):
        raise ValueError("必须发送六个有限数值")
    message = {"version": PROTOCOL_VERSION, "channel": channel,
               "session": session_id, "sequence": int(sequence),
               "force": force, "sender_monotonic_s": time.monotonic()}
    return json.dumps(message, separators=(",", ":")).encode("utf-8")


class LocalDartBridge:
    """由 main.py 所在仿真进程启动的服务，关闭输出客户端不影响仿真。"""
    def __init__(self, host=HOST, sensor_port=SENSOR_PORT,
                 active_port=ACTIVE_PORT, passive_port=PASSIVE_PORT,
                 active_clock_port=ACTIVE_CLOCK_PORT,
                 passive_clock_port=PASSIVE_CLOCK_PORT):
        self.host = host
        self.sensor_port = sensor_port
        self.ports = {"active": active_port, "passive": passive_port}
        self.clock_ports = {"active": active_clock_port, "passive": passive_clock_port}
        self._inputs = {}
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._sensor_queue = queue.Queue(maxsize=1)
        self._rx_sockets = {}
        self._listener = None
        self._clock_socket = None
        self._sender_thread = None
        try:
            for channel, port in self.ports.items():
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._rx_sockets[channel] = sock
                sock.bind((host, port))
                sock.setblocking(False)
                self._inputs[channel] = dict(force=ZERO_FORCE, last_rx=-math.inf,
                                             session=None, sequence=-1)
            self._clock_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._listener.bind((host, sensor_port))
            self._listener.listen(1)
            self._listener.settimeout(0.2)
            self._sender_thread = threading.Thread(target=self._sensor_server_loop,
                                                   daemon=True, name="DartSensorTCP")
            self._sender_thread.start()
        except Exception:
            self.close()
            raise

    @property
    def sensor_connected(self):
        return self._connected.is_set()

    def _sensor_server_loop(self):
        while not self._stop.is_set():
            try:
                conn, address = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._connected.set()
            print(f"[通信] 传感接收端已连接：{address}")
            try:
                with conn:
                    conn.settimeout(0.5)
                    # 不发送前一个订阅者残留的数据
                    while True:
                        try:
                            self._sensor_queue.get_nowait()
                        except queue.Empty:
                            break
                    while not self._stop.is_set():
                        try:
                            packet = self._sensor_queue.get(timeout=0.15)
                        except queue.Empty:
                            continue
                        conn.sendall(packet)
            except (ConnectionError, TimeoutError, BrokenPipeError, OSError):
                pass
            finally:
                self._connected.clear()
                print("[通信] 传感接收端已断开，仿真继续运行")

    def publish_sensor(self, meta, image_rgb=None):
        """从仿真主线程发布；慢客户最多缓存一帧，绝不阻塞物理循环。"""
        if not self.sensor_connected:
            return False
        if image_rgb is None:
            packet = make_sensor_frame(meta)
        else:
            height, width, channels = image_rgb.shape
            if channels != 3:
                raise ValueError("期望 RGB(H,W,3) 图像")
            packet = make_sensor_frame(meta, image_rgb.tobytes(), width, height)
        try:
            self._sensor_queue.put_nowait(packet)
        except queue.Full:
            try:
                self._sensor_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._sensor_queue.put_nowait(packet)
            except queue.Full:
                pass
        return True

    def broadcast_tick(self, frame_id, sim_time):
        """30Hz 公共时间基准：两个输入端只响应相同的仿真时刻帧号。"""
        payload = json.dumps({"version": PROTOCOL_VERSION, "tick_id": int(frame_id),
                              "time_s": float(sim_time)},
                             separators=(",", ":")).encode("utf-8")
        for port in self.clock_ports.values():
            try:
                self._clock_socket.sendto(payload, (self.host, port))
            except OSError:
                pass

    def poll_inputs(self, wall_time=None):
        """每个 30Hz 控制周期抓取两路最新值。缺席或超过200ms自动清零。"""
        if wall_time is None:
            wall_time = time.monotonic()
        forces = {}
        for channel, sock in self._rx_sockets.items():
            state = self._inputs[channel]
            while True:
                try:
                    raw, _ = sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                try:
                    packet = json.loads(raw.decode("utf-8"))
                    if packet.get("version") != PROTOCOL_VERSION or packet.get("channel") != channel:
                        continue
                    vals = tuple(float(v) for v in packet["force"])
                    if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
                        continue
                    session = str(packet["session"])
                    sequence = int(packet["sequence"])
                    if session == state["session"] and sequence <= state["sequence"]:
                        continue
                    state.update(force=vals, last_rx=wall_time,
                                 session=session, sequence=sequence)
                except (TypeError, KeyError, ValueError, UnicodeError):
                    continue
            alive = wall_time - state["last_rx"] <= STALE_INPUT_S
            forces[channel] = state["force"] if alive else ZERO_FORCE
        return forces["active"], forces["passive"]

    def clear_inputs(self):
        for state in self._inputs.values():
            state.update(force=ZERO_FORCE, last_rx=-math.inf,
                         session=None, sequence=-1)
        for sock in self._rx_sockets.values():
            while True:
                try:
                    sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break

    def close(self):
        self._stop.set()
        self._connected.clear()
        for sock in self._rx_sockets.values():
            try:
                sock.close()
            except OSError:
                pass
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass
        if self._clock_socket is not None:
            try:
                self._clock_socket.close()
            except OSError:
                pass
        if self._sender_thread is not None and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=0.6)


def send_force_loop(channel, values, hz=INTERFACE_HZ):
    """独立进程监听主程序的公共 30Hz 周期，收到 tick 才发送本通道向量。"""
    if channel not in ("active", "passive"):
        raise ValueError("channel 必须为 active/passive")
    server_port = ACTIVE_PORT if channel == "active" else PASSIVE_PORT
    clock_port = ACTIVE_CLOCK_PORT if channel == "active" else PASSIVE_CLOCK_PORT
    session = uuid.uuid4().hex
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    clock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    clock.bind((HOST, clock_port))
    clock.settimeout(0.5)
    sequence = 0
    last_tick = -1
    print(f"[{channel}] 已启动，监听主程序公共 {hz:g}Hz 时钟 {HOST}:{clock_port}")
    print(f"[{channel}] 发送至 {HOST}:{server_port}，六维向量={tuple(values)}")
    print("主程序未启动时自动等待；Ctrl+C 停止，通道会自动归零。")
    try:
        while True:
            try:
                raw, _ = clock.recvfrom(512)
            except socket.timeout:
                continue
            try:
                payload = json.loads(raw.decode("utf-8"))
                if payload.get("version") != PROTOCOL_VERSION:
                    continue
                tick = int(payload["tick_id"])
            except (ValueError, KeyError, TypeError, UnicodeError):
                continue
            if tick == last_tick:
                continue
            last_tick = tick
            sequence += 1
            sock.sendto(encode_force_packet(channel, values, sequence, session),
                        (HOST, server_port))
    except KeyboardInterrupt:
        print(f"\n[{channel}] 已停止发送")
    finally:
        try:
            for idx in range(3):
                sock.sendto(encode_force_packet(channel, ZERO_FORCE,
                                               sequence+1+idx, session),
                            (HOST, server_port))
        except OSError:
            pass
        clock.close()
        sock.close()


def parse_force_arguments(description):
    """允许编辑示例文件中的向量，或者通过 --fx 等命令行参数覆盖。"""
    import argparse
    parser = argparse.ArgumentParser(description=description)
    for key in FORCE_KEYS:
        parser.add_argument("--" + key.lower(), type=float, default=0.0,
                            help=f"{key} ({'N' if key[0]=='F' else 'N·m'})，默认0")
    args = parser.parse_args()
    return tuple(getattr(args, key.lower()) for key in FORCE_KEYS)
