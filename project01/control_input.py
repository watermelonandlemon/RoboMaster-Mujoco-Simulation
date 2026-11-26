# -*- coding: utf-8 -*-
import argparse
from dataclasses import dataclass
import json
import math
import socket
import threading
import time
import uuid

HOST = "127.0.0.1"
ACTIVE_PORT = 18981
PASSIVE_PORT = 18982
INPUT_POLL_INTERVAL_S = 0.010  # 后台接收线程约 100 Hz 检查数据
PROTOCOL_VERSION = 1
ZERO_FORCE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
FORCE_KEYS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")


@dataclass(frozen=True)
class ForceTorque:
    """飞行机体系的等效质心力与力矩；力单位 N，力矩单位 N·m。"""

    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(float(value)) for value in self.as_tuple()):
            raise ValueError("接口力和力矩必须全部是有限数字")

    def as_tuple(self):
        return (self.fx, self.fy, self.fz, self.mx, self.my, self.mz)

    def __add__(self, other):
        if not isinstance(other, ForceTorque):
            return NotImplemented
        return ForceTorque(*(a + b for a, b in zip(self.as_tuple(), other.as_tuple())))


def encode_force_packet(channel, force, sequence, session_id):
    """编码主动/被动力输入数据包，保持现有 v1 协议格式。"""
    if channel not in ("active", "passive"):
        raise ValueError("channel 必须为 active/passive")
    force = tuple(float(value) for value in force)
    if len(force) != 6 or not all(math.isfinite(value) for value in force):
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


class ControlInputReceiver:
    """仿真端输入接收器，分别监听主动输入和被动输入 UDP 端口。

    两路输入各自保存最新的有效值；停止发送不会自动归零，多发布者按
    session/sequence 去重，最新收到的有效数据覆盖旧值。
    """

    def __init__(self, host=HOST, active_port=ACTIVE_PORT, passive_port=PASSIVE_PORT):
        self.host = host
        self.ports = {"active": int(active_port), "passive": int(passive_port)}
        self._inputs = {
            "active": {"force": ZERO_FORCE, "sequence_by_session": {}},
            "passive": {"force": ZERO_FORCE, "sequence_by_session": {}},
        }
        self._input_lock = threading.Lock()
        self._stop = threading.Event()
        self._sockets = {}
        self._worker = None
        self._closed = False

        try:
            for channel, port in self.ports.items():
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind((self.host, port))
                sock.setblocking(False)
                self._sockets[channel] = sock

            self._worker = threading.Thread(
                target=self._receive_loop,
                daemon=True,
                name="DartControlInputUDP100Hz",
            )
            self._worker.start()
        except Exception:
            self.close()
            raise

    def _receive_loop(self):
        next_poll = time.perf_counter()
        while not self._stop.is_set():
            self._drain_packets()
            next_poll += INPUT_POLL_INTERVAL_S
            remaining = next_poll - time.perf_counter()
            if remaining > 0:
                self._stop.wait(remaining)
            else:
                next_poll = time.perf_counter()

    def _drain_packets(self):
        for channel, sock in self._sockets.items():
            for _ in range(256):
                try:
                    raw, _address = sock.recvfrom(4096)
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

                    values = tuple(float(value) for value in packet["force"])
                    if len(values) != 6 or not all(math.isfinite(value) for value in values):
                        continue
                    session = packet["session"]
                    if not isinstance(session, str) or not session:
                        continue
                    sequence = int(packet["sequence"])
                    if sequence < 0:
                        continue

                    with self._input_lock:
                        state = self._inputs[channel]
                        sequence_by_session = state["sequence_by_session"]
                        if sequence <= sequence_by_session.get(session, -1):
                            continue
                        sequence_by_session[session] = sequence
                        state["force"] = values
                except (TypeError, KeyError, ValueError, OverflowError, UnicodeError):
                    continue

    def poll_inputs(self):
        """无等待读取两路最新输入，返回 (active, passive)，每路均为六维 tuple。"""
        with self._input_lock:
            return self._inputs["active"]["force"], self._inputs["passive"]["force"]

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        for sock in self._sockets.values():
            try:
                sock.close()
            except OSError:
                pass
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=0.6)


def send_force_loop(channel, values, hz=30.0):
    """独立发送端示例：按指定频率持续发送六维输入，停止时不自动清零。"""
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
            print("[" + ",".join(str(float(value)) for value in force) + "]", flush=True)
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
    """命令行解析辅助函数：支持 --fx、--fy、--fz、--mx、--my、--mz。"""
    parser = argparse.ArgumentParser(description=description)
    for key in FORCE_KEYS:
        unit = "N" if key[0] == "F" else "N·m"
        parser.add_argument(
            "--" + key.lower(),
            type=float,
            default=0.0,
            help=f"{key} ({unit})，默认 0",
        )
    args = parser.parse_args()
    return tuple(getattr(args, key.lower()) for key in FORCE_KEYS)
