# -*- coding: utf-8 -*-
"""CSV 记录器：全部日志位于本 Project02，自动由 active/passive 调用。"""
import csv
import json
import math
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
ZERO = (0.0,) * 6
AXES = ("x", "y", "z")
WRENCH = ("fx_N", "fy_N", "fz_N", "mx_Nm", "my_Nm", "mz_Nm")
VECTORS = {
    "position_world_m": ("x", "y", "z"),
    "target_position_world_m": ("x", "y", "z"),
    "linear_velocity_world_mps": ("x", "y", "z"),
    "orientation_quaternion_wxyz": ("w", "x", "y", "z"),
    "orientation_yaw_pitch_roll_deg": ("yaw", "pitch", "roll"),
    "angular_velocity_body_rad_s": ("x", "y", "z"),
    "angular_velocity_body_deg_s": ("x", "y", "z"),
    "acceleration_body_mps2": ("x", "y", "z"),
    **{name: ("fx_N", "fy_N", "fz_N", "mx_Nm", "my_Nm", "mz_Nm") for name in (
        "active_force_body", "passive_force_body", "total_force_body",
        "next_active_force_body", "next_passive_force_body",
    )},
}
SCALARS = ("coordinate_frame", "acceleration_valid", "camera_error", "image_time_s")

EXTRA_VECTORS = {
    "relative_position_world_m": AXES,
    "target_velocity_est_world_mps": AXES,
    "target_acceleration_est_world_mps2": AXES,
    "guidance_acceleration_world_mps2": AXES,
    "command_force_body": WRENCH,
}
COLUMNS = ["record_index", "channel", "algorithm", "event", "frame_id", "time_s",
           "received_wall_time", "received_monotonic_s", "receive_interval_s",
           "sim_interval_s", "missing_frame_count", "send_monotonic_s", "command_sequence",
           "command_session", "range_m", "speed_mps", "closing_mps", "t_go_s",
           "accel_mps2", "target_motion_source", "error"]
COLUMNS += [f"{name}_{axis}" for name, axes in {**VECTORS, **EXTRA_VECTORS}.items() for axis in axes]
COLUMNS += list(SCALARS) + ["raw_packet_json"]


def numeric(value):
    return type(value) in (float, int) and math.isfinite(value)


class RunRecorder:
    def __init__(self, channel, algorithm, config, root=None):
        self.directory = (Path(root) if root is not None else HERE / "logs") / (
            datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + channel)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.channel, self.algorithm = channel, algorithm
        self.run, self.count = 0, 0
        self.file = self.writer = None
        (self.directory / "config.json").write_text(json.dumps(config, ensure_ascii=False,
            indent=2), encoding="utf-8")

    def new_run(self):
        self.close()
        self.run += 1
        self.file = (self.directory / f"run_{self.run:03d}.csv").open(
            "w", encoding="utf-8-sig", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=COLUMNS)
        self.writer.writeheader()
        self.file.flush()

    def write(self, event=None, status="", received=None, timing=None, diagnostics=None,
              command=None, publisher=None, error=""):
        if self.file is None:
            self.new_run()
        self.count += 1
        event = event if isinstance(event, dict) else {}
        data = event.get("data", {})
        data = data if isinstance(data, dict) else {}
        diagnostics = diagnostics or {}
        row = {"record_index": self.count, "channel": self.channel, "algorithm": self.algorithm,
               "event": status, "frame_id": event.get("frame_id"), "time_s": event.get("time_s"),
               "received_wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
               "received_monotonic_s": received, "error": error,
               "raw_packet_json": json.dumps(event, ensure_ascii=False, separators=(",", ":")) if event else ""}
        row.update(timing or {})
        for key in ("range_m", "speed_mps", "closing_mps", "t_go_s", "accel_mps2", "target_motion_source"):
            row[key] = diagnostics.get(key)
        if command is not None and publisher is not None:
            row.update(send_monotonic_s=publisher.last_send_monotonic_s,
                       command_sequence=publisher.sequence, command_session=publisher.session_id)
        values = dict(data)
        values.update(diagnostics)
        values["command_force_body"] = command
        # 相对位置由同一帧两个世界坐标相减，即使本帧不参与控制也可记录。
        p, target = data.get("position_world_m"), data.get("target_position_world_m")
        if all(isinstance(v, (list, tuple)) and len(v) == 3 and all(numeric(x) for x in v)
               for v in (p, target)):
            relative = tuple(t - x for t, x in zip(target, p))
            values["relative_position_world_m"] = relative
            if row["range_m"] is None:
                row["range_m"] = math.hypot(*relative)
        for key, axes in {**VECTORS, **EXTRA_VECTORS}.items():
            vector = values.get(key)
            if isinstance(vector, (list, tuple)) and len(vector) == len(axes):
                row.update({f"{key}_{axis}": val if numeric(val) else None
                            for axis, val in zip(axes, vector)})
        row.update({key: data.get(key) for key in SCALARS})
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = self.writer = None
