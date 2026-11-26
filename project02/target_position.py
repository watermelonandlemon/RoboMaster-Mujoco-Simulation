# -*- coding: utf-8 -*-
"""仅显示基地绿灯像素坐标：python target_position.py；纯JSON输出追加 --console。"""
import argparse
import json
import math
import time
import tkinter as tk

from simulator_link import SensorSubscriber, TOPIC_VISION_GT

BG = "#0d1522"
PANEL = "#172335"
CARD = "#1d2b40"
BORDER = "#2b3b52"
TEXT = "#edf3fb"
MUTED = "#91a3b9"
RED = "#ef4444"
CYAN = "#38bdf8"
VIOLET = "#a78bfa"
DISPLAY_HEIGHT = 240


def console():
    with SensorSubscriber(topics=(TOPIC_VISION_GT,)) as subscriber:
        try:
            while True:
                event = subscriber.recv(timeout=0.2)
                if event is not None:
                    print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)
        except KeyboardInterrupt:
            pass


class PixelView:
    def __init__(self, canvas):
        self.canvas = canvas
        self.width = 640
        self.height = 480
        self.scale = DISPLAY_HEIGHT / self.height
        self.display_width = round(self.width * self.scale)
        self.display_height = DISPLAY_HEIGHT
        self._draw_background()

    def _draw_background(self):
        self.canvas.configure(width=self.display_width, height=self.display_height, bg="white")
        self.canvas.delete("all")
        self.canvas.create_rectangle(
            0, 0, self.display_width - 1, self.display_height - 1,
            fill="#ffffff", outline="#dbe3ed", width=1, tags="background"
        )

    def resize(self, width, height):
        if (not isinstance(width, int) or isinstance(width, bool)
                or not isinstance(height, int) or isinstance(height, bool)
                or width <= 0 or height <= 0):
            return False, False
        if (width, height) == (self.width, self.height):
            return True, False
        self.width, self.height = width, height
        self.scale = DISPLAY_HEIGHT / float(height)
        self.display_height = DISPLAY_HEIGHT
        self.display_width = max(1, int(round(width * self.scale)))
        self._draw_background()
        return True, True

    def clear_marker(self):
        self.canvas.delete("marker")

    def draw(self, u, v):
        self.clear_marker()
        if (not isinstance(u, (int, float)) or isinstance(u, bool)
                or not isinstance(v, (int, float)) or isinstance(v, bool)
                or not math.isfinite(u) or not math.isfinite(v)):
            return
        if not (-0.5 <= u < self.width - 0.5 and -0.5 <= v < self.height - 0.5):
            return

        x = (u + 0.5) * self.scale
        y = (v + 0.5) * self.scale
        arm = 6
        # 延伸至画幅边缘的细虚线
        self.canvas.create_line(0, y, self.display_width, y,
                                fill=RED, dash=(6, 4), width=1, tags="marker")
        self.canvas.create_line(x, 0, x, self.display_height,
                                fill=RED, dash=(6, 4), width=1, tags="marker")
        # 中心正十字
        self.canvas.create_line(max(0, x - arm), y, min(self.display_width, x + arm), y,
                                fill=RED, width=2, tags="marker")
        self.canvas.create_line(x, max(0, y - arm), x, min(self.display_height, y + arm),
                                fill=RED, width=2, tags="marker")
        self.canvas.create_oval(x - 1.5, y - 1.5, x + 1.5, y + 1.5,
                                fill=RED, outline=RED, tags="marker")


def show_panel():
    root = tk.Tk()
    root.title("RoboMaster · 绿灯像素坐标")
    root.configure(bg=BG)
    root.resizable(False, False)

    shell = tk.Frame(root, bg=BG, padx=10, pady=10)
    shell.pack()

    # 简洁标题区
    header = tk.Frame(shell, bg=BG)
    header.pack(fill="x", pady=(0, 9))
    tk.Frame(header, bg=CYAN, width=3, height=28).pack(side="left", padx=(0, 9))
    title_group = tk.Frame(header, bg=BG)
    title_group.pack(side="left", fill="y")
    tk.Label(title_group, text="基地绿灯中心", bg=BG, fg=TEXT,
             font=("Microsoft YaHei UI", 11, "bold"), anchor="w").pack(anchor="w")
    tk.Label(title_group, text="PIXEL POSITION", bg=BG, fg=MUTED,
             font=("Consolas", 7, "bold"), anchor="w").pack(anchor="w", pady=(1, 0))

    # 白色画幅外沿加一圈细边框，画幅本身仍为纯白
    image_border = tk.Frame(shell, bg=BORDER, padx=1, pady=1)
    image_border.pack()
    canvas = tk.Canvas(image_border, bg="white", highlightthickness=0, bd=0)
    canvas.pack()
    view = PixelView(canvas)

    # 坐标显示采用两条紧凑的信息行，长小数完整显示
    coord_panel = tk.Frame(shell, bg=PANEL, padx=7, pady=6)
    coord_panel.pack(fill="x", pady=(9, 0))

    u_row = tk.Frame(coord_panel, bg=CARD)
    u_row.pack(fill="x", pady=(0, 4))
    tk.Label(u_row, text="U", bg="#12334a", fg=CYAN,
             font=("Consolas", 9, "bold"), width=3, pady=4).pack(side="left")
    u_text = tk.StringVar(value="--")
    tk.Label(u_row, textvariable=u_text, bg=CARD, fg=TEXT,
             font=("Consolas", 10, "bold"), anchor="w", padx=9, pady=4).pack(side="left", fill="x", expand=True)

    v_row = tk.Frame(coord_panel, bg=CARD)
    v_row.pack(fill="x")
    tk.Label(v_row, text="V", bg="#2b2447", fg=VIOLET,
             font=("Consolas", 9, "bold"), width=3, pady=4).pack(side="left")
    v_text = tk.StringVar(value="--")
    tk.Label(v_row, textvariable=v_text, bg=CARD, fg=TEXT,
             font=("Consolas", 10, "bold"), anchor="w", padx=9, pady=4).pack(side="left", fill="x", expand=True)

    def fit_window():
        root.update_idletasks()
        width = shell.winfo_reqwidth() + 4
        height = shell.winfo_reqheight() + 4
        root.geometry(f"{width}x{height}")

    fit_window()
    alive = True
    last_event = time.monotonic()
    stale = False

    def close():
        nonlocal alive
        alive = False
        root.destroy()

    def clear():
        view.clear_marker()
        u_text.set("--")
        v_text.set("--")

    root.protocol("WM_DELETE_WINDOW", close)

    with SensorSubscriber(topics=(TOPIC_VISION_GT,)) as subscriber:
        try:
            while alive:
                try:
                    root.update()
                except tk.TclError:
                    break
                if not alive:
                    break

                event = subscriber.recv(timeout=0.02)
                now = time.monotonic()
                if event is not None:
                    data = event.get("data", {})
                    last_event = now
                    stale = False
                    size_ok, resized = view.resize(data.get("width"), data.get("height"))
                    if not data.get("valid", False) or not size_ok:
                        clear()
                        continue
                    if resized:
                        fit_window()

                    target = data.get("targets", {}).get("base", {})
                    u, v = target.get("u"), target.get("v")
                    u_text.set(str(u) if u is not None else "--")
                    v_text.set(str(v) if v is not None else "--")
                    if target.get("in_frame") is False:
                        view.clear_marker()
                    else:
                        view.draw(u, v)

                if now - last_event > 1.5 and not stale:
                    clear()
                    stale = True
        except KeyboardInterrupt:
            pass
        finally:
            try:
                root.destroy()
            except tk.TclError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--console", action="store_true",
                        help="不打开窗口，每行打印一个JSON数据包")
    args = parser.parse_args()
    console() if args.console else show_panel()


if __name__ == "__main__":
    main()
