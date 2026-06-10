#!/usr/bin/env python3
"""Interactive map location registration tool (tkinter GUI).

Usage:
  ros2 run finav annotate_tool.py
  python3 scripts/map_annotate/annotate_tool.py
"""

import math
import os
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk
from typing import Dict, List, Optional, Tuple

from PIL import Image, ImageDraw, ImageFont, ImageTk

from map_utils import (
    discover_maps,
    load_locations,
    resolve_maps_dir,
    save_locations,
)

# Occupancy thresholds (matching map_utils.py)
OCCUPIED_PX = 60
FREE_PX = 245

COLOR_FREE = (0xF0, 0xF6, 0xF0)
COLOR_UNKNOWN = (0xB4, 0xB8, 0xB4)
COLOR_OCCUPIED = (0x34, 0x46, 0x3D)
COLOR_MARKER = (0xE0, 0x30, 0x30)
COLOR_SELECTED = (0xFF, 0xA0, 0x00)
COLOR_PLACING = (0x20, 0x80, 0xFF)
COLOR_ARROW = (0x20, 0x80, 0xFF)


def _pixel_to_rgb(pixel: int) -> Tuple[int, int, int]:
    if pixel <= OCCUPIED_PX:
        return COLOR_OCCUPIED
    if pixel >= FREE_PX:
        return COLOR_FREE
    return COLOR_UNKNOWN


def _try_load_font() -> ImageFont.FreeTypeFont:
    font_paths = [
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/usr/share/fonts/truetype/arphic/ukai.ttc",
    ]
    for fp in font_paths:
        if os.path.exists(fp):
            try:
                return ImageFont.truetype(fp, 14)
            except Exception:
                continue
    return ImageFont.load_default()


class _App:
    """Single-window app: map selector → editor."""

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("地点注册工具")
        self.root.geometry("500x400")
        self.root.minsize(400, 300)

        self.maps_dir = resolve_maps_dir()
        if not self.maps_dir or not os.path.isdir(self.maps_dir):
            messagebox.showerror("错误", "未找到 maps 目录")
            self.root.destroy()
            return

        self.maps = discover_maps(self.maps_dir)
        if not self.maps:
            messagebox.showerror("错误", "未发现任何地图")
            self.root.destroy()
            return

        self.label_font = _try_load_font()
        self._show_map_list()

    def _clear(self) -> None:
        for w in self.root.winfo_children():
            w.destroy()

    # ── Map List Screen ────────────────────────────────────────────

    def _show_map_list(self) -> None:
        self._clear()
        self.root.title("地点注册 — 选择地图")
        self.root.geometry("450x350")

        ttk.Label(self.root, text="请选择要编辑的地图:", padding=10,
                  font=("sans", 12, "bold")).pack()

        frame = ttk.Frame(self.root)
        frame.pack(padx=20, pady=5, fill=tk.BOTH, expand=True)

        scroll = ttk.Scrollbar(frame)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.map_listbox = tk.Listbox(
            frame, yscrollcommand=scroll.set, font=("monospace", 12),
            selectmode=tk.SINGLE, activestyle="none"
        )
        for m in self.maps:
            self.map_listbox.insert(tk.END, f"  {m}")
        self.map_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.config(command=self.map_listbox.yview)
        self.map_listbox.bind("<Double-1>", lambda e: self._open_map())
        if self.maps:
            self.map_listbox.selection_set(0)
        self.map_listbox.focus_set()

        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)
        ttk.Button(btn_frame, text="打开 (Enter)", command=self._open_map).pack(
            side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="退出 (Esc)", command=self.root.destroy).pack(
            side=tk.LEFT, padx=5)

        self.root.bind("<Return>", lambda e: self._open_map())
        self.root.bind("<Escape>", lambda e: self.root.destroy())

    def _open_map(self) -> None:
        sel = self.map_listbox.curselection()
        if not sel:
            return
        name = self.maps[sel[0]]
        self.root.unbind("<Return>")
        self.root.unbind("<Escape>")
        self._show_editor(name)

    # ── Map Editor Screen ──────────────────────────────────────────

    def _show_editor(self, map_name: str) -> None:
        self._clear()
        self.map_name = map_name
        self.root.title(f"编辑地点 — {map_name}")

        self.yaml_path = Path(self.maps_dir) / map_name / f"{map_name}.yaml"
        self.pgm_path = Path(self.maps_dir) / map_name / f"{map_name}.pgm"
        self.loc_path = Path(self.maps_dir) / map_name / f"{map_name}.locations.yaml"

        meta = {}
        for line in self.yaml_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
        self.resolution = float(meta.get("resolution", 0.05))
        origin_str = meta.get("origin", "[0.0, 0.0, 0.0]")
        parts = [float(x.strip()) for x in origin_str.strip("[]").split(",") if x.strip()]
        self.origin_x = parts[0] if len(parts) > 0 else 0.0
        self.origin_y = parts[1] if len(parts) > 1 else 0.0

        self.locations = load_locations(self.loc_path)

        self.pil_map = self._build_map_image()
        self.map_w = self.pil_map.width
        self.map_h = self.pil_map.height

        self.mode = "view"
        self.selected: Optional[str] = None
        self.placing_x = self.placing_y = self.placing_yaw = 0.0
        self.placing_px = self.placing_py = 0
        self.drag_px = self.drag_py = 0
        self.dragging = False
        self.zoom_level = 1.0

        self._build_editor_ui()
        self._zoom_fit()
        self._redraw()
        self._update_status("就绪 — 点击「添加地点」开始标注")

    def _map_to_pixel(self, x_map: float, y_map: float) -> Tuple[int, int]:
        px = int((x_map - self.origin_x) / self.resolution)
        py = self.map_h - 1 - int((y_map - self.origin_y) / self.resolution)
        return px, py

    def _pixel_to_map(self, px: int, py: int) -> Tuple[float, float]:
        x = self.origin_x + px * self.resolution
        y = self.origin_y + (self.map_h - 1 - py) * self.resolution
        return x, y

    def _build_map_image(self) -> Image.Image:
        pil = Image.open(self.pgm_path).convert("L")
        rgb = Image.new("RGB", pil.size)
        px_in = pil.load()
        px_out = rgb.load()
        for y in range(pil.height):
            for x in range(pil.width):
                px_out[x, y] = _pixel_to_rgb(px_in[x, y])
        return rgb

    def _build_editor_ui(self) -> None:
        bar = ttk.Frame(self.root)
        bar.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        self.btn_add = ttk.Button(bar, text="添加地点", command=self._on_add)
        self.btn_add.pack(side=tk.LEFT, padx=2)

        self.btn_del = ttk.Button(bar, text="删除选中", command=self._on_delete,
                                  state=tk.DISABLED)
        self.btn_del.pack(side=tk.LEFT, padx=2)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(bar, text="保存", command=self._on_save).pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text="返回列表", command=self._show_map_list).pack(side=tk.LEFT, padx=2)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)

        ttk.Button(bar, text="放大 +", command=lambda: self._zoom(1.25)).pack(side=tk.LEFT, padx=1)
        ttk.Button(bar, text="缩小 -", command=lambda: self._zoom(0.8)).pack(side=tk.LEFT, padx=1)
        ttk.Button(bar, text="适应窗口", command=self._zoom_fit).pack(side=tk.LEFT, padx=1)

        self.status = ttk.Label(bar, text="", relief=tk.SUNKEN, anchor=tk.W, padding=3)
        self.status.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(10, 0))

        cframe = ttk.Frame(self.root)
        cframe.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        cframe.rowconfigure(0, weight=1)
        cframe.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(cframe, bg="#888888", cursor="crosshair")
        self.hbar = ttk.Scrollbar(cframe, orient=tk.HORIZONTAL, command=self.canvas.xview)
        self.vbar = ttk.Scrollbar(cframe, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.config(xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set)

        self.vbar.grid(row=0, column=1, sticky="ns")
        self.hbar.grid(row=1, column=0, sticky="ew")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Motion>", self._on_move)
        self.canvas.bind("<MouseWheel>", self._on_wheel)
        self.canvas.bind("<Button-4>", self._on_wheel)
        self.canvas.bind("<Button-5>", self._on_wheel)

        self.root.bind("<Configure>", lambda e: self._on_resize())

    def _zoom(self, factor: float) -> None:
        self.zoom_level *= factor
        self.zoom_level = max(0.1, min(5.0, self.zoom_level))
        self._redraw()

    def _zoom_fit(self) -> None:
        self.root.update_idletasks()
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 50:
            w = 800
        if h < 50:
            h = 600
        self.zoom_level = min(w / self.map_w, h / self.map_h) * 0.92
        self._redraw()

    def _on_resize(self) -> None:
        pass

    def _update_status(self, text: str) -> None:
        self.status.config(text=text)

    # ── Toolbar actions ─────────────────────────────────────────────

    def _on_add(self) -> None:
        if self.mode == "place":
            self.mode = "view"
            self.btn_add.config(text="添加地点")
            self._update_status("已取消放置")
            self._redraw()
            return
        self.mode = "place"
        self.selected = None
        self.btn_add.config(text="取消")
        self.btn_del.config(state=tk.DISABLED)
        self._update_status("在地图上点击放置地点，拖拽设定朝向")
        self._redraw()

    def _on_delete(self) -> None:
        if not self.selected or self.selected not in self.locations:
            return
        name = self.selected
        if messagebox.askyesno("确认删除", f"确定要删除地点「{name}」吗？"):
            del self.locations[name]
            self.selected = None
            self.btn_del.config(state=tk.DISABLED)
            self._redraw()
            self._update_status(f"已删除「{name}」— 记得点击保存")

    def _on_save(self) -> None:
        save_locations(self.loc_path, self.locations)
        self._update_status(f"已保存 {len(self.locations)} 个地点")
        messagebox.showinfo("保存成功",
                            f"已保存 {len(self.locations)} 个地点到:\n{self.loc_path}")

    # ── Canvas events ───────────────────────────────────────────────

    def _on_click(self, event: tk.Event) -> None:
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)

        if self.mode == "place":
            px = int(cx / self.zoom_level)
            py = int(cy / self.zoom_level)
            if not (0 <= px < self.map_w and 0 <= py < self.map_h):
                return
            x_map, y_map = self._pixel_to_map(px, py)
            self.placing_x = x_map
            self.placing_y = y_map
            self.placing_px = px
            self.placing_py = py
            self.placing_yaw = 0.0
            self.drag_px = cx
            self.drag_py = cy
            self.dragging = True
            self._redraw()
        else:
            px = int(cx / self.zoom_level)
            py = int(cy / self.zoom_level)
            self._select_near(px, py)

    def _on_drag(self, event: tk.Event) -> None:
        if not self.dragging:
            return
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        self.drag_px = cx
        self.drag_py = cy
        dx = cx - self.placing_px * self.zoom_level
        dy = cy - self.placing_py * self.zoom_level
        if abs(dx) > 2 or abs(dy) > 2:
            self.placing_yaw = math.atan2(-dy, dx)
        self._redraw()

    def _on_release(self, event: tk.Event) -> None:
        if not self.dragging:
            return
        self.dragging = False
        if self.mode != "place":
            return

        yaw_deg = round(math.degrees(self.placing_yaw), 1)
        name = simpledialog.askstring(
            "地点名称",
            f"坐标: ({self.placing_x:.2f}, {self.placing_y:.2f})\n"
            f"朝向: {yaw_deg}°\n\n请输入地点名称:",
            parent=self.root
        )
        if not name or not name.strip():
            self._redraw()
            self._update_status("已取消 — 名称不能为空")
            return

        name = name.strip()
        if name in self.locations:
            if not messagebox.askyesno("覆盖确认",
                                       f"地点「{name}」已存在，是否覆盖？"):
                self._redraw()
                self._update_status("已取消覆盖")
                return

        self.locations[name] = {
            "x": round(self.placing_x, 3),
            "y": round(self.placing_y, 3),
            "yaw_deg": yaw_deg,
        }
        self.mode = "view"
        self.btn_add.config(text="添加地点")
        self._redraw()
        self._update_status(f"已添加「{name}」— 点击保存生效")

    def _on_move(self, event: tk.Event) -> None:
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        px = int(cx / self.zoom_level)
        py = int(cy / self.zoom_level)
        if 0 <= px < self.map_w and 0 <= py < self.map_h:
            x_m, y_m = self._pixel_to_map(px, py)
            extra = " [放置模式]" if self.mode == "place" else ""
            self._update_status(
                f"像素({px},{py}) 地图({x_m:.2f},{y_m:.2f}){extra}"
            )

    def _on_wheel(self, event: tk.Event) -> None:
        if event.num == 4:
            self._zoom(1.1)
        elif event.num == 5:
            self._zoom(1 / 1.1)
        elif hasattr(event, "delta"):
            if event.delta > 0:
                self._zoom(1.1)
            else:
                self._zoom(1 / 1.1)

    def _select_near(self, px: int, py: int) -> None:
        best_name = None
        best_dist = 100
        for name, loc in self.locations.items():
            lx, ly = self._map_to_pixel(loc["x"], loc["y"])
            d = (lx - px) ** 2 + (ly - py) ** 2
            if d < best_dist:
                best_dist = d
                best_name = name
        self.selected = best_name
        self.btn_del.config(state=tk.NORMAL if self.selected else tk.DISABLED)
        self._redraw()
        if self.selected:
            loc = self.locations[self.selected]
            self._update_status(
                f"已选中「{self.selected}」"
                f"({loc['x']:.2f}, {loc['y']:.2f}, {loc['yaw_deg']:.1f}°) — 可删除"
            )
        else:
            self._update_status("就绪")

    # ── Render ──────────────────────────────────────────────────────

    def _redraw(self) -> None:
        self.canvas.delete("all")
        self.root.update_idletasks()

        z = self.zoom_level
        zw = int(self.map_w * z)
        zh = int(self.map_h * z)

        self.canvas.config(scrollregion=(0, 0, zw, zh))

        draw_pil = self.pil_map.copy()
        pil_draw = ImageDraw.Draw(draw_pil)

        for name, loc in self.locations.items():
            lx, ly = self._map_to_pixel(loc["x"], loc["y"])
            color = COLOR_SELECTED if name == self.selected else COLOR_MARKER
            r = 6
            pil_draw.ellipse((lx - r, ly - r, lx + r, ly + r),
                             fill=color, outline=(255, 255, 255))
            yaw_rad = math.radians(loc["yaw_deg"])
            ax = lx + int(14 * math.cos(yaw_rad))
            ay = ly - int(14 * math.sin(yaw_rad))
            pil_draw.line((lx, ly, ax, ay), fill=color, width=2)
            pil_draw.text(
                (lx + 10, ly - 8), name, fill=color, font=self.label_font,
                stroke_width=1, stroke_fill=(255, 255, 255)
            )

        if self.mode == "place":
            px, py = self._map_to_pixel(self.placing_x, self.placing_y)
            r = 6
            pil_draw.ellipse((px - r, py - r, px + r, py + r),
                             fill=COLOR_PLACING, outline=(255, 255, 255))
            arrow_len = 18
            ax = px + int(arrow_len * math.cos(self.placing_yaw))
            ay = py - int(arrow_len * math.sin(self.placing_yaw))
            pil_draw.line((px, py, ax, ay), fill=COLOR_ARROW, width=3)

        py_img = draw_pil.resize((zw, zh), Image.NEAREST)
        self.tk_img = ImageTk.PhotoImage(py_img)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_img)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    maps_dir = resolve_maps_dir()
    if not maps_dir or not os.path.isdir(maps_dir):
        print("未找到 maps 目录")
        sys.exit(1)

    maps = discover_maps(maps_dir)
    if not maps:
        print("未发现任何地图")
        sys.exit(1)

    app = _App()
    app.run()


if __name__ == "__main__":
    main()
