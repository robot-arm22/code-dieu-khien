"""
slam_map_tab.py - Hien thi map SLAM trong phan mem dieu khien.
- Chon map.yaml cua ROS map_server / slam_toolbox
- Hien thi anh map .pgm/.png/.jpg
- Zoom / pan bang nut va chuot
- Ve robot, path, tram A/B/C theo toa do map (m)
- Co che do nghe ROS2 topic /marker_robot_pose va /robot_path neu may da cai ROS2
"""
import math
import time
import threading
from pathlib import Path
import tkinter as tk
from tkinter import filedialog

import customtkinter as ctk
from PIL import Image, ImageTk, ImageOps
import yaml

from constants import C_BG, C_PANEL, C_CARD, C_ACCENT, C_SUBTEXT, C_RED, C_GREEN, C_ORANGE


class SlamMapTab:
    def __init__(self, parent, shared):
        self.parent = parent
        self.shared = shared
        self.yaml_path = None
        self.map_image = None
        self.map_image_display = None
        self.tk_image = None
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.negate = 0
        self.occupied_thresh = 0.65
        self.free_thresh = 0.196
        self.zoom = 1.0
        self.offset_x = 20
        self.offset_y = 20
        self._drag_last = None
        self.calib_camera_points = []
        self.calib_map_points = []
        self.homography = None
        self.add_map_point_mode = False
        self.selected_calib_index = None

        # ================= CLICK GOAL MODE =================
        # map_goal = (x, y) theo hệ tọa độ map SLAM, đơn vị mét.
        self.map_goal = None
        self.goal_click_mode = False

        self.robot_pose = None
        self.path = []
        self.stations = {}
        self.ros_running = False
        self.ros_thread = None
        self.ros_node = None
        self.last_photo = None

        self._build_ui()
        self._redraw_loop()

    def _build_ui(self):
        self.root = ctk.CTkFrame(self.parent, fg_color=C_BG)
        self.root.pack(fill="both", expand=True, padx=6, pady=6)

        title = ctk.CTkLabel(
            self.root,
            text="BAN DO SLAM / RVIZ VIEW",
            font=("Consolas", 20, "bold"),
            text_color=C_ACCENT
        )
        title.pack(anchor="w", padx=8, pady=(4, 8))

        top = ctk.CTkFrame(self.root, fg_color=C_PANEL, corner_radius=10)
        top.pack(fill="x", padx=6, pady=(0, 8))

        ctk.CTkButton(
            top,
            text="CHON map.yaml",
            width=130,
            command=self.load_yaml_dialog
        ).pack(side="left", padx=8, pady=8)

        ctk.CTkButton(
            top,
            text="ZOOM +",
            width=80,
            command=lambda: self.set_zoom(self.zoom * 1.2)
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="ZOOM -",
            width=80,
            command=lambda: self.set_zoom(self.zoom / 1.2)
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="FIT",
            width=70,
            command=self.fit_map
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="CLEAR PATH",
            width=100,
            fg_color=C_ORANGE,
            command=self.clear_path
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="CLICK GOAL",
            width=105,
            fg_color=C_GREEN,
            command=self.enable_goal_click_mode
        ).pack(side="left", padx=(14, 4))

        ctk.CTkButton(
            top,
            text="GO TO GOAL",
            width=110,
            fg_color=C_ACCENT,
            command=self.go_to_clicked_goal
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="CLEAR GOAL",
            width=105,
            fg_color=C_ORANGE,
            command=self.clear_map_goal
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="LOAD H",
            width=80,
            command=self.load_homography_dialog
        ).pack(side="left", padx=(14, 4))

        ctk.CTkButton(
            top,
            text="SAVE H",
            width=80,
            command=self.save_homography_dialog
        ).pack(side="left", padx=4)

        ctk.CTkButton(
            top,
            text="START ROS LISTEN",
            width=140,
            fg_color=C_GREEN,
            command=self.start_ros_listener
        ).pack(side="left", padx=(18, 4))

        ctk.CTkButton(
            top,
            text="STOP ROS",
            width=90,
            fg_color=C_RED,
            command=self.stop_ros_listener
        ).pack(side="left", padx=4)

        self.status_label = ctk.CTkLabel(
            top,
            text="Chua tai map",
            font=("Consolas", 11, "bold"),
            text_color=C_SUBTEXT
        )
        self.status_label.pack(side="left", padx=12)

        body = ctk.CTkFrame(self.root, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=6, pady=4)
        body.grid_columnconfigure(0, weight=4)
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)

        map_card = ctk.CTkFrame(body, fg_color=C_PANEL, corner_radius=10)
        map_card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        map_card.grid_rowconfigure(0, weight=1)
        map_card.grid_columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(map_card, bg="#0b1120", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_down)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<Double-Button-1>", self._on_map_double_click)
        self.canvas.bind("<Button-3>", self._on_map_right_click)
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)
        self.canvas.bind("<Button-4>", lambda e: self.set_zoom(self.zoom * 1.1))
        self.canvas.bind("<Button-5>", lambda e: self.set_zoom(self.zoom / 1.1))

        side = ctk.CTkScrollableFrame(body, fg_color=C_PANEL, corner_radius=10)
        side.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        ctk.CTkLabel(
            side,
            text="TRAM TREN MAP",
            font=("Consolas", 15, "bold"),
            text_color=C_ACCENT
        ).pack(anchor="w", padx=12, pady=(12, 6))

        form = ctk.CTkFrame(side, fg_color=C_CARD, corner_radius=8)
        form.pack(fill="x", padx=10, pady=8)

        self.station_name = ctk.CTkEntry(form, width=80, placeholder_text="A")
        self.station_name.pack(side="left", padx=(10, 4), pady=10)

        self.station_x = ctk.CTkEntry(form, width=80, placeholder_text="x m")
        self.station_x.pack(side="left", padx=4, pady=10)

        self.station_y = ctk.CTkEntry(form, width=80, placeholder_text="y m")
        self.station_y.pack(side="left", padx=4, pady=10)

        ctk.CTkButton(
            form,
            text="ADD",
            width=60,
            command=self.add_station
        ).pack(side="left", padx=4, pady=10)

        ctk.CTkButton(
            side,
            text="LAY TOA DO ROBOT -> TRAM",
            command=self.add_station_from_robot
        ).pack(fill="x", padx=10, pady=4)

        ctk.CTkButton(
            side,
            text="XOA TRAM CUOI",
            fg_color=C_ORANGE,
            command=self.remove_last_station
        ).pack(fill="x", padx=10, pady=4)

        self.station_box = ctk.CTkTextbox(side, height=130, font=("Consolas", 10))
        self.station_box.pack(fill="x", padx=10, pady=(8, 12))

        ctk.CTkLabel(
            side,
            text="CALIB CAMERA ↔ SLAM MAP",
            font=("Consolas", 15, "bold"),
            text_color=C_ACCENT
        ).pack(anchor="w", padx=12, pady=(8, 6))

        calib = ctk.CTkFrame(side, fg_color=C_CARD, corner_radius=8)
        calib.pack(fill="x", padx=10, pady=8)

        ctk.CTkLabel(
            calib,
            text=(
                "1) Click tren camera live o tab CONTROL XE "
                "(se hien cham do P1,P2...)\n"
                "2) Double-click/chuot phai tren map dung diem tuong ung "
                "(se hien cham do)\n"
                "Can toi thieu 4 cap diem, thu tu CAM Pn phai khop MAP Pn."
            ),
            justify="left",
            font=("Consolas", 10),
            text_color=C_SUBTEXT
        ).pack(anchor="w", padx=10, pady=(10, 6))

        rowc = ctk.CTkFrame(calib, fg_color="transparent")
        rowc.pack(fill="x", padx=10, pady=3)

        self.cam_u = ctk.CTkEntry(rowc, width=70, placeholder_text="u px")
        self.cam_u.pack(side="left", padx=3)

        self.cam_v = ctk.CTkEntry(rowc, width=70, placeholder_text="v px")
        self.cam_v.pack(side="left", padx=3)

        ctk.CTkButton(
            rowc,
            text="ADD CAM",
            width=85,
            command=self.add_camera_point_manual
        ).pack(side="left", padx=3)

        rowm = ctk.CTkFrame(calib, fg_color="transparent")
        rowm.pack(fill="x", padx=10, pady=3)

        self.map_x = ctk.CTkEntry(rowm, width=70, placeholder_text="x m")
        self.map_x.pack(side="left", padx=3)

        self.map_y = ctk.CTkEntry(rowm, width=70, placeholder_text="y m")
        self.map_y.pack(side="left", padx=3)

        ctk.CTkButton(
            rowm,
            text="ADD MAP",
            width=85,
            command=self.add_map_point_manual
        ).pack(side="left", padx=3)

        rowd = ctk.CTkFrame(calib, fg_color="transparent")
        rowd.pack(fill="x", padx=10, pady=3)

        self.del_p = ctk.CTkEntry(rowd, width=70, placeholder_text="so P")
        self.del_p.pack(side="left", padx=3)

        ctk.CTkButton(
            rowd,
            text="XOA CAM P",
            width=90,
            fg_color=C_RED,
            command=self.delete_camera_point_by_entry
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            rowd,
            text="XOA MAP P",
            width=90,
            fg_color=C_RED,
            command=self.delete_map_point_by_entry
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            rowd,
            text="XOA CAP P",
            width=90,
            fg_color=C_ORANGE,
            command=self.delete_pair_by_entry
        ).pack(side="left", padx=3)

        ctk.CTkButton(
            calib,
            text="TINH HOMOGRAPHY",
            fg_color=C_GREEN,
            command=self.compute_homography
        ).pack(fill="x", padx=10, pady=(8, 3))

        ctk.CTkButton(
            calib,
            text="XOA DIEM CALIB",
            fg_color=C_ORANGE,
            command=self.clear_calibration_points
        ).pack(fill="x", padx=10, pady=3)

        self.calib_box = ctk.CTkTextbox(calib, height=120, font=("Consolas", 9))
        self.calib_box.pack(fill="x", padx=10, pady=(6, 10))
        self._refresh_calib_box()

        ctk.CTkLabel(
            side,
            text="ROBOT / ROS2",
            font=("Consolas", 15, "bold"),
            text_color=C_ACCENT
        ).pack(anchor="w", padx=12, pady=(8, 6))

        pose_frame = ctk.CTkFrame(side, fg_color=C_CARD, corner_radius=8)
        pose_frame.pack(fill="x", padx=10, pady=8)

        self.pose_x = ctk.CTkEntry(pose_frame, width=80, placeholder_text="x")
        self.pose_x.grid(row=0, column=0, padx=6, pady=8)

        self.pose_y = ctk.CTkEntry(pose_frame, width=80, placeholder_text="y")
        self.pose_y.grid(row=0, column=1, padx=6, pady=8)

        self.pose_yaw = ctk.CTkEntry(pose_frame, width=80, placeholder_text="yaw deg")
        self.pose_yaw.grid(row=0, column=2, padx=6, pady=8)

        ctk.CTkButton(
            pose_frame,
            text="SET ROBOT",
            command=self.set_robot_manual
        ).grid(row=1, column=0, columnspan=3, sticky="ew", padx=6, pady=(0, 8))

        self.info_box = ctk.CTkTextbox(side, height=200, font=("Consolas", 10))
        self.info_box.pack(fill="both", expand=True, padx=10, pady=(8, 12))

        self._log("Chon file map.yaml de hien thi ban do SLAM.")

        self.shared["slam_map_tab"] = self
        self.shared["add_camera_calib_point"] = self.add_camera_point_from_click
        self.shared["update_slam_from_tracking"] = self.update_from_tracking

        self._refresh_station_box()

    # ---------------- Calibration Camera <-> SLAM Map ----------------
    def canvas_to_map(self, cx, cy):
        if not self.map_image:
            return None

        ox, oy = self.origin[0], self.origin[1]

        px = (float(cx) - self.offset_x) / max(self.zoom, 1e-9)
        py_img = (float(cy) - self.offset_y) / max(self.zoom, 1e-9)

        x = ox + px * self.resolution
        y = oy + (self.map_image.height - py_img) * self.resolution

        return float(x), float(y)

    # ---------------- Click map goal -> car_control_tab.py ----------------
    def enable_goal_click_mode(self):
        if not self.map_image:
            self._log("Chua tai map.yaml nen chua dat duoc goal.")
            self.status_label.configure(text="Chua tai map", text_color=C_ORANGE)
            return

        self.goal_click_mode = True
        self.status_label.configure(text="CLICK GOAL MODE: click tren map", text_color=C_GREEN)
        self._log("CLICK GOAL MODE: click trai 1 diem tren ban do de dat goal.")

    def clear_map_goal(self):
        self.map_goal = None
        self.goal_click_mode = False
        self.shared.pop("map_goal", None)
        self.shared["map_goal_active"] = False

        self.redraw()
        self._log("Da xoa MAP GOAL.")
        self.status_label.configure(text="Da xoa MAP GOAL", text_color=C_ORANGE)

    def go_to_clicked_goal(self):
        if self.map_goal is None:
            self._log("Chua co MAP GOAL. Bam CLICK GOAL roi click tren ban do truoc.")
            self.status_label.configure(text="Chua co MAP GOAL", text_color=C_ORANGE)
            return

        self.shared["map_goal"] = self.map_goal
        self.shared["map_goal_active"] = True

        cb = self.shared.get("car_go_to_map_goal")

        if cb:
            cb()
            self._log(f"GO TO GOAL x={self.map_goal[0]:.3f}, y={self.map_goal[1]:.3f}")
            self.status_label.configure(
                text=f"GOAL {self.map_goal[0]:.2f},{self.map_goal[1]:.2f}",
                text_color=C_GREEN
            )
        else:
            self._log("Khong thay car_go_to_map_goal. Hay dam bao tab CONTROL XE da duoc khoi tao.")
            self.status_label.configure(text="CONTROL XE chua san sang", text_color=C_RED)

    def set_goal_from_canvas_click(self, cx, cy):
        pt = self.canvas_to_map(cx, cy)

        if pt is None:
            self._log("Chua tai map nen khong dat duoc goal.")
            return False

        self.map_goal = (float(pt[0]), float(pt[1]))
        self.shared["map_goal"] = self.map_goal
        self.shared["map_goal_active"] = False
        self.goal_click_mode = False

        self._log(
            f"Da dat MAP GOAL: x={pt[0]:.3f}, y={pt[1]:.3f}. "
            f"Bam GO TO GOAL de xe chay."
        )
        self.status_label.configure(
            text=f"Goal: {pt[0]:.2f},{pt[1]:.2f}",
            text_color=C_GREEN
        )
        self.redraw()
        return True

    def _on_map_double_click(self, event):
        pt = self.canvas_to_map(event.x, event.y)

        if pt is None:
            self._log("Chua tai map nen khong lay duoc diem map.")
            return

        self.calib_map_points.append(pt)
        self._refresh_calib_box()
        self._log(
            f"ADD MAP point P{len(self.calib_map_points)}: "
            f"x={pt[0]:.3f}, y={pt[1]:.3f}"
        )

    def add_camera_point_from_click(self, u, v):
        self.calib_camera_points.append((float(u), float(v)))
        self._refresh_calib_box()
        self._log(f"ADD CAMERA point: u={float(u):.1f}, v={float(v):.1f}")

    def add_camera_point_manual(self):
        try:
            u = float(self.cam_u.get().strip())
            v = float(self.cam_v.get().strip())

            self.add_camera_point_from_click(u, v)

            self.cam_u.delete(0, "end")
            self.cam_v.delete(0, "end")
        except Exception as e:
            self._log(f"LOI ADD CAM point: {e}")

    def add_map_point_manual(self):
        try:
            x = float(self.map_x.get().strip())
            y = float(self.map_y.get().strip())

            self.calib_map_points.append((x, y))
            self._refresh_calib_box()

            self.map_x.delete(0, "end")
            self.map_y.delete(0, "end")

            self._log(f"ADD MAP point P{len(self.calib_map_points)}: x={x:.3f}, y={y:.3f}")
        except Exception as e:
            self._log(f"LOI ADD MAP point: {e}")

    def _nearest_map_point_index(self, cx, cy, max_dist_canvas=18):
        if not self.calib_map_points:
            return None

        best_i, best_d = None, None

        for i, (x, y) in enumerate(self.calib_map_points):
            px, py = self.map_to_canvas(x, y)
            d = math.hypot(px - cx, py - cy)

            if best_d is None or d < best_d:
                best_i, best_d = i, d

        if best_d is not None and best_d <= max_dist_canvas:
            return best_i

        return None

    def _on_map_right_click(self, event):
        idx = self._nearest_map_point_index(event.x, event.y, max_dist_canvas=24)

        if idx is None:
            self._log("Chuot phai: khong co MAP point nao gan vi tri chuot de xoa.")
            return

        self.delete_map_point(idx)

    def delete_camera_point(self, idx):
        if idx is None or idx < 0 or idx >= len(self.calib_camera_points):
            return False

        old = self.calib_camera_points.pop(idx)
        self.shared["camera_calib_points"] = list(self.calib_camera_points)

        self._refresh_calib_box()
        self._log(f"Da xoa CAM P{idx + 1}: {old}")

        return True

    def delete_map_point(self, idx):
        if idx is None or idx < 0 or idx >= len(self.calib_map_points):
            return False

        old = self.calib_map_points.pop(idx)

        self._refresh_calib_box()
        self._log(f"Da xoa MAP P{idx + 1}: {old}")

        return True

    def delete_pair_point(self, idx):
        ok = False

        if idx is not None and 0 <= idx < len(self.calib_map_points):
            self.calib_map_points.pop(idx)
            ok = True

        if idx is not None and 0 <= idx < len(self.calib_camera_points):
            self.calib_camera_points.pop(idx)
            ok = True

        if ok:
            self.shared["camera_calib_points"] = list(self.calib_camera_points)
            self._refresh_calib_box()
            self._log(f"Da xoa CAP P{idx + 1} tren CAM/MAP neu ton tai.")

        return ok

    def _entry_index(self):
        txt = self.del_p.get().strip().upper().replace("P", "")

        if not txt:
            self._log("Nhap so P can xoa, vi du 3 hoac P3.")
            return None

        try:
            idx = int(txt) - 1

            if idx < 0:
                raise ValueError

            return idx
        except Exception:
            self._log("So P khong hop le.")
            return None

    def delete_camera_point_by_entry(self):
        idx = self._entry_index()

        if idx is not None and not self.delete_camera_point(idx):
            self._log(f"Khong co CAM P{idx + 1}.")

    def delete_map_point_by_entry(self):
        idx = self._entry_index()

        if idx is not None and not self.delete_map_point(idx):
            self._log(f"Khong co MAP P{idx + 1}.")

    def delete_pair_by_entry(self):
        idx = self._entry_index()

        if idx is not None and not self.delete_pair_point(idx):
            self._log(f"Khong co CAP P{idx + 1} de xoa.")

    def delete_nearest_camera_point(self, u, v, max_dist_px=35):
        if not self.calib_camera_points:
            return None

        best_i, best_d = None, None

        for i, p in enumerate(self.calib_camera_points):
            d = math.hypot(float(p[0]) - float(u), float(p[1]) - float(v))

            if best_d is None or d < best_d:
                best_i, best_d = i, d

        if best_d is not None and best_d <= max_dist_px:
            self.delete_camera_point(best_i)
            return best_i

        return None

    def clear_calibration_points(self):
        self.calib_camera_points.clear()
        self.calib_map_points.clear()
        self._refresh_calib_box()
        self._log("Da xoa cac diem calib.")

    def _refresh_calib_box(self):
        if not hasattr(self, "calib_box"):
            return

        self.calib_box.delete("1.0", "end")

        n = max(len(self.calib_camera_points), len(self.calib_map_points))
        self.calib_box.insert(
            "end",
            f"CAM points: {len(self.calib_camera_points)} | "
            f"MAP points: {len(self.calib_map_points)}\n"
        )

        for i in range(n):
            cp = self.calib_camera_points[i] if i < len(self.calib_camera_points) else None
            mp = self.calib_map_points[i] if i < len(self.calib_map_points) else None
            self.calib_box.insert("end", f"{i + 1:02d}: CAM={cp}  MAP={mp}\n")

        self.calib_box.insert(
            "end",
            "\nChon xoa: nhap 3/P3 hoac chuot phai gan cham tren camera/map.\n"
        )
        self.calib_box.insert(
            "end",
            "Trang thai H: " + ("DA CO" if self.homography is not None else "CHUA CO") + "\n"
        )

    def compute_homography(self):
        try:
            if len(self.calib_camera_points) < 4 or len(self.calib_map_points) < 4:
                self._log("Can toi thieu 4 diem camera va 4 diem map.")
                return

            n = min(len(self.calib_camera_points), len(self.calib_map_points))

            import numpy as np
            from vision.cv2_safe import get_cv2

            cv2 = get_cv2()

            src = np.array(self.calib_camera_points[:n], dtype=np.float32)
            dst = np.array(self.calib_map_points[:n], dtype=np.float32)

            H, mask = cv2.findHomography(src, dst, 0)

            if H is None:
                self._log("Khong tinh duoc Homography.")
                return

            self.homography = H
            self.shared["camera_to_map_homography"] = H

            self._refresh_calib_box()
            self._log("Da tinh Homography camera pixel -> map meter.")
        except Exception as e:
            self._log(f"LOI tinh Homography: {e}")

    def pixel_to_map(self, u, v):
        if self.homography is None:
            return None

        try:
            import numpy as np
            from vision.cv2_safe import get_cv2

            cv2 = get_cv2()

            arr = np.array([[[float(u), float(v)]]], dtype=np.float32)
            out = cv2.perspectiveTransform(arr, self.homography.astype(np.float32))

            return float(out[0, 0, 0]), float(out[0, 0, 1])
        except Exception:
            return None

    def save_homography_dialog(self):
        path = filedialog.asksaveasfilename(
            title="Luu homography.yaml",
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml *.yml"), ("All files", "*.*")]
        )

        if path:
            self.save_homography(path)

    def save_homography(self, path):
        if self.homography is None:
            self._log("Chua co Homography de luu.")
            return

        try:
            data = {
                "homography": self.homography.tolist(),
                "camera_points": self.calib_camera_points,
                "map_points": self.calib_map_points,
                "note": "pixel camera -> toa do map SLAM met"
            }

            with open(path, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, allow_unicode=True)

            self._log(f"Da luu Homography: {path}")
        except Exception as e:
            self._log(f"LOI luu Homography: {e}")

    def load_homography_dialog(self):
        path = filedialog.askopenfilename(
            title="Chon homography.yaml",
            filetypes=[("YAML", "*.yaml *.yml"), ("All files", "*.*")]
        )

        if path:
            self.load_homography(path)

    def load_homography(self, path):
        try:
            import numpy as np

            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            H = np.array(data.get("homography"), dtype=float)

            if H.shape != (3, 3):
                raise ValueError("homography khong phai ma tran 3x3")

            self.homography = H
            self.shared["camera_to_map_homography"] = H

            self.calib_camera_points = [tuple(p) for p in data.get("camera_points", [])]
            self.calib_map_points = [tuple(p) for p in data.get("map_points", [])]

            self.shared["camera_calib_points"] = list(self.calib_camera_points)

            self._refresh_calib_box()
            self._log(f"Da nap Homography: {path}")
        except Exception as e:
            self._log(f"LOI nap Homography: {e}")

    def update_from_tracking(self, robot_pose=None, stations=None):
        if robot_pose is not None:
            self.robot_pose = robot_pose
            x, y, _ = robot_pose

            self.path.append((x, y))

            if len(self.path) > 3000:
                self.path = self.path[-3000:]

        if stations:
            for name, xy in stations.items():
                self.stations[str(name).upper()] = (float(xy[0]), float(xy[1]))

            self._refresh_station_box()

    def _log(self, msg):
        self.info_box.insert("end", f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.info_box.see("end")

    def load_yaml_dialog(self):
        path = filedialog.askopenfilename(
            title="Chon map.yaml",
            filetypes=[("YAML map", "*.yaml *.yml"), ("All files", "*.*")]
        )

        if path:
            self.load_map_yaml(path)

    def load_map_yaml(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            img_ref = data.get("image")

            if not img_ref:
                raise ValueError("map.yaml khong co truong image")

            img_path = Path(img_ref)

            if not img_path.is_absolute():
                img_path = Path(path).parent / img_ref

            img = Image.open(img_path).convert("L")
            img = ImageOps.autocontrast(img)
            img = img.convert("RGB")

            self.yaml_path = path
            self.map_image = img
            self.resolution = float(data.get("resolution", 0.05))
            self.origin = list(data.get("origin", [0.0, 0.0, 0.0]))
            self.negate = int(data.get("negate", 0))
            self.occupied_thresh = float(data.get("occupied_thresh", 0.65))
            self.free_thresh = float(data.get("free_thresh", 0.196))

            self.status_label.configure(
                text=f"Map: {img.width}x{img.height}, res={self.resolution}",
                text_color=C_GREEN
            )

            self._log(f"Da tai map: {img_path.name}")
            self._log(f"origin={self.origin}, resolution={self.resolution}")

            self.fit_map()
        except Exception as e:
            self.status_label.configure(text="Loi tai map", text_color=C_RED)
            self._log(f"LOI map.yaml: {e}")

    def set_zoom(self, z):
        self.zoom = max(0.05, min(20.0, float(z)))
        self.redraw()

    def fit_map(self):
        if not self.map_image:
            return

        cw = max(100, self.canvas.winfo_width())
        ch = max(100, self.canvas.winfo_height())

        self.zoom = min(
            (cw - 30) / self.map_image.width,
            (ch - 30) / self.map_image.height
        )
        self.zoom = max(0.05, self.zoom)

        self.offset_x = 15
        self.offset_y = 15

        self.redraw()

    def _on_mouse_down(self, event):
        # Nếu đang ở chế độ CLICK GOAL thì click trái để đặt goal, không kéo map.
        if getattr(self, "goal_click_mode", False):
            self.set_goal_from_canvas_click(event.x, event.y)
            self._drag_last = None
            return

        self._drag_last = (event.x, event.y)

    def _on_mouse_drag(self, event):
        if self._drag_last:
            lx, ly = self._drag_last

            self.offset_x += event.x - lx
            self.offset_y += event.y - ly

            self._drag_last = (event.x, event.y)

            self.redraw()

    def _on_mouse_wheel(self, event):
        self.set_zoom(self.zoom * (1.1 if event.delta > 0 else 1 / 1.1))

    def map_to_canvas(self, x, y):
        if not self.map_image:
            return 0, 0

        ox, oy = self.origin[0], self.origin[1]

        px = (float(x) - ox) / self.resolution
        py = self.map_image.height - ((float(y) - oy) / self.resolution)

        return self.offset_x + px * self.zoom, self.offset_y + py * self.zoom

    def redraw(self):
        self.canvas.delete("all")

        if not self.map_image:
            self.canvas.create_text(
                30,
                30,
                anchor="nw",
                text="Chua tai map.yaml",
                fill="#94a3b8",
                font=("Consolas", 16)
            )
            return

        w = max(1, int(self.map_image.width * self.zoom))
        h = max(1, int(self.map_image.height * self.zoom))

        self.map_image_display = self.map_image.resize((w, h), Image.Resampling.BILINEAR)
        self.tk_image = ImageTk.PhotoImage(self.map_image_display)

        self.canvas.create_image(self.offset_x, self.offset_y, anchor="nw", image=self.tk_image)
        self.canvas.create_rectangle(
            self.offset_x,
            self.offset_y,
            self.offset_x + w,
            self.offset_y + h,
            outline="#38bdf8"
        )

        # clicked goal
        if self.map_goal is not None:
            gx, gy = self.map_goal
            gcx, gcy = self.map_to_canvas(gx, gy)
            r = 12

            self.canvas.create_oval(
                gcx - r,
                gcy - r,
                gcx + r,
                gcy + r,
                outline="#ef4444",
                width=4
            )
            self.canvas.create_oval(
                gcx - 4,
                gcy - 4,
                gcx + 4,
                gcy + 4,
                fill="#ef4444",
                outline="#ffffff",
                width=1
            )
            self.canvas.create_text(
                gcx + 16,
                gcy - 14,
                text=f"GOAL\n{gx:.2f},{gy:.2f}",
                anchor="w",
                fill="#fecaca",
                font=("Consolas", 11, "bold")
            )

        # path
        if len(self.path) >= 2:
            pts = []

            for x, y in self.path[-2000:]:
                cx, cy = self.map_to_canvas(x, y)
                pts.extend([cx, cy])

            if len(pts) >= 4:
                self.canvas.create_line(*pts, fill="#22c55e", width=2)

        # calibration map points
        for i, (x, y) in enumerate(self.calib_map_points, start=1):
            cx, cy = self.map_to_canvas(x, y)
            r = 7

            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                fill="#ef4444",
                outline="#ffffff",
                width=2
            )
            self.canvas.create_text(
                cx,
                cy,
                text=str(i),
                fill="#ffffff",
                font=("Consolas", 9, "bold")
            )
            self.canvas.create_text(
                cx + 11,
                cy + 8,
                text=f"P{i} ({x:.2f},{y:.2f})",
                anchor="nw",
                fill="#fecaca",
                font=("Consolas", 9, "bold")
            )

        # stations
        for name, (x, y) in sorted(self.stations.items()):
            cx, cy = self.map_to_canvas(x, y)
            r = 8

            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                fill="#f59e0b",
                outline="#ffffff",
                width=2
            )
            self.canvas.create_text(
                cx + 12,
                cy - 12,
                text=f"TRAM {name}\n{x:.2f},{y:.2f}",
                anchor="w",
                fill="#fde68a",
                font=("Consolas", 10, "bold")
            )

        # robot
        if self.robot_pose:
            x, y, yaw = self.robot_pose
            cx, cy = self.map_to_canvas(x, y)
            r = 10

            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                fill="#00d4ff",
                outline="#ffffff",
                width=2
            )

            ex = cx + math.cos(yaw) * 28
            ey = cy - math.sin(yaw) * 28

            self.canvas.create_line(
                cx,
                cy,
                ex,
                ey,
                fill="#ffffff",
                width=3,
                arrow=tk.LAST
            )

            self.canvas.create_text(
                cx + 14,
                cy + 12,
                text=f"ROBOT\nx={x:.2f} y={y:.2f}\nyaw={math.degrees(yaw):.1f}",
                anchor="nw",
                fill="#bae6fd",
                font=("Consolas", 10, "bold")
            )

        self.canvas.create_text(
            10,
            10,
            anchor="nw",
            text=f"zoom={self.zoom:.2f} | keo chuot de pan | cuon chuot de zoom",
            fill="#e2e8f0",
            font=("Consolas", 10)
        )

    def _redraw_loop(self):
        try:
            self.redraw()
        except Exception:
            pass

        self.parent.after(500, self._redraw_loop)

    def add_station(self):
        try:
            name = (self.station_name.get().strip() or chr(ord("A") + len(self.stations))).upper()
            x = float(self.station_x.get().strip())
            y = float(self.station_y.get().strip())

            self.stations[name] = (x, y)
            self._refresh_station_box()
            self._log(f"Them tram {name}: x={x:.2f}, y={y:.2f}")
        except Exception as e:
            self._log(f"LOI them tram: {e}")

    def add_station_from_robot(self):
        if not self.robot_pose:
            self._log("Chua co toa do robot de them tram.")
            return

        used = set(self.stations.keys())
        name = "A"

        for i in range(26):
            cand = chr(ord("A") + i)

            if cand not in used:
                name = cand
                break

        x, y, _ = self.robot_pose
        self.stations[name] = (x, y)

        self._refresh_station_box()
        self._log(f"Da lay toa do robot lam tram {name}: x={x:.2f}, y={y:.2f}")

    def remove_last_station(self):
        if not self.stations:
            return

        key = sorted(self.stations.keys())[-1]
        self.stations.pop(key, None)

        self._refresh_station_box()
        self._log(f"Da xoa tram {key}")

    def _refresh_station_box(self):
        self.station_box.delete("1.0", "end")

        if not self.stations:
            self.station_box.insert("end", "Chua co tram tren map\n")
        else:
            for k, (x, y) in sorted(self.stations.items()):
                self.station_box.insert("end", f"TRAM {k}: x={x:.3f}, y={y:.3f}\n")

    def set_robot_manual(self):
        try:
            x = float(self.pose_x.get().strip())
            y = float(self.pose_y.get().strip())
            yaw = math.radians(float(self.pose_yaw.get().strip() or "0"))

            self.robot_pose = (x, y, yaw)
            self.path.append((x, y))

            self._log(f"Set robot: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}")
        except Exception as e:
            self._log(f"LOI set robot: {e}")

    def clear_path(self):
        self.path.clear()
        self._log("Da xoa path tren map.")

    # ---------------- ROS2 listener optional ----------------
    def start_ros_listener(self):
        if self.ros_running:
            return

        self.ros_running = True
        self.ros_thread = threading.Thread(target=self._ros_worker, daemon=True)
        self.ros_thread.start()

        self.status_label.configure(text="ROS listen dang chay", text_color=C_GREEN)
        self._log("Bat dau nghe ROS2: /marker_robot_pose, /robot_path")

    def _ros_worker(self):
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path as RosPath

            if not rclpy.ok():
                rclpy.init(args=None)

            class MapUiNode(Node):
                pass

            node = MapUiNode("robot_ui_slam_map_viewer")
            self.ros_node = node

            def pose_cb(msg):
                q = msg.pose.orientation

                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                x = float(msg.pose.position.x)
                y = float(msg.pose.position.y)

                self.robot_pose = (x, y, yaw)
                self.path.append((x, y))

                if len(self.path) > 3000:
                    self.path = self.path[-3000:]

            def path_cb(msg):
                pts = []

                for p in msg.poses[-3000:]:
                    pts.append((float(p.pose.position.x), float(p.pose.position.y)))

                if pts:
                    self.path = pts

            node.create_subscription(PoseStamped, "/marker_robot_pose", pose_cb, 10)
            node.create_subscription(RosPath, "/robot_path", path_cb, 10)

            while self.ros_running:
                rclpy.spin_once(node, timeout_sec=0.1)

            node.destroy_node()
        except Exception as e:
            self.ros_running = False

            try:
                self.status_label.configure(text="ROS listen loi", text_color=C_RED)
                self._log(f"LOI ROS2 listener: {e}")
            except Exception:
                pass

    def stop_ros_listener(self):
        self.ros_running = False
        self.status_label.configure(text="ROS listen OFF", text_color=C_ORANGE)
        self._log("Da dung ROS listener.")

    def close(self):
        self.stop_ros_listener()
