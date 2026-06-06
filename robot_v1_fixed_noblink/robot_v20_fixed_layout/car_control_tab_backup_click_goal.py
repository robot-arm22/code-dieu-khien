"""
car_control_tab.py - Tab dieu khien xe tich hop trong KHO HANG.
Tinh nang:
- UDP ESP32 truc tiep trong giao dien
- Camera tracking live nhung vao app
- Chon camera index rieng cho tab CONTROL XE, co the chay song song voi camera KHO HANG
- Them/sua/xoa tram A,B,C... voi ArUco ID dong
- Dieu huong ArUco co PID, gui PWM qua UDP
- ROS/RViz: van giu che do chay main_map.py rieng neu can map day du
"""
import math
import time
import os
import sys
import signal
import subprocess
from pathlib import Path
import threading
import queue

import customtkinter as ctk
import tkinter as tk
from PIL import Image

from constants import C_BG, C_PANEL, C_CARD, C_ACCENT, C_TEXT, C_SUBTEXT, C_RED, C_GREEN, C_ORANGE
from vision.cv2_safe import get_cv2
from vision.camera_manager import CameraManager

DEFAULT_UDP_IP = "192.168.1.250"
DEFAULT_UDP_PORT = "8080"

FRONT_ID = 1
REAR_ID = 0

IDLE = 0
NAVIGATING = 1
ARRIVE = 2


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


class Kalman2D:
    def __init__(self, q=0.02, r=3):
        import numpy as np
        self.x = np.zeros(2)
        self.P = np.eye(2)
        self.Q = q * np.eye(2)
        self.R = r * np.eye(2)
        self.initialized = False

    def update(self, z):
        import numpy as np
        z = np.array(z, dtype=float)
        if not self.initialized:
            self.x = z.copy()
            self.initialized = True
            return self.x
        self.P = self.P + self.Q
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.x = self.x + K @ (z - self.x)
        self.P = (np.eye(2) - K) @ self.P
        return self.x


class CarControlTab:
    def __init__(self, parent, shared):
        self.parent = parent
        self.shared = shared
        self.cv2 = None
        self.np = None
        self.camera = CameraManager()
        self.cap = None
        self.running = False
        self.proc = None
        self.last_reply = "NO REPLY"
        self.last_photo = None
        self.last_frame_size = None
        self.last_display_size = None
        self._camera_after_id = None
        self._camera_generation = 0
        self._last_camera_idx = None
        self._closing = False
        self._camera_busy = False
        self._camera_busy_since = 0.0
        self.camera_connected = False
        self._pending_start_after_id = None
        self._open_retry_count = 0
        self._frame_queue = queue.Queue(maxsize=1)
        self._worker_thread = None
        self._ui_after_id = None

        self.station_map = {"A": 2, "B": 3}
        self.current_order = None
        self.target_id = None
        self.state = IDLE

        self.kf_front = Kalman2D()
        self.kf_rear = Kalman2D()
        self.kf_target = Kalman2D()
        self.prev_err = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.last_stop_time = 0.0
        self.arrive_counter = 0

        self.Kp = 0.75
        self.Ki = 0.0
        self.Kd = 0.10
        self.FORWARD_SPEED = 150
        self.TURN_MIN_PWM = 150
        self.TURN_MAX_PWM = 150
        self.MIN_FORWARD_PWM = 55
        self.MAX_CORRECTION = 25
        self.TURN_THRESHOLD = 8
        self.ANGLE_DEADBAND = 3
        self.ARRIVE_DISTANCE = 45
        self.SLOW_DISTANCE = 180
        self.ARRIVE_HOLD_FRAMES = 5

        self._build_ui()
        self._tick_udp()

    # ---------------- UI ----------------
    def _build_ui(self):
        self.root = ctk.CTkFrame(self.parent, fg_color=C_BG)
        self.root.pack(fill="both", expand=True, padx=6, pady=6)

        title = ctk.CTkLabel(self.root, text="DIEU KHIEN XE / CAMERA TRACKING", font=("Consolas", 20, "bold"), text_color=C_ACCENT)
        title.pack(anchor="w", padx=8, pady=(4, 8))

        top = ctk.CTkFrame(self.root, fg_color=C_PANEL, corner_radius=10)
        top.pack(fill="x", padx=6, pady=(0, 8))

        ctk.CTkLabel(top, text="ESP32 DÙNG CHUNG", font=("Consolas", 12, "bold"), text_color=C_TEXT).grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.conn_label = ctk.CTkLabel(top, text="Chưa kết nối - vào AUTO > CONTROL để CONNECT ESP32 UDP", font=("Consolas", 11, "bold"), text_color=C_ORANGE)
        self.conn_label.grid(row=0, column=1, columnspan=3, padx=4, pady=10, sticky="w")
        ctk.CTkButton(top, text="PING", width=70, command=lambda: self.send_raw("PING")).grid(row=0, column=4, padx=4)
        ctk.CTkButton(top, text="STOP", width=80, fg_color=C_RED, command=lambda: self.stop_robot()).grid(row=0, column=5, padx=4)

        ctk.CTkLabel(top, text="CAM CONTROL INDEX", font=("Consolas", 12, "bold"), text_color=C_TEXT).grid(row=0, column=6, padx=(22, 6), pady=10)
        self.cam_index = ctk.CTkComboBox(top, values=[str(i) for i in range(8)], width=70, font=("Consolas", 11))
        self.cam_index.set("1")
        self.cam_index.grid(row=0, column=7, padx=4)
        self.connect_cam_btn = ctk.CTkButton(top, text="CONNECT CAM", width=120, fg_color=C_GREEN, command=self.start_camera)
        self.connect_cam_btn.grid(row=0, column=8, padx=4)
        self.disconnect_cam_btn = ctk.CTkButton(top, text="DISCONNECT", width=110, fg_color=C_ORANGE, command=self.stop_camera)
        self.disconnect_cam_btn.grid(row=0, column=9, padx=4)

        self.status_label = ctk.CTkLabel(top, text="SAN SANG", font=("Consolas", 11, "bold"), text_color=C_GREEN)
        self.status_label.grid(row=0, column=10, padx=12, sticky="w")
        top.grid_columnconfigure(11, weight=1)

        body = ctk.CTkFrame(self.root, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=6, pady=4)
        body.grid_columnconfigure(0, weight=3)
        body.grid_columnconfigure(1, weight=2)
        body.grid_rowconfigure(0, weight=1)

        cam_card = ctk.CTkFrame(body, fg_color=C_PANEL, corner_radius=10)
        self.cam_card = cam_card
        cam_card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        cam_card.grid_rowconfigure(1, weight=1)
        cam_card.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(cam_card, text="LIVE CAMERA TRACKING - CONTROL XE", font=("Consolas", 15, "bold"), text_color=C_ACCENT).grid(row=0, column=0, sticky="w", padx=12, pady=10)
        self.video_label = None
        self._make_video_label("Chua ket noi camera")

        side = ctk.CTkScrollableFrame(body, fg_color=C_PANEL, corner_radius=10)
        side.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        ctk.CTkLabel(side, text="DIEU HUONG", font=("Consolas", 15, "bold"), text_color=C_ACCENT).pack(anchor="w", padx=12, pady=(12, 6))
        self.station_buttons = ctk.CTkFrame(side, fg_color="transparent")
        self.station_buttons.pack(fill="x", padx=10, pady=4)
        self._refresh_station_buttons()

        manual = ctk.CTkFrame(side, fg_color=C_CARD, corner_radius=8)
        manual.pack(fill="x", padx=10, pady=8)
        ctk.CTkLabel(manual, text="PWM THU CONG", font=("Consolas", 12, "bold"), text_color=C_TEXT).pack(anchor="w", padx=10, pady=(10, 4))
        row = ctk.CTkFrame(manual, fg_color="transparent")
        row.pack(fill="x", padx=10, pady=4)
        ctk.CTkLabel(row, text="L").pack(side="left")
        self.left_pwm = ctk.CTkEntry(row, width=70); self.left_pwm.insert(0, "0"); self.left_pwm.pack(side="left", padx=4)
        ctk.CTkLabel(row, text="R").pack(side="left", padx=(10,0))
        self.right_pwm = ctk.CTkEntry(row, width=70); self.right_pwm.insert(0, "0"); self.right_pwm.pack(side="left", padx=4)
        ctk.CTkButton(manual, text="GUI PWM", command=self.send_manual_pwm).pack(fill="x", padx=10, pady=(4, 10))
        ctk.CTkButton(manual, text="XOAY TRAI TEST", command=lambda: self.send_pwm(-105,105)).pack(fill="x", padx=10, pady=3)
        ctk.CTkButton(manual, text="XOAY PHAI TEST", command=lambda: self.send_pwm(105,-105)).pack(fill="x", padx=10, pady=(3,10))

        st = ctk.CTkFrame(side, fg_color=C_CARD, corner_radius=8)
        st.pack(fill="x", padx=10, pady=8)
        ctk.CTkLabel(st, text="THEM TRAM A,B,C...", font=("Consolas", 12, "bold"), text_color=C_TEXT).pack(anchor="w", padx=10, pady=(10, 4))
        form = ctk.CTkFrame(st, fg_color="transparent")
        form.pack(fill="x", padx=10, pady=4)
        self.station_name_entry = ctk.CTkEntry(form, width=70, placeholder_text="Ten")
        self.station_name_entry.pack(side="left", padx=(0, 4))
        self.station_id_entry = ctk.CTkEntry(form, width=80, placeholder_text="ID")
        self.station_id_entry.pack(side="left", padx=4)
        ctk.CTkButton(form, text="ADD", width=70, command=self.add_station).pack(side="left", padx=4)
        self.station_list_label = ctk.CTkLabel(st, text="", font=("Consolas", 11), text_color=C_SUBTEXT, justify="left")
        self.station_list_label.pack(anchor="w", padx=10, pady=(4, 10))
        self._refresh_station_label()

        ctk.CTkButton(side, text="START main_map.py ROS/RViz", command=self.start_main_map).pack(fill="x", padx=10, pady=(12, 4))
        ctk.CTkButton(side, text="STOP main_map.py", fg_color=C_RED, command=self.stop_main_map).pack(fill="x", padx=10, pady=4)
        self.proc_label = ctk.CTkLabel(side, text="main_map.py: OFF", font=("Consolas", 11), text_color=C_ORANGE)
        self.proc_label.pack(anchor="w", padx=12, pady=(4, 8))

        self.info_box = ctk.CTkTextbox(side, height=170, font=("Consolas", 10))
        self.info_box.pack(fill="both", expand=True, padx=10, pady=(6, 12))
        self._log("Da san sang. ESP32 dùng chung từ tab AUTO > CONTROL. Chọn camera index rồi bấm CONNECT CAM.")
        self._set_camera_buttons(False)

    def _make_video_label(self, text="Chua ket noi camera"):
        """Tao lai widget hien thi camera.
        V20: moi lan Stop Close / Open se rebuild CTkLabel de xoa hoan toan
        image command cu cua Tkinter. Cach nay xu ly truong hop camera da mo lai
        nhung khung live bi trong cho den khi thoat app.
        """
        try:
            if getattr(self, "video_label", None) is not None and self.video_label.winfo_exists():
                self.video_label.destroy()
        except Exception:
            pass
        self.last_photo = None
        self.video_label = ctk.CTkLabel(self.cam_card, text=text, font=("Consolas", 15), text_color=C_SUBTEXT)
        self.video_label.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 12))
        self.video_label.bind("<Double-Button-1>", self._on_video_double_click)
        self.video_label.bind("<Button-3>", self._on_video_right_click)
        try:
            self.video_label.update_idletasks()
        except Exception:
            pass

    def _button(self, parent, text, cmd, r, c):
        b = ctk.CTkButton(parent, text=text, command=cmd, height=36, font=("Consolas", 11, "bold"))
        b.grid(row=r, column=c, padx=4, pady=4, sticky="ew")
        return b

    def _refresh_station_buttons(self):
        for w in self.station_buttons.winfo_children():
            w.destroy()
        cols = 2
        for i in range(cols): self.station_buttons.grid_columnconfigure(i, weight=1)
        items = sorted(self.station_map.items())
        for idx, (name, mid) in enumerate(items):
            self._button(self.station_buttons, f"DI TRAM {name} (ID {mid})", lambda n=name: self.select_target(n), idx//cols, idx%cols)
        row = (len(items)+1)//cols
        self._button(self.station_buttons, "DUNG / IDLE", self.set_idle, row, 0)
        self._button(self.station_buttons, "XOA TRAM CUOI", self.remove_last_station, row, 1)

    def _refresh_station_label(self):
        text = "\n".join([f"Tram {k}: ArUco ID {v}" for k, v in sorted(self.station_map.items())])
        self.station_list_label.configure(text=text or "Chua co tram")

    # ---------------- ESP32 shared connection ----------------
    def _log(self, txt):
        ts = time.strftime("%H:%M:%S")
        self.info_box.insert("end", f"[{ts}] {txt}\n")
        self.info_box.see("end")

    def send_raw(self, msg):
        servo = self.shared.get("servo")
        if not servo or not servo.is_connected():
            self.status_label.configure(text="CHƯA KẾT NỐI ESP32", text_color=C_RED)
            self._log("Chưa có kết nối ESP32 dùng chung. Vào AUTO > CONTROL để CONNECT ESP32 UDP.")
            return
        try:
            servo.send_raw(str(msg))
            self.status_label.configure(text=f"ESP32 -> {msg}", text_color=C_GREEN)
            self._log(f"ESP32 -> {msg}")
        except Exception as e:
            self.status_label.configure(text="LỖI GỬI ESP32", text_color=C_RED)
            self._log(f"Lỗi gửi ESP32: {e}")

    def send_pwm(self, left, right):
        try:
            left = int(_clip(int(float(left)), -255, 255)); right = int(_clip(int(float(right)), -255, 255))
        except Exception:
            left = right = 0
        self.send_raw(f"{left},{right}")

    def send_manual_pwm(self):
        self.send_pwm(self.left_pwm.get(), self.right_pwm.get())

    def stop_robot(self):
        self.send_pwm(0, 0)

    def _read_udp_reply(self):
        servo = self.shared.get("servo")
        if not servo:
            return
        reply = servo.read_reply_nonblocking()
        while reply:
            self.last_reply = reply
            self._log(f"ESP32 <- {self.last_reply}")
            reply = servo.read_reply_nonblocking()

    def _tick_udp(self):
        servo = self.shared.get("servo")
        if servo and servo.is_connected():
            self.conn_label.configure(text=servo.connection_label(), text_color=C_GREEN)
        else:
            self.conn_label.configure(text="Chưa kết nối - vào AUTO > CONTROL để CONNECT ESP32 UDP", text_color=C_ORANGE)
        self._read_udp_reply()
        if self.proc and self.proc.poll() is not None:
            self.proc_label.configure(text=f"main_map.py: EXIT {self.proc.returncode}", text_color=C_RED)
        self.parent.after(300, self._tick_udp)

    # ---------------- Stations ----------------
    def add_station(self):
        name = (self.station_name_entry.get().strip() or "").upper()
        sid = self.station_id_entry.get().strip()
        if not name:
            # auto next A/B/C...
            used = set(self.station_map.keys())
            for ch in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                if ch not in used:
                    name = ch; break
        try:
            mid = int(sid)
        except Exception:
            used_ids = set(self.station_map.values()) | {FRONT_ID, REAR_ID}
            mid = 0
            while mid in used_ids: mid += 1
        self.station_map[name] = mid
        self.station_name_entry.delete(0, "end")
        self.station_id_entry.delete(0, "end")
        self._refresh_station_buttons(); self._refresh_station_label()
        self._log(f"Da them/cap nhat tram {name} = marker ID {mid}")
        self.shared["car_station_map"] = dict(self.station_map)

    def remove_last_station(self):
        if not self.station_map: return
        key = sorted(self.station_map.keys())[-1]
        self.station_map.pop(key, None)
        if self.current_order == key: self.set_idle()
        self._refresh_station_buttons(); self._refresh_station_label()
        self._log(f"Da xoa tram {key}")
        self.shared["car_station_map"] = dict(self.station_map)

    def select_target(self, name):
        if name not in self.station_map:
            return
        self.target_id = self.station_map[name]
        self.current_order = name
        self.state = NAVIGATING
        self.arrive_counter = 0
        self.reset_pid(); self.reset_filters()
        self._log(f"GO TO {name} | TARGET ID = {self.target_id}")

    def set_idle(self):
        self.state = IDLE; self.current_order = None; self.target_id = None; self.arrive_counter = 0
        self.reset_pid(); self.stop_robot(); self._log("STOP / IDLE")

    def _video_widget_alive(self):
        try:
            return self.video_label is not None and bool(self.video_label.winfo_exists())
        except Exception:
            return False

    def _safe_video_configure(self, **kwargs):
        """Configure video label safely.
        V13: chong loi Tkinter `image "pyimageXX" doesn't exist` khi callback camera cu
        con chay sau disconnect/reconnect hoac khi widget da bi destroy.
        """
        if not self._video_widget_alive():
            return False
        try:
            self.video_label.configure(**kwargs)
            return True
        except tk.TclError as e:
            msg = str(e)
            if "pyimage" in msg or "doesn't exist" in msg or "invalid command name" in msg:
                return False
            raise
        except Exception:
            return False

    def _set_camera_buttons(self, connected, busy=False):
        """Cap nhat nut camera theo trang thai thuc.
        V18: tach ro 3 trang thai OFF / BUSY / CONNECTED de tranh truong hop
        bam DISCONNECT xong UI van tuong nhu dang bat va khong CONNECT lai duoc.
        """
        self.camera_connected = bool(connected)
        try:
            if getattr(self, "connect_cam_btn", None) is not None:
                self.connect_cam_btn.configure(state=("disabled" if (connected or busy) else "normal"))
            if getattr(self, "disconnect_cam_btn", None) is not None:
                self.disconnect_cam_btn.configure(state=("normal" if (connected and not busy) else "disabled"))
        except Exception:
            pass

    def _set_busy(self, busy=True):
        self._camera_busy = bool(busy)
        self._camera_busy_since = time.time() if busy else 0.0

    def _busy_is_stale(self):
        return bool(self._camera_busy and self._camera_busy_since and (time.time() - self._camera_busy_since > 3.0))

    # ---------------- Camera tracking ----------------
    def start_camera(self):
        """Mo camera tracking theo co che giong camera KHO HANG: Open -> worker thread -> UI after.
        FIX Bug 2: camera.open() va warmup chay trong background thread, khong block UI.
        """
        if self.running:
            self._log("Camera dang chay. Neu muon doi index hay bam DISCONNECT truoc.")
            return
        if self._camera_busy and not self._busy_is_stale():
            self._log("Camera dang xu ly connect/disconnect, vui long doi...")
            return

        try:
            idx = int(str(self.cam_index.get()).strip() or "0")
        except Exception:
            self.status_label.configure(text="CAM INDEX KHONG HOP LE", text_color=C_RED)
            self._log("Camera index khong hop le")
            return

        # Kiểm tra conflict trước khi sang thread
        wh_idx = self.shared.get("warehouse_camera_index")
        wh_tab = self.shared.get("camera_tab")
        if wh_idx is not None and wh_tab is not None and getattr(wh_tab, "running", False) and int(wh_idx) == idx:
            self.status_label.configure(text=f"CAM {idx} DANG DUOC KHO HANG DUNG", text_color=C_RED)
            self._log(f"Khong mo camera {idx}: camera nay dang duoc tab KHO HANG su dung. Hay chon index khac.")
            return

        self._set_busy(True)
        self._set_camera_buttons(False, busy=True)
        self.status_label.configure(text=f"DANG MO CAMERA {idx}...", text_color=C_ORANGE)
        self._log(f"Dang mo camera control index {idx}...")
        self._make_video_label(f"Dang mo camera {idx}...")
        try:
            self.parent.update_idletasks()
        except Exception:
            pass

        def _open_in_bg():
            """Chay trong background thread: open + warmup. Sau do bao UI qua after()."""
            try:
                # V20: reset sach, doi the CameraManager va tao lai label truoc khi open.
                self._release_camera(clear_label=False, update_buttons=False, rebuild_label=False)
                try:
                    self.camera = CameraManager()
                except Exception:
                    pass

                cv2_local = get_cv2()
                import numpy as np

                if not self.camera.open(idx):
                    def _fail_open():
                        self.status_label.configure(text="KHONG MO DUOC CAMERA", text_color=C_RED)
                        self._log(f"Khong mo duoc camera index {idx}. Kiem tra /dev/video{idx} hoac tab khac co dang dung.")
                        self._set_busy(False)
                        self._set_camera_buttons(False)
                    try:
                        self.parent.after(0, _fail_open)
                    except Exception:
                        pass
                    return

                # Warmup: doc thu vai frame (chay trong thread, khong block UI)
                test_frame = None
                for _ in range(10):
                    test_frame = self.camera.read()
                    if test_frame is not None:
                        break
                    time.sleep(0.05)

                if test_frame is None:
                    self.camera.close()
                    def _fail_frame():
                        self.status_label.configure(text="CAMERA KHONG CO FRAME", text_color=C_RED)
                        self._log(f"Camera {idx} mo duoc nhung khong doc duoc frame.")
                        self._set_busy(False)
                        self._set_camera_buttons(False)
                    try:
                        self.parent.after(0, _fail_frame)
                    except Exception:
                        pass
                    return

                # Thành công – cập nhật state và khởi động worker từ UI thread
                def _start_success():
                    try:
                        self.cv2 = cv2_local
                        self.np = np
                        self.cap = self.camera.cap
                        self.aruco_dict = cv2_local.aruco.getPredefinedDictionary(cv2_local.aruco.DICT_4X4_50)
                        self.aruco_params = cv2_local.aruco.DetectorParameters()
                        self.detector = cv2_local.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                        self.reset_pid()
                        self.reset_filters()
                        self.running = True
                        self._closing = False
                        self._camera_generation += 1
                        gen = self._camera_generation
                        self._last_camera_idx = idx
                        self.shared["car_camera_index"] = idx

                        try:
                            while True:
                                self._frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            self._frame_queue.put_nowait(test_frame)
                        except Exception:
                            pass

                        self._worker_thread = threading.Thread(target=self._frame_worker, args=(gen,), daemon=True)
                        self._worker_thread.start()
                        self._schedule_ui_update(gen)

                        self.status_label.configure(text=f"CAM CONTROL {idx} CONNECTED", text_color=C_GREEN)
                        self._set_camera_buttons(True)
                        self._log(f"Da ket noi camera control index {idx}")
                    except Exception as e:
                        self.running = False
                        self._release_camera(clear_label=True, update_buttons=False, rebuild_label=True)
                        self.status_label.configure(text="LOI CAMERA", text_color=C_RED)
                        self._log(f"LOI camera: {e}")
                    finally:
                        self._set_busy(False)
                        if not self.running:
                            self._set_camera_buttons(False)

                try:
                    self.parent.after(0, _start_success)
                except Exception:
                    pass

            except Exception as e:
                def _fail_exc(err=e):
                    self._release_camera(clear_label=True, update_buttons=False, rebuild_label=True)
                    self.status_label.configure(text="LOI CAMERA", text_color=C_RED)
                    self._log(f"LOI camera: {err}")
                    self._set_busy(False)
                    self._set_camera_buttons(False)
                try:
                    self.parent.after(0, _fail_exc)
                except Exception:
                    pass

        threading.Thread(target=_open_in_bg, daemon=True).start()

    def _frame_worker(self, generation):
        """Doc frame rieng nhu camera KHO HANG. UI chi lay frame moi nhat tu queue."""
        while self.running and generation == self._camera_generation and self.cap is not None:
            try:
                frame = self.camera.read()
            except Exception:
                frame = None
            if frame is None:
                time.sleep(0.03)
                continue
            try:
                if self._frame_queue.full():
                    try:
                        self._frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                self._frame_queue.put_nowait(frame)
            except Exception:
                pass
            time.sleep(0.005)

    def _schedule_ui_update(self, generation=None):
        if generation is None:
            generation = self._camera_generation
        if not self.running or generation != self._camera_generation:
            return
        try:
            self._ui_after_id = self.parent.after(25, lambda g=generation: self._do_ui_update(g))
        except Exception:
            pass

    def _do_ui_update(self, generation):
        self._ui_after_id = None
        if not self.running or generation != self._camera_generation:
            return
        frame = None
        try:
            while True:
                frame = self._frame_queue.get_nowait()
        except queue.Empty:
            pass
        if frame is not None:
            try:
                frame = self._process_frame(frame)
                self._show_frame(frame)
            except Exception as e:
                self._log(f"LOI tracking: {e}")
        self._schedule_ui_update(generation)

    def _release_camera(self, clear_label=True, update_buttons=True, rebuild_label=False):
        """Dong camera tracking theo co che Stop Close that sach.
        V20: cancel after, doi generation, close camera, join worker ngan, clear queue
        va co tuy chon rebuild label de xoa loi khung live trong sau reconnect.
        """
        old_thread = getattr(self, "_worker_thread", None)
        self.running = False
        self._closing = True
        self._camera_generation += 1

        # Huy cac after dang cho.
        for attr in ("_pending_start_after_id", "_camera_after_id", "_ui_after_id"):
            aid = getattr(self, attr, None)
            setattr(self, attr, None)
            if aid is not None:
                for owner in (getattr(self, "parent", None), getattr(self, "video_label", None)):
                    try:
                        if owner is not None and owner.winfo_exists():
                            owner.after_cancel(aid)
                            break
                    except Exception:
                        pass

        self.cap = None
        try:
            self.camera.close()
        except Exception:
            pass
        try:
            if old_thread is not None and old_thread is not threading.current_thread() and old_thread.is_alive():
                old_thread.join(timeout=0.6)
        except Exception:
            pass
        self._worker_thread = None
        try:
            # Tao manager moi sau close de tranh giu handle backend V4L2 cu.
            self.camera = CameraManager()
        except Exception:
            pass
        try:
            time.sleep(0.12)
        except Exception:
            pass

        try:
            while True:
                self._frame_queue.get_nowait()
        except queue.Empty:
            pass

        self.shared.pop("car_camera_index", None)
        self.last_photo = None
        self.last_frame_size = None
        self.last_display_size = None
        if clear_label or rebuild_label:
            try:
                if rebuild_label:
                    self._make_video_label("CAMERA CLOSED")
                else:
                    self.video_label.configure(image=None, text="CAMERA CLOSED", text_color=C_SUBTEXT)
                    self.video_label.image = None
            except Exception:
                pass
        try:
            import gc
            gc.collect()
        except Exception:
            pass
        if update_buttons:
            self._set_camera_buttons(False)

    def stop_camera(self):
        """Stop Close camera tracking - giong nut STOP CLOSE cua camera kho hang."""
        if not self.running and self.cap is None:
            self._release_camera(clear_label=True, update_buttons=True, rebuild_label=True)
            self.status_label.configure(text="CAMERA OFF - CO THE CONNECT LAI", text_color=C_ORANGE)
            return
        self._set_busy(True)
        self._set_camera_buttons(False, busy=True)
        try:
            self.stop_robot()
            self._release_camera(clear_label=True, update_buttons=False, rebuild_label=True)
            self._log("Da stop close camera tracking va giai phong thiet bi.")
            self.status_label.configure(text="CAMERA OFF - CO THE CONNECT LAI", text_color=C_ORANGE)
        except Exception as e:
            self._log(f"Loi khi stop close camera: {e}")
        finally:
            self._set_busy(False)
            self._set_camera_buttons(False)
            self._closing = False

    def _label_click_to_frame_pixel(self, event):
        """Doi toa do chuot tren label -> pixel anh camera goc.
        Ham nay tinh dung phan anh dang duoc fit/center trong khung label,
        nen cham calib se nam trung voi vi tri chuot.
        """
        if not self.last_frame_size or not self.last_display_size:
            return None
        fw, fh = self.last_frame_size
        dw, dh = self.last_display_size
        lw = max(1, self.video_label.winfo_width())
        lh = max(1, self.video_label.winfo_height())
        off_x = max(0.0, (lw - dw) / 2.0)
        off_y = max(0.0, (lh - dh) / 2.0)
        x = float(event.x) - off_x
        y = float(event.y) - off_y
        if x < 0 or y < 0 or x > dw or y > dh:
            return None
        u = x * float(fw) / max(float(dw), 1.0)
        v = y * float(fh) / max(float(dh), 1.0)
        return float(u), float(v)

    def _on_video_double_click(self, event):
        """Lay diem pixel camera goc khi DOUBLE CLICK tren khung live; gui sang tab BAN DO SLAM de calib."""
        pt = self._label_click_to_frame_pixel(event)
        if pt is None:
            self._log("Double-click nam ngoai anh camera hoac chua co frame camera.")
            return
        u, v = pt
        cb = self.shared.get("add_camera_calib_point")
        if cb:
            cb(u, v)
            self.shared["camera_calib_points"] = list(getattr(self.shared.get("slam_map_tab"), "calib_camera_points", []) or [])
            self._log(f"Da them CAM point P{len(self.shared['camera_calib_points'])}: u={u:.1f}, v={v:.1f}")
        else:
            pts = list(self.shared.get("camera_calib_points", []) or [])
            pts.append((float(u), float(v)))
            self.shared["camera_calib_points"] = pts
            self._log(f"CAM point u={u:.1f}, v={v:.1f} (tab BAN DO SLAM chua san sang)")

    def _on_video_right_click(self, event):
        """Chuot phai tren camera: xoa diem CAM gan vi tri chuot nhat."""
        pt = self._label_click_to_frame_pixel(event)
        if pt is None:
            return
        u, v = pt
        slam = self.shared.get("slam_map_tab")
        if slam is not None and hasattr(slam, "delete_nearest_camera_point"):
            idx = slam.delete_nearest_camera_point(u, v, max_dist_px=35)
            if idx is not None:
                self._log(f"Da xoa CAM point P{idx+1}")
            else:
                self._log("Khong co CAM point nao gan vi tri chuot de xoa.")
        else:
            pts = list(self.shared.get("camera_calib_points", []) or [])
            if not pts:
                return
            import math
            dists = [math.hypot(float(p[0])-u, float(p[1])-v) for p in pts]
            idx = min(range(len(dists)), key=lambda i: dists[i])
            if dists[idx] <= 35:
                pts.pop(idx); self.shared["camera_calib_points"] = pts

    def _publish_tracking_to_slam(self, data, front_px=None, rear_px=None):
        cb = self.shared.get("update_slam_from_tracking")
        if not cb:
            return
        slam = self.shared.get("slam_map_tab")
        if not slam or getattr(slam, "homography", None) is None:
            return
        robot_pose = None
        stations = {}
        try:
            if front_px is not None and rear_px is not None:
                fm = slam.pixel_to_map(float(front_px[0]), float(front_px[1]))
                rm = slam.pixel_to_map(float(rear_px[0]), float(rear_px[1]))
                if fm and rm:
                    x = (fm[0] + rm[0]) / 2.0
                    y = (fm[1] + rm[1]) / 2.0
                    yaw = math.atan2(fm[1] - rm[1], fm[0] - rm[0])
                    robot_pose = (x, y, yaw)
            # cap nhat cac tram neu camera nhin thay marker tram
            for name, mid in self.station_map.items():
                if mid in data:
                    c = self.center(data[mid])
                    xy = slam.pixel_to_map(float(c[0]), float(c[1]))
                    if xy:
                        stations[name] = xy
            if robot_pose is not None or stations:
                cb(robot_pose=robot_pose, stations=stations)
        except Exception as e:
            # khong spam log lien tuc
            pass

    def reset_filters(self):
        self.kf_front = Kalman2D(); self.kf_rear = Kalman2D(); self.kf_target = Kalman2D()

    def reset_pid(self):
        self.prev_err = 0.0; self.integral = 0.0; self.last_time = time.time()

    def center(self, corner):
        c = corner.reshape((4,2))
        return self.np.array([(c[0][0]+c[2][0])/2, (c[0][1]+c[2][1])/2])

    def heading(self, front, rear):
        v = front - rear
        return math.degrees(math.atan2(v[0], v[1]))

    def angle(self, v):
        return math.degrees(math.atan2(v[0], v[1]))

    def heading_error(self, robot_angle, target_angle):
        return (target_angle - robot_angle + 180) % 360 - 180

    def send_stop_periodic(self, now):
        if now - self.last_stop_time > 0.25:
            self.stop_robot(); self.last_stop_time = now

    def draw_text(self, frame, text, pos, color=(0,255,255), scale=0.55, thick=2):
        self.cv2.putText(frame, text, pos, self.cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

    def _camera_loop(self, generation=None):
        # V18: camera loop da duoc thay bang _frame_worker + _do_ui_update,
        # giu ham nay de tuong thich neu co callback cu con goi toi.
        return

    def _process_frame(self, frame):
        cv2 = self.cv2; np = self.np
        now = time.time()
        dt = _clip(now - self.last_time, 0.001, 0.1); self.last_time = now
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        state_name = {IDLE:"IDLE", NAVIGATING:"NAVIGATING", ARRIVE:"ARRIVE"}.get(self.state,"?")

        if ids is None:
            self.send_stop_periodic(now)
            self.draw_text(frame, "NO MARKER", (20,40), (0,0,255), 0.8)
            self.draw_text(frame, f"STATE {state_name} | ESP32 {self.last_reply}", (20,75), (255,255,0), 0.55)
            return frame

        ids = ids.flatten()
        data = {int(i): c for c, i in zip(corners, ids)}
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if FRONT_ID not in data or REAR_ID not in data:
            self.send_stop_periodic(now)
            self.draw_text(frame, f"ROBOT MARKER LOST: need {FRONT_ID},{REAR_ID}", (20,40), (0,0,255), 0.75)
            self.draw_text(frame, f"Seen IDs: {list(data.keys())}", (20,75), (255,255,255), 0.55)
            return frame

        front = self.kf_front.update(self.center(data[FRONT_ID]))
        rear = self.kf_rear.update(self.center(data[REAR_ID]))
        robot_c = (front + rear) / 2
        robot_h = self.heading(front, rear)
        self._publish_tracking_to_slam(data, front, rear)
        left = right = 0

        if self.state == IDLE or self.target_id is None:
            self.send_stop_periodic(now)
            self.draw_text(frame, "STATE: IDLE", (20,40), (0,255,255), 0.8)
            self.draw_text(frame, "Bam nut DI TRAM A/B/C... tren giao dien", (20,75), (255,255,255), 0.55)
        elif self.target_id not in data:
            self.send_stop_periodic(now)
            self.draw_text(frame, f"TARGET {self.current_order} ID {self.target_id} NOT FOUND", (20,40), (0,0,255), 0.75)
        else:
            target = self.kf_target.update(self.center(data[self.target_id]))
            target_vec = target - robot_c
            target_ang = self.angle(target_vec)
            err = self.heading_error(robot_h, target_ang)
            d = float(np.linalg.norm(robot_c - target))
            if d < self.ARRIVE_DISTANCE: self.arrive_counter += 1
            else: self.arrive_counter = 0
            if self.arrive_counter >= self.ARRIVE_HOLD_FRAMES:
                self.state = ARRIVE; self.stop_robot()

            if self.state == ARRIVE:
                self.send_stop_periodic(now)
                self.draw_text(frame, f"ARRIVED STATION {self.current_order}", (20,40), (0,255,0), 0.9, 3)
            else:
                err_control = 0.0 if abs(err) < self.ANGLE_DEADBAND else err
                self.integral = _clip(self.integral + err_control * dt, -80, 80)
                derivative = _clip((err_control - self.prev_err) / dt, -200, 200)
                self.prev_err = err_control
                pid = self.Kp * err_control + self.Ki * self.integral + self.Kd * derivative
                if abs(err) > self.TURN_THRESHOLD:
                    turn_pwm = _clip(np.interp(abs(err), [self.TURN_THRESHOLD,45], [self.TURN_MIN_PWM,self.TURN_MAX_PWM]), self.TURN_MIN_PWM, self.TURN_MAX_PWM)
                    if err > 0: left, right = -turn_pwm, turn_pwm
                    else: left, right = turn_pwm, -turn_pwm
                else:
                    if d < self.SLOW_DISTANCE:
                        forward = np.interp(d, [self.ARRIVE_DISTANCE,self.SLOW_DISTANCE], [self.MIN_FORWARD_PWM,self.FORWARD_SPEED])
                    else:
                        forward = self.FORWARD_SPEED
                    correction = _clip(pid * 0.6, -self.MAX_CORRECTION, self.MAX_CORRECTION)
                    left, right = forward - correction, forward + correction
                left = int(_clip(left, -255, 255)); right = int(_clip(right, -255, 255))
                self.send_pwm(left, right)

            ri = tuple(robot_c.astype(int)); ti = tuple(target.astype(int))
            robot_end = robot_c + (front - rear) * 0.8
            cv2.arrowedLine(frame, ri, tuple(robot_end.astype(int)), (0,255,0), 3, tipLength=0.3)
            cv2.arrowedLine(frame, ri, ti, (255,0,0), 2, tipLength=0.2)
            cv2.circle(frame, ri, 6, (0,255,0), -1); cv2.circle(frame, ti, 6, (0,0,255), -1)
            panel = [f"STATE: {state_name}", f"ORDER: {self.current_order}", f"TARGET ID: {self.target_id}", f"HEADING: {robot_h:.1f}", f"ERROR: {err:.1f}", f"DISTANCE: {d:.1f}", f"PWM L/R: {left},{right}", f"ESP32: {self.last_reply}"]
            y = 110
            for t in panel:
                self.draw_text(frame, t, (10,y), (0,255,255), 0.52, 2); y += 24

        # draw station IDs seen
        y = frame.shape[0] - 20
        self.draw_text(frame, "Stations: " + ", ".join([f"{k}=ID{v}" for k,v in sorted(self.station_map.items())]), (10,y), (255,255,0), 0.5)
        return frame

    def _draw_calib_camera_points(self, frame):
        """Ve cac diem calib camera truc tiep len khung live tracking.
        Diem duoc lay tu tab BAN DO SLAM de dam bao P1/P2... khop voi map.
        """
        try:
            slam = self.shared.get("slam_map_tab")
            pts = []
            if slam is not None and hasattr(slam, "calib_camera_points"):
                pts = list(getattr(slam, "calib_camera_points", []) or [])
            else:
                pts = list(self.shared.get("camera_calib_points", []) or [])
            if not pts:
                return
            h, w = frame.shape[:2]
            for i, pt in enumerate(pts, start=1):
                try:
                    u, v = float(pt[0]), float(pt[1])
                except Exception:
                    continue
                if u < 0 or v < 0 or u >= w or v >= h:
                    continue
                x, y = int(round(u)), int(round(v))
                # Cham do + vien trang + nhan Pn de de doi chieu voi map
                self.cv2.circle(frame, (x, y), 9, (255, 255, 255), 2)
                self.cv2.circle(frame, (x, y), 7, (0, 0, 255), -1)
                self.cv2.putText(frame, f"P{i}", (x + 12, y - 10), self.cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 3)
                self.cv2.putText(frame, f"P{i}", (x + 12, y - 10), self.cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 255), 2)
        except Exception:
            pass

    def _show_frame(self, frame):
        """Hien thi frame len CTkLabel.
        V18: dung CTkImage thay cho ImageTk.PhotoImage de tranh loi reconnect da mo camera
        nhung khung live bi trong/khong ve anh sau DISCONNECT -> CONNECT.
        """
        if (not self.running) or self.cap is None or (not self._video_widget_alive()):
            return
        cv2 = self.cv2
        self._draw_calib_camera_points(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w = frame.shape[:2]
        self.last_frame_size = (w, h)
        label_w = max(320, int(self.video_label.winfo_width() or 820))
        label_h = max(240, int(self.video_label.winfo_height() or 560))
        scale = min(label_w / max(w, 1), label_h / max(h, 1))
        nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
        self.last_display_size = (nw, nh)
        try:
            img = Image.fromarray(frame).resize((nw, nh))
            photo = ctk.CTkImage(light_image=img, dark_image=img, size=(nw, nh))
            # Bat buoc giu reference song tren self va label, neu khong reconnect nhieu lan
            # co the mat image command va label chi con nen trong.
            self.last_photo = photo
            try:
                self.video_label.image = photo
            except Exception:
                pass
            ok = self._safe_video_configure(image=photo, text="")
            if not ok:
                self.last_photo = None
                try:
                    self.video_label.image = None
                except Exception:
                    pass
                return
            try:
                self.video_label.update_idletasks()
            except Exception:
                pass
        except tk.TclError as e:
            if "pyimage" in str(e) or "doesn't exist" in str(e):
                self._release_camera(clear_label=True)
                self._log("Da dung camera do loi anh Tk/pyimage. Bam CONNECT CAM de mo lai.")
            else:
                raise
        except Exception as e:
            self._log(f"LOI hien thi camera: {e}")

    # ---------------- main_map legacy ROS ----------------
    def start_main_map(self):
        if self.proc and self.proc.poll() is None:
            self._log("main_map.py dang chay roi"); return
        script = Path(__file__).resolve().parent / "main_map.py"
        if not script.exists(): self._log("Khong tim thay main_map.py"); return
        env = os.environ.copy(); env.setdefault("QT_QPA_PLATFORM", "xcb"); env.setdefault("QT_X11_NO_MITSHM", "1")
        try:
            self.proc = subprocess.Popen([sys.executable, str(script)], cwd=str(script.parent), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid if hasattr(os,"setsid") else None, env=env)
            self.proc_label.configure(text=f"main_map.py: RUNNING PID {self.proc.pid}", text_color=C_GREEN)
            self._log("Da START main_map.py ROS/RViz")
        except Exception as e:
            self._log(f"Khong chay duoc main_map.py: {e}")

    def stop_main_map(self):
        if self.proc and self.proc.poll() is None:
            try:
                if hasattr(os, "killpg"): os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                else: self.proc.terminate()
            except Exception: pass
        self.proc_label.configure(text="main_map.py: OFF", text_color=C_ORANGE)
        self.stop_robot()

    def close(self):
        self._closing = True
        self.stop_camera()
        self.stop_main_map()
        # Không đóng ESP32 ở đây vì kết nối dùng chung do main/control_tab quản lý.
