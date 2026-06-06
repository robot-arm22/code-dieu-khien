import customtkinter as ctk
import threading, time
from constants import *

# ============================================================
# Cấu trúc angles[] trong shared:
#   idx 0-4  →  S0..S4  →  PCA9685 ch0-ch4
#   idx 5    →  TARGET1 →  PCA9685 ch5  (servo 6)
#   idx 6    →  TARGET2 →  PCA9685 ch5  (servo 6)
# Cả TARGET1 và TARGET2 đều điều khiển cùng 1 servo vật lý (ch5)
# ============================================================


def _card(parent, **kw):
    return ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=10, **kw)


def _section_title(parent, text):
    ctk.CTkLabel(
        parent, text=text,
        font=("Consolas", 13, "bold"),
        text_color=C_ACCENT
    ).pack(anchor="w", padx=14, pady=(12, 4))


class ControlTab:

    def __init__(self, parent, shared):
        self.parent = parent
        self.shared = shared
        self.angles = self.shared["angles"]  # list 7 phần tử

        # ── Smooth slider state (1°/giây) ───────────────────
        self._slider_targets = list(self.angles)   # góc đích mỗi servo
        self._slider_moving  = [False] * 7          # thread đang chạy?

        # ── Root split ──────────────────────────────────────
        root = ctk.CTkFrame(parent, fg_color=C_BG)
        root.pack(fill="both", expand=True)

        self.left = ctk.CTkScrollableFrame(root, width=480, fg_color=C_BG,
                                            scrollbar_button_color=C_BORDER)
        self.left.pack(side="left", fill="y", padx=(6, 3), pady=6)

        self.right = ctk.CTkFrame(root, fg_color=C_PANEL, corner_radius=10)
        self.right.pack(side="left", fill="both", expand=True, padx=(3, 6), pady=6)

        ctk.CTkLabel(
            self.right, text="ARM  VISUALIZER\n(coming soon)",
            font=("Consolas", 16), text_color=C_SUBTEXT
        ).place(relx=0.5, rely=0.5, anchor="center")

        self._build_connection_card()
        self._build_servo_card()
        self._build_position_card()

    # ============================================================
    # CONNECTION CARD
    # ============================================================

    def _build_connection_card(self):
        card = _card(self.left)
        card.pack(fill="x", padx=6, pady=6)

        hdr = ctk.CTkFrame(card, fg_color="transparent")
        hdr.pack(fill="x", padx=14, pady=(10, 6))
        ctk.CTkLabel(hdr, text="ESP32  CONNECTION",
                     font=("Consolas", 14, "bold"),
                     text_color=C_ACCENT).pack(side="left")

        # ── USB ──────────────────────────────────────────────
        ctk.CTkFrame(card, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)
        ctk.CTkLabel(card, text="USB / COM PORT",
                     font=("Consolas", 10, "bold"),
                     text_color=C_SUBTEXT).pack(anchor="w", padx=14, pady=(8, 2))

        usb_row = ctk.CTkFrame(card, fg_color="transparent")
        usb_row.pack(fill="x", padx=10, pady=4)

        ports_raw = self.shared["servo"].get_ports()
        labels = [lbl for _, lbl in ports_raw] if ports_raw else ["NO PORT"]
        self._port_map = {lbl: dev for dev, lbl in ports_raw}

        self.com_menu = ctk.CTkComboBox(usb_row, values=labels,
                                         font=("Consolas", 11), width=260)
        self.com_menu.set(labels[0])
        self.com_menu.pack(side="left", padx=(0, 6))

        ctk.CTkButton(usb_row, text="REF", width=36, height=32,
                      font=("Consolas", 16),
                      fg_color=C_CARD, hover_color=C_BORDER,
                      command=self._refresh_ports).pack(side="left")

        btn_usb = ctk.CTkFrame(card, fg_color="transparent")
        btn_usb.pack(fill="x", padx=10, pady=(2, 8))

        ctk.CTkButton(btn_usb, text="USB CONNECT USB",
                      fg_color="#15803d", hover_color="#166534",
                      font=("Consolas", 12, "bold"), height=38,
                      command=self._connect_usb).pack(side="left", fill="x",
                                                       expand=True, padx=(0, 4))
        ctk.CTkButton(btn_usb, text="RESET RESET",
                      fg_color=C_ORANGE, hover_color="#b45309",
                      font=("Consolas", 12, "bold"), height=38,
                      command=self._reset_esp32).pack(side="left", padx=(0, 4))
        ctk.CTkButton(btn_usb, text="X DISC",
                      fg_color=C_RED, hover_color="#991b1b",
                      font=("Consolas", 12, "bold"), height=38,
                      command=self._disconnect).pack(side="left")

        # ── ESP32 WiFi UDP dùng chung ─────────────────────────
        ctk.CTkFrame(card, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)
        ctk.CTkLabel(card, text="ESP32  WiFi UDP  DÙNG CHUNG",
                     font=("Consolas", 10, "bold"),
                     text_color=C_ORANGE).pack(anchor="w", padx=14, pady=(8, 2))

        esp_row = ctk.CTkFrame(card, fg_color="transparent")
        esp_row.pack(fill="x", padx=10, pady=4)

        self.esp_ip_entry = ctk.CTkEntry(esp_row, placeholder_text="IP ESP32 (vd: 192.168.1.250)",
                                          font=("Consolas", 12), width=220)
        self.esp_ip_entry.insert(0, "192.168.1.250")
        self.esp_ip_entry.pack(side="left", padx=(0, 6))

        self.esp_port_entry = ctk.CTkEntry(esp_row, placeholder_text="8080",
                                            font=("Consolas", 12), width=70)
        self.esp_port_entry.insert(0, "8080")
        self.esp_port_entry.pack(side="left", padx=(0, 6))

        ctk.CTkButton(esp_row, text="CONNECT ESP32 UDP",
                      fg_color=C_ORANGE, hover_color="#b45309",
                      font=("Consolas", 12, "bold"), height=38,
                      command=self._connect_esp32_udp).pack(side="left")

        ctk.CTkLabel(card,
                     text="  ⓘ Kết nối ESP32 một lần tại đây; CONTROL XE và AUTO sẽ dùng chung kết nối này",
                     font=("Consolas", 9), text_color=C_SUBTEXT
                     ).pack(anchor="w", padx=14, pady=(0, 6))

        # ── Status badge ──────────────────────────────────────
        status_row = ctk.CTkFrame(card, fg_color="transparent")
        status_row.pack(fill="x", padx=10, pady=(4, 12))

        self.status_dot = ctk.CTkLabel(status_row, text="*",
                                        font=("Consolas", 18),
                                        text_color=C_RED)
        self.status_dot.pack(side="left", padx=(4, 4))

        self.status_label = ctk.CTkLabel(status_row, text="DISCONNECTED",
                                          font=("Consolas", 13, "bold"),
                                          text_color=C_RED)
        self.status_label.pack(side="left")

    # ── actions ──────────────────────────────────────────────

    def _refresh_ports(self):
        ports_raw = self.shared["servo"].get_ports()
        labels = [lbl for _, lbl in ports_raw] if ports_raw else ["NO PORT"]
        self._port_map = {lbl: dev for dev, lbl in ports_raw}
        self.com_menu.configure(values=labels)
        self.com_menu.set(labels[0])

    def _connect_usb(self):
        lbl = self.com_menu.get()
        port = self._port_map.get(lbl, lbl.split()[0])
        if port == "NO PORT":
            return
        threading.Thread(target=self.__do_connect_usb, args=(port,), daemon=True).start()

    def __do_connect_usb(self, port):
        ok = self.shared["servo"].connect(port)
        self._update_status(ok, f"USB  {port}")

    def _connect_pi(self):
        # Giữ hàm để tương thích nếu file cũ còn gọi, nhưng giao diện Pi đã được bỏ để tránh thừa kết nối.
        return

    def __do_connect_pi(self, ip, port):
        ok = self.shared["servo"].connect_pi(ip, port)
        self._update_status(ok, f"WiFi Pi  {ip}:{port}")

    def _connect_esp32_wifi(self):
        ip = self.esp_ip_entry.get().strip()
        if not ip:
            return
        try:
            port = int(self.esp_port_entry.get().strip() or "8080")
        except:
            port = 8080
        threading.Thread(target=self.__do_connect_esp32_wifi, args=(ip, port), daemon=True).start()

    def __do_connect_esp32_wifi(self, ip, port):
        ok = self.shared["servo"].connect_esp32_wifi(ip, port)
        self._update_status(ok, f"WiFi ESP32 TCP  {ip}:{port}")

    def _connect_esp32_udp(self):
        ip = self.esp_ip_entry.get().strip()
        if not ip:
            return
        try:
            port = int(self.esp_port_entry.get().strip() or "8080")
        except Exception:
            port = 8080
        threading.Thread(target=self.__do_connect_esp32_udp, args=(ip, port), daemon=True).start()

    def __do_connect_esp32_udp(self, ip, port):
        ok = self.shared["servo"].connect_esp32_udp(ip, port)
        self._update_status(ok, f"WiFi ESP32 UDP  {ip}:{port}")

    def _disconnect(self):
        self.shared["servo"].disconnect()
        self._update_status(False, "DISCONNECTED")

    def _reset_esp32(self):
        ok = self.shared["servo"].reset_esp32()
        if ok:
            self._flash_status("RESET OK", C_ORANGE)

    def _update_status(self, ok, text):
        color = C_GREEN if ok else C_RED
        self.status_dot.configure(text_color=color)
        self.status_label.configure(text=text, text_color=color)
        try:
            self.shared["status_dot"].configure(text_color=color)
            self.shared["status_text"].configure(
                text="ON" if ok else "OFF", text_color=color)
        except:
            pass

    def _flash_status(self, text, color):
        self.status_dot.configure(text_color=color)
        self.status_label.configure(text=text, text_color=color)

    # ============================================================
    # SERVO CARD
    # ============================================================

    def _build_servo_card(self):
        card = _card(self.left)
        card.pack(fill="x", padx=6, pady=6)

        ctk.CTkLabel(card, text="ARM  CONTROL  —  PCA9685",
                     font=("Consolas", 14, "bold"),
                     text_color=C_ACCENT).pack(anchor="w", padx=14, pady=(10, 2))

        # Ghi chú nhỏ về TARGET1/TARGET2
        ctk.CTkLabel(card,
                     text="  TARGET1 & TARGET2 → cùng servo ch5  (PCA9685)",
                     font=("Consolas", 9), text_color=C_SUBTEXT
                     ).pack(anchor="w", padx=14, pady=(0, 4))

        ctk.CTkFrame(card, height=1, fg_color=C_BORDER).pack(fill="x", padx=12, pady=2)

        self.slider_vars   = []
        self.angle_labels  = []
        self.angle_entries = []

        # ── 7 slider: S0-S4 + TARGET1 + TARGET2 ──────────────
        # idx 0-4 → PCA9685 ch0-ch4 (arm)
        # idx 5   → PCA9685 ch5 qua lệnh TARGET1
        # idx 6   → PCA9685 ch5 qua lệnh TARGET2
        names  = ["S0", "S1", "S2", "S3", "S4", "TARGET 1", "TARGET 2"]
        colors = [C_ACCENT, "#3b82f6", C_ACCENT2, "#10b981",
                  C_ORANGE, C_YELLOW, C_RED]

        for i, name in enumerate(names):

            # Divider trước TARGET group
            if i == 5:
                ctk.CTkFrame(card, height=1, fg_color=C_BORDER).pack(
                    fill="x", padx=12, pady=(6, 2))
                ctk.CTkLabel(card,
                             text="  SERVO 6  (ch5)  –  có thể dùng TARGET1 hoặc TARGET2",
                             font=("Consolas", 9, "bold"),
                             text_color=C_YELLOW).pack(anchor="w", padx=14, pady=(0, 2))

            row = ctk.CTkFrame(card, fg_color="transparent")
            row.pack(fill="x", padx=10, pady=3)

            ctk.CTkLabel(row, text=name,
                         font=("Consolas", 11, "bold"),
                         text_color=colors[i],
                         width=72, anchor="w").pack(side="left", padx=(4, 0))

            slider = ctk.CTkSlider(row, from_=0, to=180,
                                   progress_color=colors[i],
                                   command=lambda v, idx=i: self.on_servo_change(idx, v))
            slider.set(self.angles[i])
            slider.pack(side="left", fill="x", expand=True, padx=6)
            self.slider_vars.append(slider)

            lbl = ctk.CTkLabel(row, text=f"{self.angles[i]:>3}°",
                                font=("Consolas", 12, "bold"),
                                text_color=colors[i], width=46)
            lbl.pack(side="left", padx=2)
            self.angle_labels.append(lbl)

            ent = ctk.CTkEntry(row, width=52, font=("Consolas", 11))
            ent.insert(0, str(self.angles[i]))
            ent.pack(side="left", padx=4)
            ent.bind("<Return>", lambda e, idx=i, en=ent: self._manual_angle(idx, en))
            self.angle_entries.append(ent)

        # Send all button
        ctk.CTkButton(card, text="-> SEND ALL  (ch0-ch5)",
                      fg_color=C_ACCENT2, hover_color="#5b21b6",
                      font=("Consolas", 12, "bold"), height=36,
                      command=self._send_all_now
                      ).pack(fill="x", padx=10, pady=(6, 12))

    def on_servo_change(self, idx, value):
        """Xử lý khi slider thay đổi — smooth move 1°/giây trong thread riêng.
        idx 0-4 → S<idx>:<angle>  → PCA9685 ch0-ch4
        idx 5   → TARGET1:<angle> → PCA9685 ch5
        idx 6   → TARGET2:<angle> → PCA9685 ch5
        """
        target = int(value)
        # Cập nhật label/entry ngay lập tức
        self.angle_labels[idx].configure(text=f"{target:>3}°")
        self.angle_entries[idx].delete(0, "end")
        self.angle_entries[idx].insert(0, str(target))

        # Ghi target mới, thread đang chạy sẽ tự đọc
        self._slider_targets[idx] = target

        # Nếu chưa có thread đang chạy cho servo này → khởi động
        if not self._slider_moving[idx]:
            self._slider_moving[idx] = True
            threading.Thread(
                target=self._smooth_slider_thread,
                args=(idx,),
                daemon=True
            ).start()

    def _smooth_slider_thread(self, idx):
        """Chạy smooth move 1 giây / 1°. Nếu target thay đổi giữa chừng,
        tự động cập nhật đích mới mà không khởi động lại thread."""
        servo = self.shared["servo"]

        def _send(angle):
            if idx <= 4:
                servo.send_servo(idx, angle)
            elif idx == 5:
                servo.send_target1(angle)
            elif idx == 6:
                servo.send_target2(angle)

        DEGREE_DELAY = 1 / 10   # 10° / giây  →  10° ≈ 1 giây

        while True:
            current = self.angles[idx]
            target  = self._slider_targets[idx]

            if current == target:
                # Đã đến đích — kết thúc thread
                self._slider_moving[idx] = False
                break

            # Tiến thêm 1° về phía target
            step = 1 if target > current else -1
            next_angle = current + step
            next_angle = max(0, min(180, next_angle))

            _send(next_angle)
            self.angles[idx] = next_angle
            self.shared["angles"] = self.angles

            # Cập nhật UI an toàn từ thread
            def _update_ui(a=next_angle, i=idx):
                try:
                    self.slider_vars[i].set(a)
                    self.angle_labels[i].configure(text=f"{a:>3}°")
                    self.angle_entries[i].delete(0, "end")
                    self.angle_entries[i].insert(0, str(a))
                except Exception:
                    pass
            try:
                self.parent.after(0, _update_ui)
            except Exception:
                pass

            time.sleep(DEGREE_DELAY)

    def _manual_angle(self, idx, entry):
        try:
            angle = max(0, min(180, int(entry.get())))
            self.slider_vars[idx].set(angle)
            self._slider_targets[idx] = angle
            self.angle_labels[idx].configure(text=f"{angle:>3}°")
            if not self._slider_moving[idx]:
                self._slider_moving[idx] = True
                threading.Thread(
                    target=self._smooth_slider_thread,
                    args=(idx,),
                    daemon=True
                ).start()
        except Exception:
            pass

    def _send_all_now(self):
        """Gửi ALL với 6 giá trị cho PCA9685 ch0-ch5.
        ch5 lấy giá trị của TARGET1 (angles[5]).
        """
        arm_angles = list(self.angles[:5])   # ch0-ch4
        ch5_angle  = self.angles[5]           # TARGET1 → ch5
        self.shared["servo"].send_all(arm_angles + [ch5_angle])

    # ============================================================
    # POSITION CARD
    # ============================================================

    def _build_position_card(self):
        card = _card(self.left)
        card.pack(fill="x", padx=6, pady=6)

        ctk.CTkLabel(card, text="SAVED  POSITIONS",
                     font=("Consolas", 14, "bold"),
                     text_color=C_ACCENT).pack(anchor="w", padx=14, pady=(10, 4))
        ctk.CTkFrame(card, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)

        # -- PICK --
        ctk.CTkLabel(card, text="PICK", font=("Consolas", 11, "bold"),
                     text_color="#10b981").pack(anchor="w", padx=14, pady=(8, 2))

        add_pick_row = ctk.CTkFrame(card, fg_color="transparent")
        add_pick_row.pack(fill="x", padx=10, pady=2)
        self.pick_entry = ctk.CTkEntry(add_pick_row, placeholder_text="Pick name…",
                                        font=("Consolas", 11))
        self.pick_entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
        ctk.CTkButton(add_pick_row, text="+ ADD",
                      fg_color="#15803d", hover_color="#166534",
                      font=("Consolas", 11, "bold"), width=64, height=30,
                      command=self._add_pick).pack(side="left")

        self.pick_box = ctk.CTkScrollableFrame(card, height=120, fg_color=C_BG,
                                                corner_radius=6)
        self.pick_box.pack(fill="x", padx=10, pady=(2, 8))

        # -- PLACE --
        ctk.CTkFrame(card, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)
        ctk.CTkLabel(card, text="PLACE", font=("Consolas", 11, "bold"),
                     text_color="#3b82f6").pack(anchor="w", padx=14, pady=(8, 2))

        add_place_row = ctk.CTkFrame(card, fg_color="transparent")
        add_place_row.pack(fill="x", padx=10, pady=2)
        self.place_entry = ctk.CTkEntry(add_place_row, placeholder_text="Place name…",
                                         font=("Consolas", 11))
        self.place_entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
        ctk.CTkButton(add_place_row, text="+ ADD",
                      fg_color="#1d4ed8", hover_color="#1e3a8a",
                      font=("Consolas", 11, "bold"), width=64, height=30,
                      command=self._add_place).pack(side="left")

        self.place_box = ctk.CTkScrollableFrame(card, height=120, fg_color=C_BG,
                                                 corner_radius=6)
        self.place_box.pack(fill="x", padx=10, pady=(2, 12))

        self.refresh_position_lists()

    def _add_pick(self):
        name = self.pick_entry.get().strip()
        if not name:
            return
        # Lưu 7 giá trị: [S0,S1,S2,S3,S4, T1_angle, T2_angle]
        self.shared["positions"]["pick"][name] = list(self.angles)
        self.pick_entry.delete(0, "end")
        self.refresh_position_lists()

    def _add_place(self):
        name = self.place_entry.get().strip()
        if not name:
            return
        self.shared["positions"]["place"][name] = list(self.angles)
        self.place_entry.delete(0, "end")
        self.refresh_position_lists()

    def _del_pick(self, name):
        self.shared["positions"]["pick"].pop(name, None)
        self.refresh_position_lists()

    def _del_place(self, name):
        self.shared["positions"]["place"].pop(name, None)
        self.refresh_position_lists()

    def refresh_position_lists(self):
        for w in self.pick_box.winfo_children():
            w.destroy()
        for w in self.place_box.winfo_children():
            w.destroy()
        for name in self.shared["positions"]["pick"]:
            self._pos_row(self.pick_box, name, "#10b981",
                          lambda b, s, n=name: self.run_pick(n, b, s),
                          lambda n=name: self._del_pick(n))
        for name in self.shared["positions"]["place"]:
            self._pos_row(self.place_box, name, "#3b82f6",
                          lambda b, s, n=name: self.run_place(n, b, s),
                          lambda n=name: self._del_place(n))

    def _pos_row(self, parent, name, color, run_cb, del_cb):
        """Mỗi hàng position gồm: tên | status | [RUN] [X]
        run_cb(btn, status_lbl) được gọi khi nhấn RUN.
        """
        outer = ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=6)
        outer.pack(fill="x", padx=3, pady=2)

        # Tên vị trí
        ctk.CTkLabel(outer, text=name, font=("Consolas", 11),
                     text_color=C_TEXT, anchor="w").pack(side="left",
                                                          fill="x", expand=True, padx=8)

        # Nhãn trạng thái bước hiện tại
        status_lbl = ctk.CTkLabel(outer, text="──",
                                   font=("Consolas", 9), text_color=C_SUBTEXT, width=64)
        status_lbl.pack(side="left", padx=2)

        # Nút xóa
        ctk.CTkButton(outer, text="X", width=28, height=26,
                      font=("Consolas", 10),
                      fg_color=C_RED, hover_color="#991b1b",
                      command=del_cb).pack(side="right", padx=(0, 4), pady=4)

        # Nút RUN (tạo sau để có thể truyền vào run_cb)
        run_btn = ctk.CTkButton(outer, text="RUN", width=52, height=26,
                                 font=("Consolas", 10, "bold"),
                                 fg_color=color, hover_color=C_BORDER)
        run_btn.configure(command=lambda b=run_btn, s=status_lbl: run_cb(b, s))
        run_btn.pack(side="right", padx=2, pady=4)

    # ============================================================
    # MOVE
    # ============================================================

    # Tốc độ chung: 10° / giây  →  10° ≈ 1 s
    _DEG_PER_SEC = 10
    _DEGREE_DELAY = 1 / _DEG_PER_SEC   # giây / 1°

    def _send_servo_cmd(self, idx, angle):
        servo = self.shared["servo"]
        if idx <= 4:
            servo.send_servo(idx, angle)
        elif idx == 5:
            servo.send_target1(angle)
        elif idx == 6:
            servo.send_target2(angle)

    def move_sequence(self, sequence, status_cb=None):
        """Chạy lần lượt từng servo: S0 → … → TARGET2.
        Mỗi servo di chuyển smooth 3°/giây (cùng tốc độ slider).
        status_cb(step_name) được gọi khi bắt đầu mỗi servo.
        """
        full = list(sequence)
        while len(full) < 7:
            full.append(90)

        names = ["S0", "S1", "S2", "S3", "S4", "TARGET1", "TARGET2"]

        for idx, target in enumerate(full):
            target = max(0, min(180, int(target)))

            if status_cb:
                try:
                    self.parent.after(0, lambda n=names[idx]: status_cb(n))
                except Exception:
                    pass

            current = self.angles[idx]
            step = 1 if target > current else -1

            # TARGET1 (idx=5) và TARGET2 (idx=6) là lệnh gripper — phải gửi
            # dù góc hiện tại == góc đích (để gripper thực sự nhận lệnh).
            if current == target and idx in (5, 6):
                try:
                    self._send_servo_cmd(idx, target)
                except Exception as e:
                    print(f"[SEQ ERROR] {names[idx]}: {e}")
                self.angles[idx] = target
                time.sleep(self._DEGREE_DELAY)
                continue

            # Di chuyển từng độ một
            pos = current
            while pos != target:
                pos += step
                pos = max(0, min(180, pos))

                try:
                    self._send_servo_cmd(idx, pos)
                except Exception as e:
                    print(f"[SEQ ERROR] {names[idx]}: {e}")

                self.angles[idx] = pos
                self.shared["angles"] = self.angles

                def _ui(i=idx, a=pos):
                    try:
                        self.slider_vars[i].set(a)
                        self.angle_labels[i].configure(text=f"{a:>3}°")
                        self.angle_entries[i].delete(0, "end")
                        self.angle_entries[i].insert(0, str(a))
                    except Exception:
                        pass
                try:
                    self.parent.after(0, _ui)
                except Exception:
                    pass

                time.sleep(self._DEGREE_DELAY)

        self.shared["angles"] = list(full)

        if status_cb:
            try:
                self.parent.after(0, lambda: status_cb("DONE ✓"))
            except Exception:
                pass

    def _run_sequence_threaded(self, seq, btn, status_lbl):
        """Chạy sequence trong thread, disable nút RUN khi đang chạy."""
        def _worker():
            try:
                btn.configure(state="disabled", text=">>")
            except Exception:
                pass

            def _set_status(name):
                try:
                    status_lbl.configure(text=name)
                except Exception:
                    pass

            self.move_sequence(seq, status_cb=_set_status)

            try:
                btn.configure(state="normal", text="RUN")
            except Exception:
                pass

        threading.Thread(target=_worker, daemon=True).start()

    def run_pick(self, name, btn=None, status_lbl=None):
        seq = self.shared["positions"]["pick"].get(name)
        if not seq:
            return
        if btn and status_lbl:
            self._run_sequence_threaded(seq, btn, status_lbl)
        else:
            threading.Thread(target=self.move_sequence, args=(seq,), daemon=True).start()

    def run_place(self, name, btn=None, status_lbl=None):
        seq = self.shared["positions"]["place"].get(name)
        if not seq:
            return
        if btn and status_lbl:
            self._run_sequence_threaded(seq, btn, status_lbl)
        else:
            threading.Thread(target=self.move_sequence, args=(seq,), daemon=True).start()
