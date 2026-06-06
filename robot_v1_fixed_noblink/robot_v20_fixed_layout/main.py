"""
main.py  – ROBOT CONTROLLER v5
"""
import customtkinter as ctk
import json, os
from pathlib import Path

from theme import apply_theme
from servo_controller import ServoController
from constants import (WINDOW_WIDTH, WINDOW_HEIGHT, DEFAULT_ANGLE,
                       C_BG, C_PANEL)

from control_tab    import ControlTab
from camera_tab     import CameraTab
from auto_tab       import AutoTab
from position_tab   import PositionTab
from mapping_tab    import MappingTab
from car_control_tab import CarControlTab
from slam_map_tab import SlamMapTab
from vision.qr_roi_mapping import QRROIMapping


# ============================================================
# POSITIONS
# ============================================================

def load_positions():
    path = "data/positions.json"
    os.makedirs("data", exist_ok=True)
    if os.path.exists(path):
        try:
            with open(path) as f:
                data = json.load(f)
            data.setdefault("pick",  {})
            data.setdefault("place", {})
            return data
        except Exception:
            pass
    default = {"pick": {}, "place": {}}
    with open(path, "w") as f:
        json.dump(default, f, indent=2)
    return default


def save_positions(positions):
    os.makedirs("data", exist_ok=True)
    with open("data/positions.json", "w") as f:
        json.dump(positions, f, indent=2)


# ============================================================
# MAIN
# ============================================================

def main():
    # Chạy đúng thư mục chương trình dù gọi từ nơi khác trên Ubuntu
    os.chdir(Path(__file__).resolve().parent)
    apply_theme()

    root = ctk.CTk()
    root.title("ROBOT CONTROLLER v20 - Camera Tracking Open/Stop Close Stable")
    root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
    root.configure(fg_color=C_BG)

    # --------------------------------------------------------
    # SHARED DATA
    # --------------------------------------------------------
    servo     = ServoController()
    positions = load_positions()
    mapping   = QRROIMapping()

    shared = {
        "servo":     servo,
        "angles":    [DEFAULT_ANGLE] * 7,
        "positions": positions,
        "last_qr":   None,
        "last_roi":  None,
        "mapping":   mapping,
    }

    # --------------------------------------------------------
    # SIDEBAR + CONTENT
    # --------------------------------------------------------
    root_frame = ctk.CTkFrame(root, fg_color=C_BG)
    root_frame.pack(fill="both", expand=True)

    sidebar = ctk.CTkFrame(root_frame, width=120, fg_color=C_PANEL, corner_radius=0)
    sidebar.pack(side="left", fill="y")
    sidebar.pack_propagate(False)

    content = ctk.CTkFrame(root_frame, fg_color=C_BG)
    content.pack(side="left", fill="both", expand=True)

    # ---- Tab frames ----
    tab_names = ["KHO HÀNG", "AUTO"]
    tab_frames = {}
    for name in tab_names:
        f = ctk.CTkFrame(content, fg_color=C_BG)
        tab_frames[name] = f

    def show_tab(name):
        for n, f in tab_frames.items():
            f.pack_forget()
        tab_frames[name].pack(fill="both", expand=True)
        for btn in nav_buttons:
            if btn._text == name:
                btn.configure(fg_color="#1e3a5f", text_color="#00d4ff")
            else:
                btn.configure(fg_color="transparent", text_color="#64748b")

    # ---- Logo ----
    ctk.CTkLabel(sidebar, text="ROBOT", font=("Consolas", 18, "bold"), text_color="#00d4ff").pack(pady=(20, 4))
    ctk.CTkLabel(sidebar, text="ROBOT\nCTRL",
                 font=("Consolas", 11, "bold"), text_color="#00d4ff",
                 justify="center").pack(pady=(0, 20))
    ctk.CTkFrame(sidebar, height=1, fg_color="#1e3a5f").pack(fill="x", padx=10, pady=6)

    # ---- Nav buttons ----
    nav_buttons = []
    for name in tab_names:
        btn = ctk.CTkButton(
            sidebar,
            text=name,
            width=100, height=46,
            font=("Consolas", 9, "bold"),
            fg_color="transparent",
            text_color="#64748b",
            hover_color="#1a2744",
            corner_radius=8,
            command=lambda n=name: show_tab(n)
        )
        btn._text = name
        btn.pack(pady=2, padx=8)
        nav_buttons.append(btn)

    # ---- Build KHO HANG with sub-tabs: kho camera + dieu khien xe ----
    kho_root = ctk.CTkFrame(tab_frames["KHO HÀNG"], fg_color=C_BG)
    kho_root.pack(fill="both", expand=True)
    kho_nav = ctk.CTkFrame(kho_root, fg_color=C_PANEL, height=48, corner_radius=0)
    kho_nav.pack(fill="x")
    kho_content = ctk.CTkFrame(kho_root, fg_color=C_BG)
    kho_content.pack(fill="both", expand=True)
    kho_camera_frame = ctk.CTkFrame(kho_content, fg_color=C_BG)
    kho_car_frame = ctk.CTkFrame(kho_content, fg_color=C_BG)
    kho_map_frame = ctk.CTkFrame(kho_content, fg_color=C_BG)

    def show_kho_subtab(name):
        kho_camera_frame.pack_forget()
        kho_car_frame.pack_forget()
        kho_map_frame.pack_forget()
        if name == "camera":
            kho_camera_frame.pack(fill="both", expand=True)
            btn_kho_cam.configure(fg_color="#1e3a5f", text_color="#00d4ff")
            btn_kho_car.configure(fg_color="transparent", text_color="#64748b")
            btn_kho_map.configure(fg_color="transparent", text_color="#64748b")
        elif name == "car":
            kho_car_frame.pack(fill="both", expand=True)
            btn_kho_cam.configure(fg_color="transparent", text_color="#64748b")
            btn_kho_car.configure(fg_color="#1e3a5f", text_color="#00d4ff")
            btn_kho_map.configure(fg_color="transparent", text_color="#64748b")
        else:
            kho_map_frame.pack(fill="both", expand=True)
            btn_kho_cam.configure(fg_color="transparent", text_color="#64748b")
            btn_kho_car.configure(fg_color="transparent", text_color="#64748b")
            btn_kho_map.configure(fg_color="#1e3a5f", text_color="#00d4ff")

    btn_kho_cam = ctk.CTkButton(kho_nav, text="KHO HÀNG / QR", width=150, height=34, font=("Consolas", 11, "bold"), command=lambda: show_kho_subtab("camera"))
    btn_kho_cam.pack(side="left", padx=(12, 6), pady=7)
    btn_kho_car = ctk.CTkButton(kho_nav, text="CONTROL XE / UDP", width=170, height=34, font=("Consolas", 11, "bold"), command=lambda: show_kho_subtab("car"))
    btn_kho_car.pack(side="left", padx=6, pady=7)
    btn_kho_map = ctk.CTkButton(kho_nav, text="BAN DO SLAM", width=140, height=34, font=("Consolas", 11, "bold"), command=lambda: show_kho_subtab("map"))
    btn_kho_map.pack(side="left", padx=6, pady=7)

    camera_tab   = CameraTab(kho_camera_frame,   shared)
    shared["camera_tab"]  = camera_tab
    car_control_tab = CarControlTab(kho_car_frame, shared)
    shared["car_control_tab"] = car_control_tab
    slam_map_tab = SlamMapTab(kho_map_frame, shared)
    shared["slam_map_tab"] = slam_map_tab
    show_kho_subtab("camera")

    # AUTO tab được tạo trước, sau đó embed Control/Mapping/Position vào sub-tabs của nó
    auto_tab     = AutoTab(tab_frames["AUTO"],         shared)
    shared["auto_tab"]    = auto_tab

    control_tab  = ControlTab(auto_tab.control_sub_frame,  shared)
    mapping_tab  = MappingTab(auto_tab.mapping_sub_frame,  shared)
    position_tab = PositionTab(auto_tab.position_sub_frame, shared)

    shared["control_tab"]  = control_tab
    shared["mapping_tab"]  = mapping_tab
    shared["position_tab"] = position_tab

    # ---- Status bar ----
    status_bar = ctk.CTkFrame(sidebar, fg_color=C_PANEL, height=60)
    status_bar.pack(side="bottom", fill="x", padx=6, pady=8)
    status_bar.pack_propagate(False)

    shared["status_dot"]  = ctk.CTkLabel(status_bar, text="*", font=("Consolas", 14),
                                          text_color="#ff3355")
    shared["status_dot"].pack(pady=(8, 0))
    shared["status_text"] = ctk.CTkLabel(status_bar, text="OFF", font=("Consolas", 8),
                                          text_color="#64748b")
    shared["status_text"].pack()

    # ---- Default tab ----
    show_tab("AUTO")

    # ---- Close ----
    def on_close():
        save_positions(shared["positions"])
        mapping.save()
        camera_tab.close_camera()
        try:
            car_control_tab.close()
        except Exception:
            pass
        try:
            slam_map_tab.close()
        except Exception:
            pass
        servo.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
