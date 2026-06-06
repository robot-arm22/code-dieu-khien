import customtkinter as ctk
import json, os
from constants import *


class PositionTab:

    def __init__(self, parent, shared):
        self.parent = parent
        self.shared = shared
        self._build_ui()

    def _build_ui(self):
        root = ctk.CTkFrame(self.parent, fg_color=C_BG)
        root.pack(fill="both", expand=True)

        hdr = ctk.CTkFrame(root, fg_color=C_PANEL, corner_radius=10, height=48)
        hdr.pack(fill="x", padx=8, pady=(8, 4))
        hdr.pack_propagate(False)
        ctk.CTkLabel(hdr, text="POS  SAVED  POSITIONS",
                     font=("Consolas", 14, "bold"),
                     text_color=C_ACCENT).pack(side="left", padx=16, pady=12)

        ctk.CTkButton(hdr, text="SAVE EXPORT JSON",
                      fg_color=C_ACCENT2, hover_color="#5b21b6",
                      font=("Consolas", 11, "bold"), width=140, height=32,
                      command=self._export).pack(side="right", padx=12, pady=8)

        cols = ctk.CTkFrame(root, fg_color=C_BG)
        cols.pack(fill="both", expand=True, padx=8, pady=4)

        for kind, color, label in [("pick", "#10b981", "PICK POSITIONS"),
                                     ("place", "#3b82f6", "PLACE POSITIONS")]:
            col = ctk.CTkFrame(cols, fg_color=C_PANEL, corner_radius=10)
            col.pack(side="left", fill="both", expand=True, padx=4, pady=4)

            ctk.CTkLabel(col, text=label,
                         font=("Consolas", 13, "bold"),
                         text_color=color).pack(anchor="w", padx=16, pady=(12, 4))
            ctk.CTkFrame(col, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)

            box = ctk.CTkScrollableFrame(col, fg_color=C_BG)
            box.pack(fill="both", expand=True, padx=8, pady=8)

            setattr(self, f"{kind}_box", box)

        self._refresh()

    def _refresh(self):
        for kind in ("pick", "place"):
            box = getattr(self, f"{kind}_box")
            for w in box.winfo_children():
                w.destroy()
            color = "#10b981" if kind == "pick" else "#3b82f6"
            for name, angles in self.shared["positions"][kind].items():
                row = ctk.CTkFrame(box, fg_color=C_CARD, corner_radius=8)
                row.pack(fill="x", padx=4, pady=3)

                ctk.CTkLabel(row, text=name,
                             font=("Consolas", 12, "bold"),
                             text_color=C_TEXT, anchor="w"
                             ).pack(side="left", padx=10, pady=8, fill="x", expand=True)

                angles_str = ", ".join(str(a) for a in angles[:5]) + "…"
                ctk.CTkLabel(row, text=angles_str,
                             font=("Consolas", 9),
                             text_color=C_SUBTEXT
                             ).pack(side="left", padx=6)

                ctk.CTkButton(row, text="> RUN", width=64, height=28,
                              font=("Consolas", 10, "bold"),
                              fg_color=color, hover_color=C_BORDER,
                              command=lambda k=kind, n=name: self._run(k, n)
                              ).pack(side="right", padx=4, pady=6)
                ctk.CTkButton(row, text="X", width=28, height=28,
                              font=("Consolas", 10),
                              fg_color=C_RED, hover_color="#991b1b",
                              command=lambda k=kind, n=name: self._delete(k, n)
                              ).pack(side="right", padx=(0, 2), pady=6)

    def _run(self, kind, name):
        ctrl = self.shared.get("control_tab")
        if not ctrl:
            return
        if kind == "pick":
            ctrl.run_pick(name)
        else:
            ctrl.run_place(name)

    def _delete(self, kind, name):
        self.shared["positions"][kind].pop(name, None)
        self._refresh()
        try:
            self.shared["control_tab"].refresh_position_lists()
        except:
            pass

    def _export(self):
        os.makedirs("data", exist_ok=True)
        with open("data/positions.json", "w") as f:
            json.dump(self.shared["positions"], f, indent=2)
        print("[EXPORT] data/positions.json saved")
