"""
mapping_tab.py
==============
Tab cấu hình mapping:
  ① ROI Mapping   — Mỗi ROI gắn với 1 Pick position
  ② QR Mapping    — Mỗi QR code gắn với 1 Place position

Dữ liệu lưu qua QRROIMapping và được shared["mapping"] trỏ đến,
auto_tab đọc trực tiếp để điều hướng robot.
"""
import customtkinter as ctk
from constants import *
from vision.qr_roi_mapping import QRROIMapping


def _card(parent, **kw):
    return ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=10, **kw)


class MappingTab:

    def __init__(self, parent, shared):
        self.parent  = parent
        self.shared  = shared
        self.mapping: QRROIMapping = shared["mapping"]
        self._build_ui()
        self._auto_refresh()

    # ==============================================================
    # BUILD UI
    # ==============================================================

    def _build_ui(self):
        root = ctk.CTkFrame(self.parent, fg_color=C_BG)
        root.pack(fill="both", expand=True)

        # ── Header ────────────────────────────────────────────────
        hdr = ctk.CTkFrame(root, fg_color=C_PANEL, corner_radius=10, height=52)
        hdr.pack(fill="x", padx=8, pady=(8, 4))
        hdr.pack_propagate(False)

        ctk.CTkLabel(hdr, text="LINK  MAPPING  SETUP",
                     font=("Consolas", 14, "bold"),
                     text_color=C_ACCENT).pack(side="left", padx=16)

        ctk.CTkButton(hdr, text="SAVE SAVE ALL",
                      fg_color=C_ACCENT2, hover_color="#5b21b6",
                      font=("Consolas", 11, "bold"), width=120, height=32,
                      command=self._save_all
                      ).pack(side="right", padx=12, pady=8)

        # ── Two-column layout ────────────────────────────────────
        cols = ctk.CTkFrame(root, fg_color=C_BG)
        cols.pack(fill="both", expand=True, padx=8, pady=4)

        self._build_roi_panel(cols)
        self._build_qr_panel(cols)

    # ==============================================================
    # LEFT: ROI → PICK
    # ==============================================================

    def _build_roi_panel(self, parent):
        panel = ctk.CTkFrame(parent, fg_color=C_PANEL, corner_radius=10)
        panel.pack(side="left", fill="both", expand=True, padx=(0, 4), pady=4)

        # Title
        ctk.CTkLabel(panel, text="⬡  ROI  →  PICK  POSITION",
                     font=("Consolas", 13, "bold"),
                     text_color="#10b981").pack(anchor="w", padx=16, pady=(12, 2))
        ctk.CTkLabel(panel,
                     text="Chọn ROI và Pick position tương ứng rồi nhấn + ADD\n"
                          "TIP Auto: ROI_1→pick1, ROI_2→pick2 … (tự động khi tạo ROI)",
                     font=("Consolas", 9), text_color=C_SUBTEXT
                     ).pack(anchor="w", padx=16, pady=(0, 6))
        ctk.CTkFrame(panel, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)

        # Input row
        inp = ctk.CTkFrame(panel, fg_color="transparent")
        inp.pack(fill="x", padx=12, pady=8)

        ctk.CTkLabel(inp, text="ROI", font=("Consolas", 10, "bold"),
                     text_color=C_SUBTEXT, width=48).pack(side="left", padx=(0, 4))
        self.roi_sel = ctk.CTkComboBox(inp, values=["NO ROI"],
                                        font=("Consolas", 11))
        self.roi_sel.pack(side="left", fill="x", expand=True, padx=4)

        ctk.CTkLabel(inp, text="→", font=("Consolas", 13),
                     text_color=C_ACCENT).pack(side="left", padx=6)

        ctk.CTkLabel(inp, text="PICK", font=("Consolas", 10, "bold"),
                     text_color=C_SUBTEXT, width=38).pack(side="left", padx=(0, 4))
        self.pick_sel = ctk.CTkComboBox(inp, values=["NO PICK"],
                                         font=("Consolas", 11))
        self.pick_sel.pack(side="left", fill="x", expand=True, padx=4)

        ctk.CTkButton(inp, text="+ ADD", width=72, height=32,
                      fg_color="#15803d", hover_color="#166534",
                      font=("Consolas", 11, "bold"),
                      command=self._add_roi_mapping
                      ).pack(side="left", padx=4)

        # Live status
        self.roi_live_lbl = ctk.CTkLabel(panel,
                                          text="Live ROI: —",
                                          font=("Consolas", 10),
                                          text_color=C_SUBTEXT)
        self.roi_live_lbl.pack(anchor="w", padx=16, pady=(0, 4))

        # List
        self.roi_list_box = ctk.CTkScrollableFrame(panel, fg_color=C_BG,
                                                    scrollbar_button_color=C_BORDER)
        self.roi_list_box.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        self._refresh_roi_list()

    # ==============================================================
    # RIGHT: QR → PLACE
    # ==============================================================

    def _build_qr_panel(self, parent):
        panel = ctk.CTkFrame(parent, fg_color=C_PANEL, corner_radius=10)
        panel.pack(side="left", fill="both", expand=True, padx=(4, 0), pady=4)

        # Title
        ctk.CTkLabel(panel, text="⬡  QR  CODE  →  PLACE  POSITION",
                     font=("Consolas", 13, "bold"),
                     text_color="#3b82f6").pack(anchor="w", padx=16, pady=(12, 2))
        ctk.CTkLabel(panel,
                     text="Nhập QR code và Place position tương ứng rồi nhấn + ADD\n"
                          "TIP Auto: QR-001→place1, QR-002→place2 … (tự động khi camera đọc)",
                     font=("Consolas", 9), text_color=C_SUBTEXT
                     ).pack(anchor="w", padx=16, pady=(0, 6))
        ctk.CTkFrame(panel, height=1, fg_color=C_BORDER).pack(fill="x", padx=12)

        # Input row
        inp = ctk.CTkFrame(panel, fg_color="transparent")
        inp.pack(fill="x", padx=12, pady=8)

        ctk.CTkLabel(inp, text="QR", font=("Consolas", 10, "bold"),
                     text_color=C_SUBTEXT, width=24).pack(side="left", padx=(0, 4))
        self.qr_entry = ctk.CTkEntry(inp,
                                      font=("Consolas", 12, "bold"),
                                      text_color=C_GREEN,
                                      placeholder_text="VD: QR-001")
        self.qr_entry.pack(side="left", fill="x", expand=True, padx=4)

        # Quick-fill button: bấm để tự điền QR đang thấy
        ctk.CTkButton(inp, text="CAM FILL", width=72, height=32,
                      fg_color=C_ACCENT2, hover_color="#5b21b6",
                      font=("Consolas", 10, "bold"),
                      command=self._fill_live_qr
                      ).pack(side="left", padx=4)

        ctk.CTkLabel(inp, text="→", font=("Consolas", 13),
                     text_color=C_ACCENT).pack(side="left", padx=6)

        ctk.CTkLabel(inp, text="PLACE", font=("Consolas", 10, "bold"),
                     text_color=C_SUBTEXT, width=42).pack(side="left", padx=(0, 4))
        self.place_sel = ctk.CTkComboBox(inp, values=["NO PLACE"],
                                          font=("Consolas", 11))
        self.place_sel.pack(side="left", fill="x", expand=True, padx=4)

        ctk.CTkButton(inp, text="+ ADD", width=72, height=32,
                      fg_color="#15803d", hover_color="#166534",
                      font=("Consolas", 11, "bold"),
                      command=self._add_qr_mapping
                      ).pack(side="left", padx=4)

        # Live status
        self.qr_live_lbl = ctk.CTkLabel(panel,
                                         text="Live QR: —",
                                         font=("Consolas", 10),
                                         text_color=C_SUBTEXT)
        self.qr_live_lbl.pack(anchor="w", padx=16, pady=(0, 4))

        # List
        self.qr_list_box = ctk.CTkScrollableFrame(panel, fg_color=C_BG,
                                                   scrollbar_button_color=C_BORDER)
        self.qr_list_box.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        self._refresh_qr_list()

    # ==============================================================
    # REFRESH DROPDOWNS  (called by auto_refresh & externally)
    # ==============================================================

    def refresh_data(self):
        """Cập nhật dropdown ROI và PICK/PLACE từ shared data."""
        # ROI list
        roi_vals = []
        try:
            roi_vals = [r["name"] for r in self.shared["roi_manager"].rois]
        except Exception:
            pass
        if not roi_vals:
            roi_vals = ["NO ROI"]
        cur = self.roi_sel.get()
        self.roi_sel.configure(values=roi_vals)
        if cur not in roi_vals:
            self.roi_sel.set(roi_vals[0])

        # Pick list
        pick_vals = list(self.shared["positions"]["pick"].keys()) or ["NO PICK"]
        cur = self.pick_sel.get()
        self.pick_sel.configure(values=pick_vals)
        if cur not in pick_vals:
            self.pick_sel.set(pick_vals[0])

        # Place list
        place_vals = list(self.shared["positions"]["place"].keys()) or ["NO PLACE"]
        cur = self.place_sel.get()
        self.place_sel.configure(values=place_vals)
        if cur not in place_vals:
            self.place_sel.set(place_vals[0])

        # Live status labels
        live_roi = self.shared.get("last_roi") or "—"
        live_qr  = self.shared.get("last_qr")  or "—"

        # ROI live
        mapped_pick = self.mapping.get_pick_for_roi(live_roi) if live_roi != "—" else None
        self.roi_live_lbl.configure(
            text=f"Live ROI: {live_roi}  →  Pick: {mapped_pick or '(chưa map)'}",
            text_color=C_GREEN if mapped_pick else C_SUBTEXT
        )

        # QR live
        mapped_place = self.mapping.get_place_for_qr(live_qr) if live_qr != "—" else None
        self.qr_live_lbl.configure(
            text=f"Live QR: {live_qr}  →  Place: {mapped_place or '(chưa map)'}",
            text_color="#3b82f6" if mapped_place else C_SUBTEXT
        )

    def _auto_refresh(self):
        try:
            self.refresh_data()
        except Exception:
            pass
        self.parent.after(700, self._auto_refresh)

    # ==============================================================
    # ADD / DELETE  ROI MAPPING
    # ==============================================================

    def _add_roi_mapping(self):
        roi  = self.roi_sel.get().strip()
        pick = self.pick_sel.get().strip()
        if roi in ("", "NO ROI") or pick in ("", "NO PICK"):
            return
        self.mapping.set_roi_pick(roi, pick)
        self._refresh_roi_list()

    def _delete_roi_mapping(self, roi_name):
        self.mapping.remove_roi_pick(roi_name)
        self._refresh_roi_list()

    def _refresh_roi_list(self):
        for w in self.roi_list_box.winfo_children():
            w.destroy()

        if not self.mapping.roi_pick:
            ctk.CTkLabel(self.roi_list_box,
                         text="Chưa có mapping nào",
                         font=("Consolas", 11),
                         text_color=C_SUBTEXT
                         ).pack(pady=20)
            return

        for roi_name, pick_name in self.mapping.roi_pick.items():
            row = ctk.CTkFrame(self.roi_list_box, fg_color=C_CARD, corner_radius=8)
            row.pack(fill="x", padx=4, pady=3)

            # ROI badge
            badge = ctk.CTkFrame(row, fg_color="#064e3b", corner_radius=6, width=80)
            badge.pack(side="left", padx=8, pady=8)
            badge.pack_propagate(False)
            ctk.CTkLabel(badge, text=roi_name,
                         font=("Consolas", 11, "bold"),
                         text_color="#10b981").pack(padx=6, pady=4)

            ctk.CTkLabel(row, text="→",
                         font=("Consolas", 14, "bold"),
                         text_color=C_ACCENT).pack(side="left", padx=6)

            ctk.CTkLabel(row, text=pick_name,
                         font=("Consolas", 12, "bold"),
                         text_color=C_TEXT).pack(side="left", fill="x", expand=True)

            ctk.CTkButton(row, text="X", width=28, height=28,
                          font=("Consolas", 10),
                          fg_color=C_RED, hover_color="#991b1b",
                          command=lambda r=roi_name: self._delete_roi_mapping(r)
                          ).pack(side="right", padx=8, pady=8)

    # ==============================================================
    # ADD / DELETE  QR MAPPING
    # ==============================================================

    def _fill_live_qr(self):
        """Điền QR code đang thấy trong camera vào ô nhập."""
        live = self.shared.get("last_qr")
        if live:
            self.qr_entry.delete(0, "end")
            self.qr_entry.insert(0, live)

    def _add_qr_mapping(self):
        qr    = self.qr_entry.get().strip()
        place = self.place_sel.get().strip()
        if not qr or place in ("", "NO PLACE"):
            return
        self.mapping.set_qr_place(qr, place)
        self._refresh_qr_list()

    def _delete_qr_mapping(self, qr_code):
        self.mapping.remove_qr_place(qr_code)
        self._refresh_qr_list()

    def _refresh_qr_list(self):
        for w in self.qr_list_box.winfo_children():
            w.destroy()

        if not self.mapping.qr_place:
            ctk.CTkLabel(self.qr_list_box,
                         text="Chưa có mapping nào",
                         font=("Consolas", 11),
                         text_color=C_SUBTEXT
                         ).pack(pady=20)
            return

        for qr_code, place_name in self.mapping.qr_place.items():
            row = ctk.CTkFrame(self.qr_list_box, fg_color=C_CARD, corner_radius=8)
            row.pack(fill="x", padx=4, pady=3)

            # QR badge
            badge = ctk.CTkFrame(row, fg_color="#1e3a5f", corner_radius=6, width=110)
            badge.pack(side="left", padx=8, pady=8)
            badge.pack_propagate(False)
            ctk.CTkLabel(badge, text=qr_code,
                         font=("Consolas", 11, "bold"),
                         text_color=C_GREEN).pack(padx=6, pady=4)

            ctk.CTkLabel(row, text="→",
                         font=("Consolas", 14, "bold"),
                         text_color=C_ACCENT).pack(side="left", padx=6)

            ctk.CTkLabel(row, text=place_name,
                         font=("Consolas", 12, "bold"),
                         text_color=C_TEXT).pack(side="left", fill="x", expand=True)

            ctk.CTkButton(row, text="X", width=28, height=28,
                          font=("Consolas", 10),
                          fg_color=C_RED, hover_color="#991b1b",
                          command=lambda q=qr_code: self._delete_qr_mapping(q)
                          ).pack(side="right", padx=8, pady=8)

    # ==============================================================
    # SAVE ALL
    # ==============================================================

    def _save_all(self):
        self.mapping.save()
        print("[MAPPING] Đã lưu toàn bộ mapping → data/qr_roi_mapping.json")
