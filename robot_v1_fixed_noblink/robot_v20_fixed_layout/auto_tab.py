"""
auto_tab.py  – v17  «LẤY HÀNG»
================================
Tab duy nhất: Lấy Hàng
  • Hiển thị danh sách QR từ tab Kho Hàng (camera_tab._inventory_order)
  • Chọn một hoặc nhiều QR -> xem mapping pick/place tương ứng
  • Nhấn > CHẠY -> robot thực hiện pick->place theo thứ tự đã chọn
  • Trạng thái từng mục: ... chờ / > đang chạy / OK xong / X lỗi
  • Nút DỪNG dừng ngay sau bước hiện tại
  • Tự động làm mới danh sách kho mỗi 1 giây

Sub-frames được nhúng bởi main.py:
  auto_tab.control_sub_frame   ← ControlTab
  auto_tab.mapping_sub_frame   ← MappingTab
  auto_tab.position_sub_frame  ← PositionTab
"""

import customtkinter as ctk
import threading
import time
from constants import (
    C_BG, C_CARD, C_PANEL, C_BORDER,
    C_TEXT, C_SUBTEXT, C_ACCENT,
    C_GREEN, C_RED, C_YELLOW, C_ORANGE,
)


# ─────────────────────────────────────────────────────────────────────────────
# Màu badge trạng thái
# ─────────────────────────────────────────────────────────────────────────────
_STATUS = {
    "pending": ("#1e293b", C_SUBTEXT,  "..."),
    "running": ("#1e3a5f", C_ACCENT,   ">"),
    "done":    ("#14532d", C_GREEN,    "OK"),
    "error":   ("#450a0a", C_RED,      "X"),
    "skip":    ("#2d1f00", C_ORANGE,   "!"),
}


def _card(parent, **kw):
    return ctk.CTkFrame(parent, fg_color=C_CARD, corner_radius=10, **kw)


# ─────────────────────────────────────────────────────────────────────────────

class AutoTab:
    """Quản lý toàn bộ tab AUTO (sidebar embed + tab Lấy Hàng)."""

    def __init__(self, parent, shared):
        self.parent = parent
        self.shared = shared

        # Trạng thái chạy hàng
        self._queue: list[str] = []          # QR code đã chọn theo thứ tự
        self._status: dict[str, str] = {}    # qr -> pending/running/done/error/skip
        self._running = False
        self._stop_flag = False

        # Cache để tránh rebuild UI không cần thiết (gây nhấp nháy)
        self._last_inv_snapshot: list = []   # snapshot kho hàng lần trước
        self._last_queue_snapshot: list = [] # snapshot queue lần trước
        self._last_status_snapshot: dict = {}

        self._build_ui()
        self._tick()  # vòng lặp refresh 1 s

    # ═════════════════════════════════════════════════════════════════════════
    # BUILD UI
    # ═════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        root = ctk.CTkFrame(self.parent, fg_color=C_BG)
        root.pack(fill="both", expand=True)

        # TabView chính: Lấy Hàng / Control / Mapping / Position
        self._tv = ctk.CTkTabview(
            root,
            fg_color=C_CARD,
            segmented_button_fg_color=C_PANEL,
            segmented_button_selected_color=C_ACCENT,
            segmented_button_selected_hover_color="#1e40af",
            segmented_button_unselected_color=C_PANEL,
            segmented_button_unselected_hover_color="#1e2a44",
            text_color=C_TEXT,
            text_color_disabled=C_SUBTEXT,
            corner_radius=10,
        )
        self._tv.pack(fill="both", expand=True, padx=6, pady=6)

        # Trên một số bản Ubuntu/Tk thiếu font emoji, CTkTabview có thể lỗi khi tên tab có icon.
        # Mặc định vẫn giữ nguyên icon/bố cục như bản gốc; nếu máy thiếu font sẽ tự fallback sang tên không icon.
        tab_specs = {
            "lay_hang": ("LẤY HÀNG", "LẤY HÀNG"),
            "control":  ("⚙ CONTROL", "CONTROL"),
            "mapping":  ("LINK MAPPING", "MAPPING"),
            "position": ("POS POSITION", "POSITION"),
        }
        self._tab_names = {}
        for key, (label, fallback) in tab_specs.items():
            try:
                self._tv.add(label)
                self._tab_names[key] = label
            except Exception:
                self._tv.add(fallback)
                self._tab_names[key] = fallback

        # Sub-frames dành cho main.py nhúng vào
        self.control_sub_frame  = self._tv.tab(self._tab_names["control"])
        self.mapping_sub_frame  = self._tv.tab(self._tab_names["mapping"])
        self.position_sub_frame = self._tv.tab(self._tab_names["position"])

        self._build_lay_hang_tab(self._tv.tab(self._tab_names["lay_hang"]))

    # ─────────────────────────────────────────────────────────────────────────
    # TAB: LẤY HÀNG
    # ─────────────────────────────────────────────────────────────────────────

    def _build_lay_hang_tab(self, parent):
        root = ctk.CTkFrame(parent, fg_color=C_BG)
        root.pack(fill="both", expand=True)

        # ── Header bar ───────────────────────────────────────────────────────
        hdr = ctk.CTkFrame(root, fg_color=C_PANEL, corner_radius=10, height=54)
        hdr.pack(fill="x", padx=8, pady=(8, 4))
        hdr.pack_propagate(False)

        ctk.CTkLabel(
            hdr, text=" LẤY  HÀNG  -  PICK -> PLACE",
            font=("Consolas", 15, "bold"), text_color=C_ACCENT,
        ).pack(side="left", padx=16, pady=12)

        self._status_lbl = ctk.CTkLabel(
            hdr, text="* IDLE",
            font=("Consolas", 11, "bold"), text_color=C_SUBTEXT,
        )
        self._status_lbl.pack(side="right", padx=16)

        # ── Body: cột trái (kho) + cột phải (queue) ──────────────────────────
        body = ctk.CTkFrame(root, fg_color=C_BG)
        body.pack(fill="both", expand=True, padx=8, pady=4)

        self._build_inventory_panel(body)
        self._build_queue_panel(body)

    # ── Cột trái: Danh sách kho hàng ─────────────────────────────────────────

    def _build_inventory_panel(self, parent):
        frame = ctk.CTkFrame(parent, fg_color=C_PANEL, corner_radius=10, width=310)
        frame.pack(side="left", fill="y", padx=(0, 4), pady=4)
        frame.pack_propagate(False)

        # Tiêu đề
        th = ctk.CTkFrame(frame, fg_color="transparent")
        th.pack(fill="x", padx=12, pady=(10, 4))
        ctk.CTkLabel(
            th, text="KHO HÀNG",
            font=("Consolas", 13, "bold"), text_color="#10b981",
        ).pack(side="left")
        self._inv_count_lbl = ctk.CTkLabel(
            th, text="0 mục",
            font=("Consolas", 10), text_color=C_SUBTEXT,
        )
        self._inv_count_lbl.pack(side="right")

        ctk.CTkLabel(
            frame,
            text="Click để chọn / bỏ chọn\nGiữ Ctrl (hoặc click nhiều) để chọn nhiều",
            font=("Consolas", 9), text_color=C_SUBTEXT, justify="left",
        ).pack(anchor="w", padx=14, pady=(0, 4))
        ctk.CTkFrame(frame, height=1, fg_color=C_BORDER).pack(fill="x", padx=10)

        # Nút Chọn tất cả / Bỏ chọn
        btns = ctk.CTkFrame(frame, fg_color="transparent")
        btns.pack(fill="x", padx=8, pady=6)
        ctk.CTkButton(
            btns, text="OK Chọn tất", width=120, height=28,
            font=("Consolas", 10, "bold"),
            fg_color="#15803d", hover_color="#166534",
            command=self._select_all,
        ).pack(side="left", padx=2)
        ctk.CTkButton(
            btns, text="X Bỏ chọn", width=120, height=28,
            font=("Consolas", 10, "bold"),
            fg_color="#450a0a", hover_color="#7f1d1d",
            command=self._deselect_all,
        ).pack(side="left", padx=2)

        # Danh sách cuộn
        self._inv_scroll = ctk.CTkScrollableFrame(
            frame, fg_color=C_BG, corner_radius=8,
            scrollbar_button_color=C_BORDER,
        )
        self._inv_scroll.pack(fill="both", expand=True, padx=8, pady=(0, 10))

    # ── Cột phải: Queue + điều khiển ─────────────────────────────────────────

    def _build_queue_panel(self, parent):
        frame = ctk.CTkFrame(parent, fg_color=C_PANEL, corner_radius=10)
        frame.pack(side="left", fill="both", expand=True, padx=(4, 0), pady=4)

        # Tiêu đề
        th = ctk.CTkFrame(frame, fg_color="transparent")
        th.pack(fill="x", padx=12, pady=(10, 4))
        ctk.CTkLabel(
            th, text="HÀNG CHỜ",
            font=("Consolas", 13, "bold"), text_color=C_ACCENT,
        ).pack(side="left")
        self._queue_count_lbl = ctk.CTkLabel(
            th, text="0 gói",
            font=("Consolas", 10), text_color=C_SUBTEXT,
        )
        self._queue_count_lbl.pack(side="right")

        ctk.CTkLabel(
            frame,
            text="Thứ tự chạy - dùng UPDOWN đổi vị trí, X bỏ khỏi queue",
            font=("Consolas", 9), text_color=C_SUBTEXT,
        ).pack(anchor="w", padx=14, pady=(0, 4))
        ctk.CTkFrame(frame, height=1, fg_color=C_BORDER).pack(fill="x", padx=10)

        # Mapping preview
        self._mapping_lbl = ctk.CTkLabel(
            frame, text="",
            font=("Consolas", 9), text_color=C_SUBTEXT,
            justify="left", wraplength=400,
        )
        self._mapping_lbl.pack(anchor="w", padx=14, pady=(4, 0))

        # Danh sách queue
        self._queue_scroll = ctk.CTkScrollableFrame(
            frame, fg_color=C_BG, corner_radius=8,
            scrollbar_button_color=C_BORDER,
        )
        self._queue_scroll.pack(fill="both", expand=True, padx=8, pady=(4, 6))

        # Nút điều khiển
        ctrl = ctk.CTkFrame(frame, fg_color="transparent")
        ctrl.pack(fill="x", padx=8, pady=(0, 8))

        self._run_btn = ctk.CTkButton(
            ctrl, text=">  CHẠY",
            fg_color="#7c3aed", hover_color="#5b21b6",
            font=("Consolas", 14, "bold"), height=48,
            command=self._start,
        )
        self._run_btn.pack(side="left", fill="x", expand=True, padx=(0, 4))

        self._stop_btn = ctk.CTkButton(
            ctrl, text="STOP DỪNG",
            fg_color=C_RED, hover_color="#991b1b",
            font=("Consolas", 12, "bold"), height=48, width=90,
            command=self._stop,
        )
        self._stop_btn.pack(side="left")

        self._progress_lbl = ctk.CTkLabel(
            frame, text="* IDLE - chọn hàng và nhấn CHẠY",
            font=("Consolas", 10, "bold"), text_color=C_SUBTEXT,
        )
        self._progress_lbl.pack(pady=(0, 4))

    # ═════════════════════════════════════════════════════════════════════════
    # REFRESH INVENTORY (đọc từ camera_tab)
    # ═════════════════════════════════════════════════════════════════════════

    def _get_inventory(self) -> list[str]:
        """Trả về danh sách QR hiện có trong kho (camera_tab._inventory_order)."""
        try:
            cam = self.shared.get("camera_tab")
            if cam and hasattr(cam, "_inventory_order"):
                return list(cam._inventory_order)
        except Exception:
            pass
        return []

    def _refresh_inventory_ui(self):
        """Vẽ lại danh sách kho bên trái (chỉ khi dữ liệu thay đổi)."""
        inv = self._get_inventory()

        # Xóa các QR không còn trong kho khỏi queue
        queue_changed = False
        for qr in list(self._queue):
            if qr not in inv:
                self._queue.remove(qr)
                self._status.pop(qr, None)
                queue_changed = True

        # Kiểm tra xem dữ liệu có thay đổi không
        inv_snapshot = list(inv)
        queue_snapshot = list(self._queue)
        inv_changed = (inv_snapshot != self._last_inv_snapshot)
        sel_changed = (queue_snapshot != self._last_queue_snapshot)

        # Nếu không có gì thay đổi, không rebuild UI (tránh nhấp nháy)
        if not inv_changed and not sel_changed and not queue_changed:
            return

        self._last_inv_snapshot = inv_snapshot
        self._last_queue_snapshot = queue_snapshot

        self._inv_count_lbl.configure(text=f"{len(inv)} mục")

        # Xóa widget cũ
        for w in self._inv_scroll.winfo_children():
            w.destroy()

        if not inv:
            ctk.CTkLabel(
                self._inv_scroll,
                text="Chưa có hàng nào\nMở tab KHO HÀNG và bật camera để quét QR",
                font=("Consolas", 10), text_color=C_SUBTEXT,
                justify="center", wraplength=260,
            ).pack(pady=30)
            return

        mapping = self.shared.get("mapping")
        cam = self.shared.get("camera_tab")

        for qr in inv:
            selected = qr in self._queue

            # Lấy thông tin mapping
            place = mapping.get_place_for_qr(qr) if mapping else None
            roi_name = None
            try:
                if cam and hasattr(cam, "_inventory"):
                    with cam._lock:
                        roi_name = cam._inventory.get(qr, {}).get("roi")
            except Exception:
                pass
            pick = mapping.get_pick_for_roi(roi_name) if mapping and roi_name else None

            # Màu theo mapping
            if pick and place:
                map_text = f"  {pick} -> {place}"
                map_color = C_GREEN
            elif place:
                map_text = f"  ! thiếu pick  -> {place}"
                map_color = C_YELLOW
            elif pick:
                map_text = f"  {pick} -> ! thiếu place"
                map_color = C_YELLOW
            else:
                map_text = "  ! chưa có mapping"
                map_color = C_ORANGE

            # Row
            row = ctk.CTkFrame(
                self._inv_scroll,
                fg_color="#14532d" if selected else C_CARD,
                corner_radius=8,
            )
            row.pack(fill="x", padx=4, pady=3)

            # Checkbox indicator
            ctk.CTkLabel(
                row,
                text="OK" if selected else "O",
                font=("Consolas", 14, "bold"),
                text_color=C_GREEN if selected else C_SUBTEXT,
                width=26,
            ).pack(side="left", padx=(10, 2), pady=8)

            # QR name
            ctk.CTkLabel(
                row, text=qr,
                font=("Consolas", 12, "bold"),
                text_color=C_GREEN if selected else C_TEXT,
                anchor="w",
            ).pack(side="left", fill="x", expand=True, pady=8)

            # Mapping info
            ctk.CTkLabel(
                row, text=map_text,
                font=("Consolas", 9),
                text_color=map_color,
            ).pack(side="right", padx=10)

            # Click handler (disable khi đang chạy)
            if not self._running:
                row.bind("<Button-1>", lambda e, q=qr: self._toggle(q))
                for child in row.winfo_children():
                    child.bind("<Button-1>", lambda e, q=qr: self._toggle(q))

    # ═════════════════════════════════════════════════════════════════════════
    # QUEUE MANAGEMENT
    # ═════════════════════════════════════════════════════════════════════════

    def _toggle(self, qr: str):
        """Chọn / bỏ chọn một QR."""
        if self._running:
            return
        if qr in self._queue:
            self._queue.remove(qr)
            self._status.pop(qr, None)
        else:
            self._queue.append(qr)
            self._status[qr] = "pending"
        self._refresh_inventory_ui()
        self._render_queue(force=True)

    def _select_all(self):
        if self._running:
            return
        inv = self._get_inventory()
        for qr in inv:
            if qr not in self._queue:
                self._queue.append(qr)
                self._status[qr] = "pending"
        self._refresh_inventory_ui()
        self._render_queue(force=True)

    def _deselect_all(self):
        if self._running:
            return
        self._queue.clear()
        self._status.clear()
        self._refresh_inventory_ui()
        self._render_queue(force=True)

    def _move(self, idx: int, delta: int):
        ni = idx + delta
        if 0 <= ni < len(self._queue):
            self._queue[idx], self._queue[ni] = self._queue[ni], self._queue[idx]
        self._render_queue(force=True)

    def _remove(self, qr: str):
        if qr in self._queue:
            self._queue.remove(qr)
        self._status.pop(qr, None)
        self._refresh_inventory_ui()
        self._render_queue(force=True)

    def _render_queue(self, force=False):
        """Vẽ lại bảng hàng chờ bên phải (chỉ khi dữ liệu thay đổi)."""
        queue_snapshot = list(self._queue)
        status_snapshot = dict(self._status)

        # Chỉ rebuild khi có thay đổi thực sự (tránh nhấp nháy)
        if not force:
            if (queue_snapshot == self._last_queue_snapshot and
                    status_snapshot == self._last_status_snapshot):
                return

        self._last_queue_snapshot = queue_snapshot
        self._last_status_snapshot = status_snapshot

        for w in self._queue_scroll.winfo_children():
            w.destroy()

        n = len(self._queue)
        self._queue_count_lbl.configure(text=f"{n} gói")

        if n == 0:
            ctk.CTkLabel(
                self._queue_scroll,
                text="Chưa có hàng nào được chọn\nClick vào mục ở danh sách bên trái để thêm",
                font=("Consolas", 10), text_color=C_SUBTEXT,
                justify="center", wraplength=380,
            ).pack(pady=30)
            # Cập nhật mapping preview
            self._mapping_lbl.configure(text="")
            return

        # Mapping preview tổng hợp
        mapping = self.shared.get("mapping")
        preview_parts = []
        cam = self.shared.get("camera_tab")
        for qr in self._queue:
            place = mapping.get_place_for_qr(qr) if mapping else None
            roi_name = None
            try:
                if cam and hasattr(cam, "_inventory"):
                    with cam._lock:
                        roi_name = cam._inventory.get(qr, {}).get("roi")
            except Exception:
                pass
            pick = mapping.get_pick_for_roi(roi_name) if mapping and roi_name else None
            if pick and place:
                preview_parts.append(f"{qr}: {pick}->{place}")
            else:
                preview_parts.append(f"{qr}: !")
        self._mapping_lbl.configure(text="  " + "   |   ".join(preview_parts[:6]))

        # Rows
        status_label = {
            "pending": "chờ chạy",
            "running": "đang chạy...",
            "done":    "hoàn thành",
            "error":   "lỗi / skip",
            "skip":    "bỏ qua",
        }

        for i, qr in enumerate(self._queue):
            st = self._status.get(qr, "pending")
            bg, fg, icon = _STATUS.get(st, _STATUS["pending"])

            row = ctk.CTkFrame(self._queue_scroll, fg_color=bg, corner_radius=8)
            row.pack(fill="x", padx=6, pady=3)

            # Số thứ tự
            ctk.CTkLabel(
                row, text=f"{i+1:02d}",
                font=("Consolas", 13, "bold"), text_color=C_SUBTEXT, width=30,
            ).pack(side="left", padx=(10, 4), pady=8)

            # Badge icon
            ctk.CTkLabel(
                row, text=icon,
                font=("Consolas", 13), text_color=fg, width=22,
            ).pack(side="left", padx=(0, 4))

            # QR name
            ctk.CTkLabel(
                row, text=qr,
                font=("Consolas", 12, "bold"), text_color=fg, anchor="w",
            ).pack(side="left", fill="x", expand=True, pady=8)

            # Trạng thái text
            ctk.CTkLabel(
                row, text=status_label.get(st, ""),
                font=("Consolas", 9), text_color=fg,
            ).pack(side="right", padx=8)

            # Nút UP DOWN X (chỉ khi không chạy)
            if not self._running:
                btn_frame = ctk.CTkFrame(row, fg_color="transparent")
                btn_frame.pack(side="right", padx=4)
                if i > 0:
                    ctk.CTkButton(
                        btn_frame, text="UP", width=28, height=28,
                        font=("Consolas", 12),
                        fg_color=C_PANEL, hover_color=C_BORDER,
                        command=lambda idx=i: self._move(idx, -1),
                    ).pack(side="left", padx=1)
                if i < n - 1:
                    ctk.CTkButton(
                        btn_frame, text="DOWN", width=28, height=28,
                        font=("Consolas", 12),
                        fg_color=C_PANEL, hover_color=C_BORDER,
                        command=lambda idx=i: self._move(idx, +1),
                    ).pack(side="left", padx=1)
                ctk.CTkButton(
                    btn_frame, text="X", width=28, height=28,
                    font=("Consolas", 10),
                    fg_color=C_RED, hover_color="#991b1b",
                    command=lambda q=qr: self._remove(q),
                ).pack(side="left", padx=(2, 0))

    # ═════════════════════════════════════════════════════════════════════════
    # CHẠY / DỪNG
    # ═════════════════════════════════════════════════════════════════════════

    def _start(self):
        if self._running:
            return
        if not self._queue:
            self._progress_lbl.configure(
                text="! Chưa chọn hàng nào - click vào kho để chọn",
                text_color=C_ORANGE,
            )
            return

        # Kiểm tra mapping trước khi chạy
        missing = []
        mapping = self.shared.get("mapping")
        cam = self.shared.get("camera_tab")
        for qr in self._queue:
            place = mapping.get_place_for_qr(qr) if mapping else None
            roi_name = None
            try:
                if cam and hasattr(cam, "_inventory"):
                    with cam._lock:
                        roi_name = cam._inventory.get(qr, {}).get("roi")
            except Exception:
                pass
            pick = mapping.get_pick_for_roi(roi_name) if mapping and roi_name else None
            if not pick or not place:
                missing.append(qr)

        if missing:
            self._progress_lbl.configure(
                text=f"! {len(missing)} gói thiếu mapping: {', '.join(missing[:3])}{'...' if len(missing)>3 else ''}",
                text_color=C_ORANGE,
            )
            # Vẫn cho chạy, các gói thiếu mapping sẽ bị skip

        # Reset status
        for qr in self._queue:
            self._status[qr] = "pending"

        self._running = True
        self._stop_flag = False
        self._run_btn.configure(state="disabled", fg_color="#374151")
        self._status_lbl.configure(text="> ĐANG CHẠY", text_color="#7c3aed")
        self._render_queue(force=True)

        threading.Thread(target=self._run_thread, daemon=True).start()

    def _stop(self):
        self._stop_flag = True
        self._progress_lbl.configure(
            text="STOP Đang dừng sau bước hiện tại...",
            text_color=C_ORANGE,
        )

    def _run_thread(self):
        total = len(self._queue)
        done = 0
        skipped = 0

        mapping = self.shared.get("mapping")
        ctrl    = self.shared.get("control_tab")
        cam     = self.shared.get("camera_tab")

        for idx, qr in enumerate(list(self._queue)):
            if self._stop_flag:
                # Đánh dấu các mục còn lại là pending (không chạy)
                break

            # Lấy mapping
            place = mapping.get_place_for_qr(qr) if mapping else None
            roi_name = None
            try:
                if cam and hasattr(cam, "_inventory"):
                    with cam._lock:
                        roi_name = cam._inventory.get(qr, {}).get("roi")
            except Exception:
                pass
            pick = mapping.get_pick_for_roi(roi_name) if mapping and roi_name else None

            if not pick or not place:
                print(f"[LẤY HÀNG] [{idx+1}/{total}] {qr} – bỏ qua (thiếu mapping: pick={pick}, place={place})")
                self._set_status(qr, "skip")
                skipped += 1
                continue

            # Cập nhật UI
            self._set_status(qr, "running")
            self._ui(lambda i=idx, t=total, q=qr, pk=pick, pl=place: (
                self._progress_lbl.configure(
                    text=f">  [{i+1}/{t}]  {q}  ->  {pk} -> {pl}",
                    text_color=C_ACCENT,
                ),
                self._status_lbl.configure(
                    text=f"> [{i+1}/{t}]",
                    text_color="#7c3aed",
                ),
            ))

            print(f"[LẤY HÀNG] [{idx+1}/{total}] {qr}  pick={pick}  place={place}")

            try:
                if ctrl:
                    # run_pick là blocking (dùng move_sequence trực tiếp)
                    seq_pick  = self.shared["positions"]["pick"].get(pick)
                    seq_place = self.shared["positions"]["place"].get(place)

                    if seq_pick:
                        ctrl.move_sequence(seq_pick)
                    else:
                        print(f"[LẤY HÀNG]   ! không tìm thấy sequence pick: {pick}")

                    time.sleep(0.3)

                    if seq_place:
                        ctrl.move_sequence(seq_place)
                    else:
                        print(f"[LẤY HÀNG]   ! không tìm thấy sequence place: {place}")

                    self._set_status(qr, "done")
                    done += 1
                    print("[LẤY HÀNG]   OK xong")
                else:
                    print("[LẤY HÀNG]   ! control_tab chưa sẵn sàng")
                    self._set_status(qr, "error")

            except Exception as e:
                print(f"[LẤY HÀNG]   X lỗi: {e}")
                self._set_status(qr, "error")

            time.sleep(0.5)  # nghỉ ngắn giữa các gói

        # Hoàn thành
        self._running = False

        def _finish():
            self._run_btn.configure(state="normal", fg_color="#7c3aed")
            if self._stop_flag:
                self._status_lbl.configure(text="STOP ĐÃ DỪNG", text_color=C_RED)
                self._progress_lbl.configure(
                    text=f"STOP Đã dừng - {done}/{total} gói hoàn thành",
                    text_color=C_RED,
                )
            elif done + skipped == total:
                color = C_GREEN if done == total else C_YELLOW
                self._status_lbl.configure(text=f"OK XONG {done}/{total}", text_color=color)
                self._progress_lbl.configure(
                    text=f"OK Hoàn thành {done}/{total} gói"
                         + (f"  (bỏ qua {skipped} thiếu mapping)" if skipped else ""),
                    text_color=color,
                )
            else:
                self._status_lbl.configure(text=f"OK DONE {done}/{total}", text_color=C_YELLOW)
                self._progress_lbl.configure(
                    text=f"OK {done} thành công / {skipped} bỏ qua / {total-done-skipped} lỗi",
                    text_color=C_YELLOW,
                )
            self._render_queue(force=True)

        self._ui(_finish)

    # ═════════════════════════════════════════════════════════════════════════
    # HELPERS
    # ═════════════════════════════════════════════════════════════════════════

    def _set_status(self, qr: str, st: str):
        """Cập nhật trạng thái và re-render queue (thread-safe)."""
        self._status[qr] = st
        self._ui(lambda: self._render_queue(force=True))

    def _ui(self, fn):
        """Chạy hàm fn trên main thread."""
        try:
            self.parent.after(0, fn)
        except Exception:
            pass

    def _tick(self):
        """Vòng lặp 1 s: làm mới danh sách kho và queue."""
        try:
            self._refresh_inventory_ui()
            if not self._running:
                self._render_queue()  # không force - chỉ render khi có thay đổi
        except Exception:
            pass
        try:
            self.parent.after(1000, self._tick)
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Backward-compat: các hàm cũ main.py hoặc module khác có thể gọi
    # ─────────────────────────────────────────────────────────────────────────

    def refresh_data(self):
        """Tương thích ngược – gọi khi dữ liệu thay đổi."""
        try:
            self._refresh_inventory_ui()
            self._render_queue(force=True)
        except Exception:
            pass
