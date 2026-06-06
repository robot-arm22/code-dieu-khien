"""
roi_manager.py  [v5]
=====================
Quản lý danh sách ROI. Thêm hỗ trợ:
  • Kích thước mặc định 4×4 cm (px_per_cm cấu hình được).
  • add_roi_center(name, cx, cy, w_px, h_px) – đặt ROI theo tâm điểm.
"""
import json
import os


class ROIManager:

    def __init__(self):
        self.file = "data/roi.json"
        self.rois = []
        self.load()

    # =====================================================
    # LOAD
    # =====================================================

    def load(self):
        if not os.path.exists(self.file):
            self.rois = []
            return
        try:
            with open(self.file, "r") as f:
                data = json.load(f)
            self.rois = []
            for roi in data:
                if all(k in roi for k in ("x1", "y1", "x2", "y2")):
                    self.rois.append(roi)
        except:
            self.rois = []

    # =====================================================
    # SAVE
    # =====================================================

    def save(self):
        os.makedirs("data", exist_ok=True)
        with open(self.file, "w") as f:
            json.dump(self.rois, f, indent=4)

    # =====================================================
    # ADD ROI (góc trái-trên / phải-dưới)
    # =====================================================

    def add_roi(self, name, x1, y1, x2, y2):
        roi = {
            "name": name,
            "x1": int(x1),
            "y1": int(y1),
            "x2": int(x2),
            "y2": int(y2),
        }
        self.rois.append(roi)
        self.save()

    # =====================================================
    # ADD ROI CENTER  (NEW – đặt theo tâm + kích thước)
    # =====================================================

    def add_roi_center(self, name, cx, cy, w_px, h_px):
        """Tạo ROI có tâm tại (cx, cy), kích thước w_px × h_px pixel."""
        half_w = w_px // 2
        half_h = h_px // 2
        x1 = max(0, cx - half_w)
        y1 = max(0, cy - half_h)
        x2 = cx + half_w
        y2 = cy + half_h
        self.add_roi(name, x1, y1, x2, y2)

    # =====================================================
    # DELETE ROI
    # =====================================================

    def delete_roi(self, name):
        self.rois = [r for r in self.rois if r["name"] != name]
        self.save()

    # =====================================================
    # HELPER – lấy chỉ số số của ROI_N
    # =====================================================

    @staticmethod
    def extract_index(roi_name: str) -> int | None:
        """'ROI_3' → 3,  'ROI_12' → 12,  'abc' → None"""
        try:
            parts = roi_name.split("_")
            return int(parts[-1])
        except (ValueError, IndexError):
            return None
