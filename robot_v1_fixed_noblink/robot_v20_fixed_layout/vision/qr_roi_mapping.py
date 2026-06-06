"""
qr_roi_mapping.py
=================
Quản lý bảng mapping:
  - ROI  → Pick position   (khi camera nhìn vào ROI đó sẽ đi pick ở đâu)
  - QR   → Place position  (khi quét được QR đó sẽ đặt ở vị trí nào)

Dữ liệu lưu tại data/qr_roi_mapping.json
"""
import json, os


_FILE = "data/qr_roi_mapping.json"


class QRROIMapping:
    """
    Cấu trúc nội bộ:
        roi_pick  = { "ROI_1": "pick1", "ROI_2": "pick2", ... }
        qr_place  = { "QR-001": "place1", "QR-002": "place2", ... }
    """

    def __init__(self):
        self.roi_pick: dict[str, str] = {}   # ROI name  → pick name
        self.qr_place: dict[str, str] = {}   # QR string → place name
        self.load()

    # ----------------------------------------------------------
    # LOAD / SAVE
    # ----------------------------------------------------------

    def load(self):
        if not os.path.exists(_FILE):
            self.roi_pick = {}
            self.qr_place = {}
            return
        try:
            with open(_FILE, "r") as f:
                data = json.load(f)
            self.roi_pick = data.get("roi_pick", {})
            self.qr_place = data.get("qr_place", {})
        except Exception as e:
            print(f"[MAPPING] load error: {e}")
            self.roi_pick = {}
            self.qr_place = {}

    def save(self):
        os.makedirs("data", exist_ok=True)
        with open(_FILE, "w") as f:
            json.dump({
                "roi_pick": self.roi_pick,
                "qr_place": self.qr_place,
            }, f, indent=2)

    # ----------------------------------------------------------
    # ROI → PICK
    # ----------------------------------------------------------

    def set_roi_pick(self, roi_name: str, pick_name: str):
        self.roi_pick[roi_name] = pick_name
        self.save()

    def remove_roi_pick(self, roi_name: str):
        self.roi_pick.pop(roi_name, None)
        self.save()

    def get_pick_for_roi(self, roi_name: str):
        """Trả về tên pick position cho ROI, hoặc None."""
        return self.roi_pick.get(roi_name)

    # ----------------------------------------------------------
    # QR → PLACE
    # ----------------------------------------------------------

    def set_qr_place(self, qr_code: str, place_name: str):
        self.qr_place[qr_code] = place_name
        self.save()

    def remove_qr_place(self, qr_code: str):
        self.qr_place.pop(qr_code, None)
        self.save()

    def get_place_for_qr(self, qr_code: str):
        """Trả về tên place position cho QR code, hoặc None."""
        return self.qr_place.get(qr_code)

    # ----------------------------------------------------------
    # RESOLVE FULL TASK
    # ----------------------------------------------------------

    def resolve(self, roi_name: str, qr_code: str):
        """
        Từ ROI + QR đang thấy trong thực tế, trả về (pick, place) hoặc None.
        Trả về None nếu thiếu bất kỳ vế nào.
        """
        pick  = self.get_pick_for_roi(roi_name)
        place = self.get_place_for_qr(qr_code)
        if pick and place:
            return pick, place
        return None
