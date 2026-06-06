import platform
from .cv2_safe import get_cv2


class CameraManager:

    def __init__(self):
        self.cap = None

    # =====================================================
    # OPEN CAMERA
    # =====================================================

    def open(self, index=0):
        self.close()
        cv2 = get_cv2()

        # Windows dùng CAP_DSHOW; Ubuntu/Linux dùng V4L2 hoặc backend mặc định.
        # Giữ nguyên giao diện/chức năng, chỉ đổi backend để camera mở được trên Ubuntu.
        try:
            index = int(index)
        except Exception:
            pass

        system = platform.system().lower()
        backends = []
        if "linux" in system:
            backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        elif "windows" in system:
            backends = [cv2.CAP_DSHOW, cv2.CAP_ANY]
        else:
            backends = [cv2.CAP_ANY]

        for backend in backends:
            try:
                self.cap = cv2.VideoCapture(index, backend)
            except Exception:
                self.cap = cv2.VideoCapture(index)

            if self.cap is not None and self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                return True

            try:
                if self.cap is not None:
                    self.cap.release()
            except Exception:
                pass
            self.cap = None

        print("[CAMERA FAILED]")
        return False

    # =====================================================
    # READ FRAME
    # =====================================================

    def read(self):
        if self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    # =====================================================
    # CLOSE
    # =====================================================

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
