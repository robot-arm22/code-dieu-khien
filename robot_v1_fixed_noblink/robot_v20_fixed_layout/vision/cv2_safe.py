"""Lazy OpenCV loader for Ubuntu."""
_cv2 = None

def get_cv2():
    global _cv2
    if _cv2 is not None:
        return _cv2
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError(f"Không import được OpenCV/cv2: {exc}") from exc
    _cv2 = cv2
    return _cv2
