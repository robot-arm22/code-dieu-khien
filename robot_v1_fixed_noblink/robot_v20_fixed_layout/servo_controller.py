import serial
import serial.tools.list_ports
import socket
import time
import threading


class ServoController:

    def __init__(self):
        self.serial = None
        self.sock   = None
        self.udp_sock = None
        self.udp_addr = None
        self.mode   = None        # "usb" | "wifi_pi" | "wifi_esp32" | "udp_esp32"
        self._port_name = ""
        self._ip_name   = ""
        self._baudrate  = 115200
        self._lock      = threading.Lock()   # thread-safe send

    # =========================================================
    # PORTS
    # =========================================================

    def get_ports(self):
        ports = serial.tools.list_ports.comports()
        result = []
        for p in ports:
            desc  = p.description or ""
            label = f"{p.device}"
            if desc and desc != "n/a":
                label = f"{p.device}  [{desc}]"
            result.append((p.device, label))
        return result

    # =========================================================
    # CONNECT USB / COM
    # =========================================================

    def connect(self, port, baudrate=115200, retries=3):
        """
        Kết nối USB/COM với retry.
        Xử lý PermissionError(13) do Windows giữ port sau khi ngắt kết nối.
        """
        self.disconnect()
        self._baudrate = baudrate

        for attempt in range(1, retries + 1):
            try:
                ser = serial.Serial()
                ser.port      = port
                ser.baudrate  = baudrate
                ser.timeout   = 1
                ser.write_timeout = 2
                ser.open()

                with self._lock:
                    self.serial = ser
                    self.mode = "usb"
                    self._port_name = port

                time.sleep(1.5)
                print(f"[USB CONNECTED] {port} @ {baudrate}")
                return True

            except serial.SerialException as e:
                err_str = str(e)
                print(f"[USB ERROR] {err_str}")

                if "PermissionError" in err_str or "13" in err_str:
                    if attempt < retries:
                        wait = attempt * 1.5
                        print(f"[USB RETRY] Lần {attempt}/{retries}, đợi {wait:.1f}s...")
                        time.sleep(wait)
                        continue

                self.serial = None
                return False

            except Exception as e:
                print(f"[USB ERROR] {e}")
                self.serial = None
                return False

        self.serial = None
        return False

    # =========================================================
    # CONNECT PI / WIFI (qua Raspberry Pi bridge)
    # =========================================================

    def connect_pi(self, ip, port=5000, timeout=5):
        try:
            self.disconnect()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(timeout)
            self.sock.connect((ip, port))
            self.sock.settimeout(None)
            self.mode = "wifi_pi"
            self._ip_name = f"{ip}:{port}"
            print(f"[PI CONNECTED] {ip}:{port}")
            return True
        except Exception as e:
            print(f"[PI ERROR] {e}")
            self.sock = None
            return False

    # =========================================================
    # CONNECT ESP32 WIFI (kết nối trực tiếp vào ESP32 TCP server)
    # =========================================================

    def connect_esp32_wifi(self, ip, port=8080, timeout=5):
        """
        Kết nối thẳng vào TCP server chạy trên ESP32.
        ESP32 lắng nghe tại port 8080 (mặc định).
        Không cần Raspberry Pi hay bridge server.
        """
        try:
            self.disconnect()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((ip, port))
            sock.settimeout(None)

            with self._lock:
                self.sock = sock
                self.mode = "wifi_esp32"
                self._ip_name = f"{ip}:{port}"

            print(f"[ESP32 WiFi CONNECTED] {ip}:{port}")
            return True
        except ConnectionRefusedError:
            print("[ESP32 WiFi ERROR] Kết nối bị từ chối – kiểm tra IP/port và ESP32 đã khởi động WiFi chưa")
            self.sock = None
            return False
        except TimeoutError:
            print("[ESP32 WiFi ERROR] Timeout – kiểm tra ESP32 có chung mạng WiFi không")
            self.sock = None
            return False
        except Exception as e:
            print(f"[ESP32 WiFi ERROR] {e}")
            self.sock = None
            return False

    # =========================================================
    # CONNECT ESP32 UDP (dùng chung cho điều khiển tay + auto xe)
    # =========================================================

    def connect_esp32_udp(self, ip, port=8080, timeout=2):
        """
        Tạo một kênh UDP dùng chung tới ESP32.
        UDP không có bắt tay kết nối như TCP, nên hàm này ping thử một gói nhỏ
        và lưu lại địa chỉ để mọi tab dùng chung self.send_raw().
        """
        try:
            self.disconnect()
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp.setblocking(False)
            addr = (ip, int(port))
            udp.sendto(b"PING", addr)

            with self._lock:
                self.udp_sock = udp
                self.udp_addr = addr
                self.mode = "udp_esp32"
                self._ip_name = f"{ip}:{port}"

            print(f"[ESP32 UDP READY] {ip}:{port}")
            return True
        except Exception as e:
            print(f"[ESP32 UDP ERROR] {e}")
            try:
                udp.close()
            except Exception:
                pass
            self.udp_sock = None
            self.udp_addr = None
            return False

    # =========================================================
    # DISCONNECT
    # =========================================================

    def disconnect(self):
        with self._lock:
            try:
                if self.serial and self.serial.is_open:
                    self.serial.cancel_write()
            except Exception:
                pass
            try:
                if self.serial:
                    self.serial.close()
                    self.serial = None
            except Exception:
                pass
            try:
                if self.sock:
                    self.sock.close()
                    self.sock = None
            except Exception:
                pass
            try:
                if self.udp_sock:
                    self.udp_sock.close()
                    self.udp_sock = None
                    self.udp_addr = None
            except Exception:
                pass
            self.mode = None

    # =========================================================
    # IS CONNECTED
    # =========================================================

    def is_connected(self):
        return self.serial is not None or self.sock is not None or self.udp_sock is not None

    def connection_label(self):
        if self.mode == "usb":
            return f"USB  {self._port_name}"
        if self.mode == "wifi_pi":
            return f"WiFi Pi  {self._ip_name}"
        if self.mode == "wifi_esp32":
            return f"WiFi ESP32 TCP  {self._ip_name}"
        if self.mode == "udp_esp32":
            return f"WiFi ESP32 UDP  {self._ip_name}"
        return "DISCONNECTED"

    # =========================================================
    # SEND RAW
    # =========================================================

    def send_raw(self, cmd):
        text = str(cmd)
        line = (text + "\n").encode()
        with self._lock:
            if self.udp_sock and self.udp_addr:
                try:
                    self.udp_sock.sendto(text.encode(), self.udp_addr)
                except Exception as e:
                    print(f"[UDP SEND ERROR] {e}")
            elif self.sock:
                try:
                    self.sock.sendall(line)
                except Exception as e:
                    print(f"[SOCKET SEND ERROR] {e}")
            elif self.serial:
                try:
                    self.serial.write(line)
                except serial.SerialTimeoutException:
                    print("[SERIAL SEND ERROR] Write timeout – kiểm tra kết nối USB")
                except serial.SerialException as e:
                    err_str = str(e)
                    print(f"[SERIAL SEND ERROR] {err_str}")
                    if "PermissionError" in err_str or "13," in err_str or \
                       "not functioning" in err_str or "ClearCommError" in err_str:
                        print("[USB] Thiết bị bị ngắt, đóng port...")
                        try:
                            self.serial.close()
                        except Exception:
                            pass
                        self.serial = None
                        self.mode = None
                except Exception as e:
                    print(f"[SERIAL SEND ERROR] {e}")

    def read_reply_nonblocking(self):
        """Đọc phản hồi UDP nếu ESP32 có gửi lại; không chặn giao diện."""
        with self._lock:
            udp = self.udp_sock
        if not udp:
            return None
        try:
            data, _ = udp.recvfrom(1024)
            return data.decode(errors="ignore").strip()
        except BlockingIOError:
            return None
        except Exception as e:
            print(f"[UDP READ ERROR] {e}")
            return None

    # =========================================================
    # SERVO  (idx 0-5, ch0-ch4 = S0-S4, ch5 = SERVO6)
    # =========================================================

    def send_servo(self, idx, angle):
        self.send_raw(f"S{idx}:{int(angle)}")

    def send_all(self, angles):
        a = list(angles)
        while len(a) < 6:
            a.append(90)
        a = a[:6]
        cmd = "ALL:" + ",".join(str(int(v)) for v in a)
        self.send_raw(cmd)

    # =========================================================
    # TARGET
    # =========================================================

    def send_target1(self, angle):
        self.send_raw(f"TARGET1:{int(angle)}")

    def send_target2(self, angle):
        self.send_raw(f"TARGET2:{int(angle)}")

    # =========================================================
    # CAR
    # =========================================================

    def send_car(self, action, speed=150):
        self.send_raw(f"CAR:{action},{int(speed)}")

    def stop_car(self):        self.send_car("STOP", 0)
    def forward(self, s=150):  self.send_car("FORWARD", s)
    def backward(self, s=150): self.send_car("BACKWARD", s)
    def left(self, s=150):     self.send_car("LEFT", s)
    def right(self, s=150):    self.send_car("RIGHT", s)
    def rotate_left(self, s=150):  self.send_car("ROTATE_LEFT", s)
    def rotate_right(self, s=150): self.send_car("ROTATE_RIGHT", s)

    # =========================================================
    # RESET ESP32 (USB only)
    # =========================================================

    def reset_esp32(self):
        if not self.serial:
            return False
        try:
            self.serial.dtr = False
            self.serial.rts = True
            time.sleep(0.2)
            self.serial.dtr = True
            self.serial.rts = False
            time.sleep(0.1)
            print("[ESP32 RESET]")
            return True
        except Exception as e:
            print(f"[RESET ERROR] {e}")
            return False

    # =========================================================
    # PING
    # =========================================================

    def ping(self):
        self.send_raw("PING")
