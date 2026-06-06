BAN V12 - Camera calib points tren live tracking

Bản Ubuntu v3 - giữ bố cục và chức năng phần mềm gốc.

Thay đổi kỹ thuật:
- Dùng customtkinter như bản gốc.
- Giữ layout/tab/chức năng.
- Thay các icon emoji trong button/label bằng ký hiệu chữ ASCII để tránh Tk/customtkinter bị Segmentation fault trên Ubuntu.
- Dùng opencv-python-headless để tránh xung đột Qt/X11.

Cài:
  ./install_ubuntu.sh

Chạy:
  ./run_ubuntu.sh

Nếu không mở được USB/Arduino/ESP32:
  sudo usermod -aG dialout $USER
Sau đó đăng xuất/đăng nhập lại.


V4 CAR:
- Da gop main_map.py vao thu muc phan mem.
- Them tab DIEU KHIEN XE de gui UDP PWM/STOP/PING va START/STOP main_map.py.
- main_map.py can ROS2/rclpy, camera va file /home/tu/ceiling_marker_config/homography.yaml neu muon dung map/RViz nhu file goc.


V6 DUAL CAMERA:
- Tab KHO HANG / QR dung camera rieng, mac dinh index 0.
- Tab CONTROL XE / UDP dung camera rieng, mac dinh index 1.
- Co the mo song song 2 camera khac nhau. Khong cho mo trung index khi camera kia dang chay.


V8 CALIB CAMERA ↔ SLAM MAP:
- Them khu vuc CALIB trong tab BAN DO SLAM.
- Click tren khung camera live CONTROL XE de lay diem pixel camera.
- Double-click hoac chuot phai tren ban do SLAM de lay diem map tuong ung.
- Can toi thieu 4 cap diem, bam TINH HOMOGRAPHY.
- Co nut LOAD H / SAVE H de nap/luu homography.yaml.
- Khi Homography da co, robot va cac tram ArUco nhin thay tren camera CONTROL XE se tu dong hien thi dung vi tri tren map SLAM.


V12: Click camera tracking da duoc can theo anh hien thi; chuot phai gan cham de xoa diem CAM/MAP rieng, hoac nhap so P de xoa CAM/MAP/CAP.


V14:
- Sua trang thai nut CONNECT/DISCONNECT camera control.
- Camera control OFF thi khoa DISCONNECT, chi cho CONNECT.
- Them diem calib tren camera tracking bang double-click, giong map SLAM.
- Chuot phai gan cham camera/map van xoa rieng diem do.
