#!/usr/bin/env python3
import cv2
import numpy as np
import math
import socket
import time
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


# ===================== UDP WIFI DIRECT TO ESP32 =====================
UDP_IP = "192.168.1.250"
UDP_PORT = 8080

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

last_udp_reply = "NO REPLY"


def send_udp(msg):
    sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))


def send_pwm(left, right):
    left = int(np.clip(left, -255, 255))
    right = int(np.clip(right, -255, 255))
    msg = f"{left},{right}"
    send_udp(msg)


def stop_robot():
    send_pwm(0, 0)


def read_udp_reply():
    global last_udp_reply
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            last_udp_reply = data.decode(errors="ignore").strip()
    except BlockingIOError:
        pass


# ===================== ARUCO =====================
aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

params = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(
    aruco_dict,
    params
)


# ===================== CAMERA =====================
cap = cv2.VideoCapture(0)

# Nếu webcam hay lag/rớt, bật 3 dòng dưới:
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cap.set(cv2.CAP_PROP_FPS, 30)


# ===================== ROS MAP DISPLAY =====================
MAP_FRAME = "map"
BASE_FRAME = "base_link"

HOMOGRAPHY_FILE = "/home/tu/ceiling_marker_config/homography.yaml"

# Nếu robot trên RViz đúng vị trí nhưng ngược đầu, thử đổi thành math.pi
BASE_OFFSET_YAW = 0.0

# Nếu tâm robot không đúng giữa 2 marker, chỉnh thêm sau. Đơn vị mét.
BASE_OFFSET_X = 0.0
BASE_OFFSET_Y = 0.0

with open(os.path.expanduser(HOMOGRAPHY_FILE), "r") as f:
    homography_data = yaml.safe_load(f)

H = np.array(homography_data["homography"], dtype=np.float32)

rclpy.init(args=None)
ros_node = Node("marker_map_controller")

tf_broadcaster = TransformBroadcaster(ros_node)
pose_pub = ros_node.create_publisher(PoseStamped, "/marker_robot_pose", 10)
marker_pub = ros_node.create_publisher(MarkerArray, "/rviz_markers", 10)
path_pub = ros_node.create_publisher(Path, "/robot_path", 10)

robot_path = Path()
robot_path.header.frame_id = MAP_FRAME

print("ROS map display started")
print(f"  TF: {MAP_FRAME} -> {BASE_FRAME}")
print("  Pose: /marker_robot_pose")
print("  Markers: /rviz_markers")
print("  Path: /robot_path")
print(f"  Homography: {HOMOGRAPHY_FILE}")


# ===================== IDS =====================
FRONT_ID = 1
REAR_ID = 0

TARGET_MAP = {
    "A": 2,
    "B": 3,
}

TARGET_ID = None
current_order = None


# ===================== STATE =====================
IDLE = 0
NAVIGATING = 1
ARRIVE = 2

state = IDLE


# ===================== PID =====================
Kp = 0.75
Ki = 0.0
Kd = 0.10

prev_err = 0.0
integral = 0.0

# Đã test ổn định
TURN_SIGN = 1


# ===================== SPEED =====================
FORWARD_SPEED = 70
MAX_FORWARD_SPEED = 90

TURN_MIN_PWM = 70
TURN_MAX_PWM = 70

MIN_FORWARD_PWM = 55
MAX_CORRECTION = 25

TURN_THRESHOLD = 8
ANGLE_DEADBAND = 3

ARRIVE_DISTANCE = 45
SLOW_DISTANCE = 180

ARRIVE_HOLD_FRAMES = 5
arrive_counter = 0


# ===================== TIME =====================
last_time = time.time()
last_print_time = 0
last_stop_time = 0
last_map_print_time = 0


# ===================== KALMAN =====================
class Kalman2D:
    def __init__(self, q=0.02, r=3):
        self.x = np.zeros(2)
        self.P = np.eye(2)
        self.Q = q * np.eye(2)
        self.R = r * np.eye(2)
        self.initialized = False

    def update(self, z):
        z = np.array(z, dtype=float)

        if not self.initialized:
            self.x = z.copy()
            self.initialized = True
            return self.x

        self.P = self.P + self.Q
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.x = self.x + K @ (z - self.x)
        self.P = (np.eye(2) - K) @ self.P

        return self.x


kf_front = Kalman2D()
kf_rear = Kalman2D()
kf_target = Kalman2D()


def reset_filters():
    global kf_front, kf_rear, kf_target
    kf_front = Kalman2D()
    kf_rear = Kalman2D()
    kf_target = Kalman2D()


def reset_pid():
    global prev_err, integral, last_time
    prev_err = 0.0
    integral = 0.0
    last_time = time.time()


# ===================== BASIC UTILS =====================
def center(corner):
    c = corner.reshape((4, 2))

    return np.array([
        (c[0][0] + c[2][0]) / 2,
        (c[0][1] + c[2][1]) / 2
    ])


def robot_center(front, rear):
    return (front + rear) / 2


def heading(front, rear):
    v = front - rear
    return math.degrees(math.atan2(v[0], v[1]))


def angle(v):
    return math.degrees(math.atan2(v[0], v[1]))


def heading_error(robot_angle, target_angle):
    return (target_angle - robot_angle + 180) % 360 - 180


def dist(a, b):
    return np.linalg.norm(a - b)


def wrap_angle_rad(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


def pixel_to_map(pt):
    arr = np.array([[[float(pt[0]), float(pt[1])]]], dtype=np.float32)
    out = cv2.perspectiveTransform(arr, H)
    return np.array([out[0, 0, 0], out[0, 0, 1]], dtype=float)


def draw_text(frame, text, pos, color=(0, 255, 255), scale=0.7, thick=2):
    cv2.putText(
        frame,
        text,
        pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thick
    )


def select_target(order):
    global TARGET_ID, current_order, state, arrive_counter

    TARGET_ID = TARGET_MAP[order]
    current_order = order
    state = NAVIGATING
    arrive_counter = 0

    reset_pid()
    reset_filters()

    print(f"GO TO {order} | TARGET ID = {TARGET_ID}")


def send_stop_periodic(now):
    global last_stop_time

    if now - last_stop_time > 0.2:
        stop_robot()
        last_stop_time = now


# ===================== RVIZ MARKERS =====================
def make_marker(ns, marker_id, marker_type):
    m = Marker()
    m.header.frame_id = MAP_FRAME
    m.header.stamp = ros_node.get_clock().now().to_msg()
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.lifetime.sec = 1
    return m


def add_text_marker(arr, marker_id, text, x, y, z, r, g, b, scale=0.16):
    m = make_marker("text", marker_id, Marker.TEXT_VIEW_FACING)
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = float(z)
    m.scale.z = float(scale)
    m.color.r = float(r)
    m.color.g = float(g)
    m.color.b = float(b)
    m.color.a = 1.0
    m.text = text
    arr.markers.append(m)


def add_target_marker(arr, name, marker_id, x, y, active=False):
    color = {
        "A": (0.0, 0.55, 1.0),
        "B": (0.85, 0.1, 1.0),
        "C": (0.0, 1.0, 0.35),
    }.get(name, (1.0, 1.0, 0.0))

    r, g, b = color

    cube = make_marker("targets", 100 + marker_id, Marker.CUBE)
    cube.pose.position.x = float(x)
    cube.pose.position.y = float(y)
    cube.pose.position.z = 0.05
    cube.pose.orientation.w = 1.0

    if active:
        cube.scale.x = 0.22
        cube.scale.y = 0.22
        cube.scale.z = 0.12
    else:
        cube.scale.x = 0.16
        cube.scale.y = 0.16
        cube.scale.z = 0.08

    cube.color.r = r
    cube.color.g = g
    cube.color.b = b
    cube.color.a = 0.9
    arr.markers.append(cube)

    add_text_marker(
        arr,
        200 + marker_id,
        f"STATION {name} | ID {TARGET_MAP[name]}",
        x,
        y,
        0.28,
        1.0,
        1.0,
        1.0,
        scale=0.16,
    )

    # arrival circle
    circle = make_marker("arrival_circle", 300 + marker_id, Marker.LINE_STRIP)
    circle.scale.x = 0.025
    circle.color.r = r
    circle.color.g = g
    circle.color.b = b
    circle.color.a = 0.8

    radius = 0.15
    for i in range(73):
        a = 2.0 * math.pi * i / 72.0
        p = Point()
        p.x = float(x + radius * math.cos(a))
        p.y = float(y + radius * math.sin(a))
        p.z = 0.03
        circle.points.append(p)

    arr.markers.append(circle)


def add_line_marker(arr, marker_id, x1, y1, x2, y2):
    line = make_marker("goal_line", marker_id, Marker.LINE_STRIP)
    line.scale.x = 0.025
    line.color.r = 0.0
    line.color.g = 1.0
    line.color.b = 1.0
    line.color.a = 0.8

    p1 = Point()
    p1.x = float(x1)
    p1.y = float(y1)
    p1.z = 0.05

    p2 = Point()
    p2.x = float(x2)
    p2.y = float(y2)
    p2.z = 0.05

    line.points.append(p1)
    line.points.append(p2)
    arr.markers.append(line)


def publish_robot_pose_and_markers(data):
    global last_map_print_time

    arr = MarkerArray()
    robot_map_pose = None

    target_map_positions = {}

    # Publish target A/B nếu camera thấy marker A/B
    for name, tid in TARGET_MAP.items():
        if tid in data:
            t_px = center(data[tid])
            t_map = pixel_to_map(t_px)
            target_map_positions[name] = t_map
            add_target_marker(
                arr,
                name,
                tid,
                t_map[0],
                t_map[1],
                active=(current_order == name),
            )

    # Publish robot pose nếu thấy đủ 2 marker robot
    if FRONT_ID in data and REAR_ID in data:
        front_px = center(data[FRONT_ID])
        rear_px = center(data[REAR_ID])

        front_map = pixel_to_map(front_px)
        rear_map = pixel_to_map(rear_px)

        marker_center = (front_map + rear_map) / 2.0

        dx = front_map[0] - rear_map[0]
        dy = front_map[1] - rear_map[1]

        yaw = math.atan2(dy, dx)
        yaw = wrap_angle_rad(yaw + BASE_OFFSET_YAW)

        c = math.cos(yaw)
        s = math.sin(yaw)

        base_x = marker_center[0] + c * BASE_OFFSET_X - s * BASE_OFFSET_Y
        base_y = marker_center[1] + s * BASE_OFFSET_X + c * BASE_OFFSET_Y

        stamp = ros_node.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = MAP_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = float(base_x)
        tf.transform.translation.y = float(base_y)
        tf.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        tf_broadcaster.sendTransform(tf)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = MAP_FRAME
        pose.pose.position.x = float(base_x)
        pose.pose.position.y = float(base_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        pose_pub.publish(pose)

        robot_path.header.stamp = stamp
        robot_path.header.frame_id = MAP_FRAME
        robot_path.poses.append(pose)

        if len(robot_path.poses) > 800:
            robot_path.poses.pop(0)

        path_pub.publish(robot_path)

        add_text_marker(
            arr,
            10,
            f"ROBOT x={base_x:.2f} y={base_y:.2f} yaw={math.degrees(yaw):.1f}",
            base_x,
            base_y,
            0.35,
            0.1,
            1.0,
            0.1,
            scale=0.11,
        )

        robot_map_pose = (base_x, base_y, yaw)

        if current_order in target_map_positions:
            tgt = target_map_positions[current_order]
            add_line_marker(arr, 500, base_x, base_y, tgt[0], tgt[1])

        now = time.time()
        if now - last_map_print_time > 1.0:
            print(f"MAP_POSE x={base_x:.3f} y={base_y:.3f} yaw={math.degrees(yaw):.1f} deg")
            last_map_print_time = now

    marker_pub.publish(arr)
    rclpy.spin_once(ros_node, timeout_sec=0.0)

    return robot_map_pose, target_map_positions


# ===================== TEST WIFI ESP32 =====================
try:
    send_udp("ping")
    print("Sent ping to ESP32")
except Exception as e:
    print("UDP ping error:", e)


# ===================== MAIN LOOP =====================
try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Cannot read camera")
            stop_robot()
            break

        now = time.time()
        read_udp_reply()

        dt = now - last_time
        last_time = now
        dt = np.clip(dt, 0.001, 0.1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        key = cv2.waitKey(1) & 0xFF

        # ===================== KEY COMMAND =====================
        if key == ord('a'):
            select_target("A")

        elif key == ord('b'):
            select_target("B")

        elif key == ord('s'):
            state = IDLE
            current_order = None
            TARGET_ID = None
            arrive_counter = 0
            reset_pid()
            stop_robot()
            print("STOP / IDLE")

        elif key == ord('q'):
            send_pwm(-105, 105)
            print("MANUAL SPIN 1")

        elif key == ord('e'):
            send_pwm(105, -105)
            print("MANUAL SPIN 2")

        elif key == 27:
            stop_robot()
            break

        # ===================== NO MARKER =====================
        if ids is None:
            send_stop_periodic(now)

            draw_text(frame, "NO MARKER", (20, 40), color=(0, 0, 255), scale=0.8)
            draw_text(frame, "PRESS A OR B | S STOP | Q/E SPIN TEST | ESC EXIT", (20, 80), color=(255, 255, 255), scale=0.6)
            draw_text(frame, f"ESP32: {last_udp_reply}", (20, 115), color=(255, 255, 0), scale=0.55)

            empty_arr = MarkerArray()
            marker_pub.publish(empty_arr)
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            cv2.imshow("tracking_map", frame)
            continue

        ids = ids.flatten()

        data = {
            int(i): c
            for c, i in zip(corners, ids)
        }

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        map_pose, target_map_positions = publish_robot_pose_and_markers(data)

        # ===================== CHECK ROBOT =====================
        if FRONT_ID not in data or REAR_ID not in data:
            send_stop_periodic(now)

            draw_text(frame, "ROBOT MARKER LOST", (20, 40), color=(0, 0, 255), scale=0.8)
            draw_text(frame, f"NEED FRONT ID {FRONT_ID} AND REAR ID {REAR_ID}", (20, 80), color=(255, 255, 255), scale=0.6)
            draw_text(frame, f"ESP32: {last_udp_reply}", (20, 115), color=(255, 255, 0), scale=0.55)

            cv2.imshow("tracking_map", frame)
            continue

        # ===================== IDLE =====================
        if state == IDLE:
            send_stop_periodic(now)

            draw_text(frame, "STATE: IDLE", (20, 40), color=(0, 255, 255), scale=0.8)
            draw_text(frame, "PRESS A TO GO STATION A", (20, 80), color=(255, 255, 255), scale=0.6)
            draw_text(frame, "PRESS B TO GO STATION B", (20, 110), color=(255, 255, 255), scale=0.6)
            draw_text(frame, "Q/E = MANUAL SPIN TEST", (20, 140), color=(255, 255, 255), scale=0.6)
            draw_text(frame, f"ESP32: {last_udp_reply}", (20, 175), color=(255, 255, 0), scale=0.55)

            if map_pose is not None:
                mx, my, myaw = map_pose
                draw_text(frame, f"MAP x={mx:.2f} y={my:.2f} yaw={math.degrees(myaw):.1f}",
                          (20, 205), color=(0, 255, 0), scale=0.55)

            cv2.imshow("tracking_map", frame)
            continue

        # ===================== CHECK TARGET =====================
        if TARGET_ID not in data:
            send_stop_periodic(now)

            draw_text(frame, "TARGET NOT FOUND", (20, 40), color=(0, 0, 255), scale=0.8)
            draw_text(frame, f"WAITING TARGET ID {TARGET_ID}", (20, 80), color=(255, 255, 255), scale=0.6)
            draw_text(frame, f"ESP32: {last_udp_reply}", (20, 115), color=(255, 255, 0), scale=0.55)

            cv2.imshow("tracking_map", frame)
            continue

        # ===================== FILTER =====================
        front = kf_front.update(center(data[FRONT_ID]))
        rear = kf_rear.update(center(data[REAR_ID]))
        target = kf_target.update(center(data[TARGET_ID]))

        # ===================== ROBOT INFO =====================
        robot_c = robot_center(front, rear)
        robot_h = heading(front, rear)

        target_vec = target - robot_c
        target_ang = angle(target_vec)
        err = heading_error(robot_h, target_ang)
        d = dist(robot_c, target)

        # ===================== ARRIVE CHECK =====================
        if d < ARRIVE_DISTANCE:
            arrive_counter += 1
        else:
            arrive_counter = 0

        if arrive_counter >= ARRIVE_HOLD_FRAMES:
            state = ARRIVE
            stop_robot()

        # ===================== CONTROL =====================
        left = 0
        right = 0

        if state == ARRIVE:
            send_stop_periodic(now)

            draw_text(frame, "ARRIVED", (20, 40), color=(0, 255, 0), scale=1.0, thick=3)
            draw_text(frame, f"AT STATION {current_order}", (20, 80), color=(255, 255, 255), scale=0.8)

        else:
            if abs(err) < ANGLE_DEADBAND:
                err_control = 0.0
            else:
                err_control = err

            integral += err_control * dt
            integral = np.clip(integral, -80, 80)

            derivative = (err_control - prev_err) / dt
            derivative = np.clip(derivative, -200, 200)

            prev_err = err_control

            pid = (
                Kp * err_control +
                Ki * integral +
                Kd * derivative
            )

            # ===================== TURN MODE =====================
            if abs(err) > TURN_THRESHOLD:
                turn_pwm = np.interp(
                    abs(err),
                    [TURN_THRESHOLD, 45],
                    [TURN_MIN_PWM, TURN_MAX_PWM]
                )

                turn_pwm = np.clip(
                    turn_pwm,
                    TURN_MIN_PWM,
                    TURN_MAX_PWM
                )

                if err > 0:
                    left = -TURN_SIGN * turn_pwm
                    right = TURN_SIGN * turn_pwm
                else:
                    left = TURN_SIGN * turn_pwm
                    right = -TURN_SIGN * turn_pwm

            # ===================== FORWARD MODE =====================
            else:
                if d < SLOW_DISTANCE:
                    forward = np.interp(
                        d,
                        [ARRIVE_DISTANCE, SLOW_DISTANCE],
                        [MIN_FORWARD_PWM, FORWARD_SPEED]
                    )
                else:
                    forward = FORWARD_SPEED

                correction = np.clip(
                    pid * 0.6,
                    -MAX_CORRECTION,
                    MAX_CORRECTION
                )

                left = forward - correction
                right = forward + correction

            left = int(np.clip(left, -255, 255))
            right = int(np.clip(right, -255, 255))

            send_pwm(left, right)

            if now - last_print_time > 0.2:
                print(
                    f"ORDER={current_order} | "
                    f"L={left} "
                    f"R={right} "
                    f"ERR={err:.1f} "
                    f"D={d:.1f} | "
                    f"ESP32={last_udp_reply}"
                )

                last_print_time = now

        # ===================== VISUAL =====================
        robot_i = tuple(robot_c.astype(int))
        target_i = tuple(target.astype(int))

        robot_vec = front - rear
        robot_end = robot_c + robot_vec * 0.8

        cv2.arrowedLine(
            frame,
            robot_i,
            tuple(robot_end.astype(int)),
            (0, 255, 0),
            3,
            tipLength=0.3
        )

        cv2.arrowedLine(
            frame,
            robot_i,
            target_i,
            (255, 0, 0),
            2,
            tipLength=0.2
        )

        cv2.circle(frame, robot_i, 6, (0, 255, 0), -1)
        cv2.circle(frame, target_i, 6, (0, 0, 255), -1)

        state_name = {
            IDLE: "IDLE",
            NAVIGATING: "NAVIGATING",
            ARRIVE: "ARRIVE"
        }.get(state, "UNKNOWN")

        panel = [
            f"STATE: {state_name}",
            f"ORDER: {current_order}",
            f"TARGET ID: {TARGET_ID}",
            f"ROBOT X: {robot_c[0]:.1f}",
            f"ROBOT Y: {robot_c[1]:.1f}",
            f"HEADING: {robot_h:.1f}",
            f"TARGET ANGLE: {target_ang:.1f}",
            f"ERROR: {err:.2f}",
            f"DISTANCE: {d:.1f}",
            f"PWM L/R: {left}, {right}",
            f"TURN_SIGN: {TURN_SIGN}",
            f"ESP32: {last_udp_reply}",
            "KEYS: A | B | S STOP | Q/E TEST | ESC EXIT"
        ]

        if map_pose is not None:
            mx, my, myaw = map_pose
            panel.insert(
                10,
                f"MAP: x={mx:.2f} y={my:.2f} yaw={math.degrees(myaw):.1f}"
            )

        y = 120

        for t in panel:
            draw_text(
                frame,
                t,
                (10, y),
                color=(0, 255, 255),
                scale=0.55,
                thick=2
            )

            y += 25

        cv2.imshow("tracking_map", frame)

finally:
    print("Stopping robot...")

    try:
        stop_robot()
        time.sleep(0.1)
        stop_robot()
    except Exception:
        pass

    try:
        ros_node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass

    cap.release()
    sock.close()
    cv2.destroyAllWindows()
