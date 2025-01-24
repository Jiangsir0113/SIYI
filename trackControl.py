import sys
import struct
import time
import serial
import cv2
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QSizePolicy, QFrame)
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# ============================== 云台控制模块 ==============================
crc16_tab = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]


def crc16_cal(data):
    crc = 0
    for byte in data:
        temp = (crc >> 8) & 0xFF
        crc = ((crc << 8) & 0xFFFF) ^ crc16_tab[(byte ^ temp) & 0xFF]
    return crc


def send_gimbal_control(ser, yaw, pitch):
    cmd = bytearray([
        0x55, 0x66,
        0x01,
        0x04, 0x00,
        0x00, 0x00,
        0x0E
    ])
    cmd.extend(struct.pack('<h', int(yaw * 10)))
    cmd.extend(struct.pack('<h', int(pitch * 10)))
    crc = crc16_cal(cmd)
    cmd.extend(struct.pack('<H', crc))
    ser.write(cmd)


def control_gimbal(ser, yaw, pitch):
    if not (-135.0 <= yaw <= 135.0):
        raise ValueError("Yaw超出范围(-135.0~135.0)！")
    if not (-90.0 <= pitch <= 25.0):
        raise ValueError("Pitch超出范围(-90.0~25.0)！")
    send_gimbal_control(ser, yaw, pitch)


def send_home_command(ser):
    cmd = bytearray([0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x01])
    crc = crc16_cal(cmd)
    cmd.extend(struct.pack('<H', crc))
    ser.write(cmd)
    print("一键回中指令已发送")


# ============================== 视频处理线程 ==============================
class VideoThread(QThread):
    change_pixmap = pyqtSignal(QImage)
    update_info = pyqtSignal(str)
    tracking_status = pyqtSignal(bool)
    target_selected = pyqtSignal(bool)

    def __init__(self, model_path, rtsp_url, serial_port):
        super().__init__()
        self.ser = serial.Serial(serial_port, 115200, timeout=2)
        self.model = YOLO(model_path).to('cuda')
        self.tracker = DeepSort(max_age=30)
        self.cap = cv2.VideoCapture(rtsp_url)
        self.selected_id = None
        self.running = True
        self.current_yaw = 0
        self.current_pitch = 0
        self.step_size = 5
        self.tracking_enabled = False
        self.target_center = None
        self.centered_threshold = 30  # 阈值设为30像素

    def mouse_callback(self, x, y):
        self.selected_id = None
        self.target_center = None
        for track in self.tracks:
            if not track.is_confirmed():
                continue
            ltrb = track.to_ltrb()
            if ltrb[0] <= x <= ltrb[2] and ltrb[1] <= y <= ltrb[3]:
                self.selected_id = track.track_id
                self.target_center = ((ltrb[0] + ltrb[2]) // 2, (ltrb[1] + ltrb[3]) // 2)
                self.target_selected.emit(True)
                return
        self.target_selected.emit(False)

    def run(self):
        control_gimbal(self.ser, self.current_yaw, self.current_pitch)
        time.sleep(2)

        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            results = self.model(frame, verbose=False)[0]
            detections = [(box.xyxy[0].cpu().numpy(), box.conf.item(), box.cls.item())
                          for box in results.boxes]

            self.tracks = self.tracker.update_tracks(
                [([x1, y1, x2 - x1, y2 - y1], conf, cls_id)
                 for (x1, y1, x2, y2), conf, cls_id in detections],
                frame=frame
            )

            info_text = "当前跟踪目标信息：\n暂无选择"
            for track in self.tracks:
                if not track.is_confirmed():
                    continue
                track_id = track.track_id
                ltrb = track.to_ltrb()
                x1, y1, x2, y2 = map(int, ltrb)

                if self.selected_id is None:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"ID: {track_id}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if self.selected_id and track_id == self.selected_id:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"Tracking ID: {track_id}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    self.target_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                    info_text = f"跟踪目标ID: {track_id}\n坐标范围:\nX: {x1}-{x2}\nY: {y1}-{y2}"

            self.update_info.emit(info_text)

            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.change_pixmap.emit(qt_image)

    def move_gimbal(self, delta_yaw, delta_pitch):
        self.current_yaw = max(min(self.current_yaw + delta_yaw, 135), -135)
        self.current_pitch = max(min(self.current_pitch + delta_pitch, 25), -90)
        control_gimbal(self.ser, self.current_yaw, self.current_pitch)

    def return_home(self):
        self.current_yaw = 0
        self.current_pitch = 0
        send_home_command(self.ser)

    def auto_track(self):
        if not self.tracking_enabled or not self.target_center:
            self.stop_auto_track()
            return

        W, H = 1920, 1080
        frame_center = (W // 2, H // 2)
        delta_x = self.target_center[0] - frame_center[0]
        delta_y = self.target_center[1] - frame_center[1]

        # 判断是否达到阈值
        if abs(delta_x) < self.centered_threshold and abs(delta_y) < self.centered_threshold:
            self.stop_auto_track()
            return

        alpha_x = 81.0 / W
        alpha_y = 62.1 / H
        delta_yaw = -delta_x * alpha_x
        delta_pitch = -delta_y * alpha_y

        new_yaw = max(min(self.current_yaw + delta_yaw, 135), -135)
        new_pitch = max(min(self.current_pitch + delta_pitch, 25), -90)

        if (new_yaw != self.current_yaw) or (new_pitch != self.current_pitch):
            control_gimbal(self.ser, -new_yaw, -new_pitch)
            self.current_yaw = new_yaw
            self.current_pitch = new_pitch

        # 单次计算后停止追踪
        # self.stop_auto_track()

    def start_auto_track(self):
        if self.selected_id is None:
            return
        self.tracking_enabled = True
        self.auto_track()

    def stop_auto_track(self):
        self.tracking_enabled = False
        self.target_center = None
        self.selected_id = None
        self.tracking_status.emit(False)
        self.target_selected.emit(False)

    def stop(self):
        self.running = False
        self.cap.release()
        self.ser.close()


# ============================== 主界面 ==============================
class MainWindow(QMainWindow):
    def __init__(self, model_path, rtsp_url, serial_port):
        super().__init__()
        self.setWindowTitle("云台相机控制系统")
        self.setGeometry(100, 100, 1280, 720)

        # 初始化视频线程
        self.video_thread = VideoThread(model_path, rtsp_url, serial_port)
        self.video_thread.change_pixmap.connect(self.update_image)
        self.video_thread.update_info.connect(self.update_target_info)
        self.video_thread.tracking_status.connect(self.update_tracking_status)
        self.video_thread.target_selected.connect(self.update_selection_status)

        # 主界面布局
        main_widget = QWidget()
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(15)

        # 左侧视频区域
        video_frame = QFrame()
        video_frame.setStyleSheet("background-color: black; border-radius: 8px;")
        video_layout = QVBoxLayout(video_frame)
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.mousePressEvent = self.get_pixel_position
        video_layout.addWidget(self.video_label)
        main_layout.addWidget(video_frame, 3)

        # 右侧控制区域
        right_panel = QFrame()
        right_panel.setStyleSheet("background-color: #FFFFFF; border-radius: 8px;")
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(15, 15, 15, 15)
        right_layout.setSpacing(20)

        # 信息面板
        info_frame = QFrame()
        info_frame.setStyleSheet("background-color: #F5F5F5; border-radius: 8px; padding: 15px;")
        info_layout = QVBoxLayout(info_frame)
        self.info_label = QLabel("当前跟踪目标信息：\n暂无选择")
        self.info_label.setStyleSheet("font: 14px '微软雅黑'; color: #333; line-height: 1.5;")
        info_layout.addWidget(self.info_label)
        right_layout.addWidget(info_frame)

        # 自动追踪控制面板
        tracking_control_frame = QFrame()
        tracking_control_frame.setStyleSheet("background-color: #FAFAFA; border-radius: 8px;")
        tracking_layout = QHBoxLayout(tracking_control_frame)

        self.btn_start_track = QPushButton("开始追踪")
        self.btn_stop_track = QPushButton("停止追踪")
        for btn in [self.btn_start_track, self.btn_stop_track]:
            btn.setFixedHeight(45)
            btn.setStyleSheet("""
                QPushButton {
                    font: bold 16px '微软雅黑';
                    color: white;
                    background-color: #2196F3;
                    border-radius: 6px;
                    padding: 8px;
                }
                QPushButton:hover { background-color: #1976D2; }
                QPushButton:pressed { background-color: #0D47A1; }
                QPushButton:disabled { background-color: #BBDEFB; }
            """)
        tracking_layout.addWidget(self.btn_start_track)
        tracking_layout.addWidget(self.btn_stop_track)
        right_layout.addWidget(tracking_control_frame)

        # 手动控制面板
        control_frame = QFrame()
        control_frame.setStyleSheet("background-color: #FAFAFA; border-radius: 8px;")
        control_layout = QGridLayout(control_frame)
        control_layout.setContentsMargins(20, 20, 20, 20)
        control_layout.setSpacing(15)

        self.btn_home = self.create_control_button("回中", "home")
        self.btn_up = self.create_control_button("↑", "vertical")
        self.btn_down = self.create_control_button("↓", "vertical")
        self.btn_left = self.create_control_button("←", "horizontal")
        self.btn_right = self.create_control_button("→", "horizontal")

        control_layout.addWidget(self.btn_up, 0, 1)
        control_layout.addWidget(self.btn_left, 1, 0)
        control_layout.addWidget(self.btn_home, 1, 1)
        control_layout.addWidget(self.btn_right, 1, 2)
        control_layout.addWidget(self.btn_down, 2, 1)

        control_layout.setRowStretch(0, 1)
        control_layout.setRowStretch(2, 1)
        control_layout.setColumnStretch(0, 1)
        control_layout.setColumnStretch(2, 1)
        right_layout.addWidget(control_frame)

        # 退出按钮
        exit_btn = QPushButton("退出系统")
        exit_btn.setFixedHeight(45)
        exit_btn.setStyleSheet("""
            QPushButton {
                font: bold 16px '微软雅黑';
                color: white;
                background-color: #F44336;
                border-radius: 6px;
                padding: 8px;
            }
            QPushButton:hover { background-color: #E53935; }
            QPushButton:pressed { background-color: #D32F2F; }
        """)
        exit_btn.clicked.connect(self.close)
        right_layout.addWidget(exit_btn)

        main_layout.addWidget(right_panel, 1)
        self.setCentralWidget(main_widget)

        # 事件绑定
        self.btn_up.pressed.connect(lambda: self.start_move('up'))
        self.btn_down.pressed.connect(lambda: self.start_move('down'))
        self.btn_left.pressed.connect(lambda: self.start_move('left'))
        self.btn_right.pressed.connect(lambda: self.start_move('right'))
        self.btn_up.released.connect(self.stop_move)
        self.btn_down.released.connect(self.stop_move)
        self.btn_left.released.connect(self.stop_move)
        self.btn_right.released.connect(self.stop_move)
        self.btn_home.clicked.connect(self.video_thread.return_home)
        self.btn_start_track.clicked.connect(self.video_thread.start_auto_track)
        self.btn_stop_track.clicked.connect(self.video_thread.stop_auto_track)

        # 初始按钮状态
        self.btn_start_track.setEnabled(False)
        self.btn_stop_track.setEnabled(False)

        # 移动定时器
        self.move_timer = QTimer()
        self.move_timer.setInterval(100)
        self.current_direction = None

        self.video_thread.start()

    def create_control_button(self, text, btn_type):
        btn = QPushButton(text)
        btn.setFixedSize(80, 80)
        color_map = {
            "vertical": "#4CAF50",
            "horizontal": "#2196F3",
            "home": "#FF9800"
        }
        btn.setStyleSheet(f"""
            QPushButton {{
                font: bold 24px;
                color: white;
                background-color: {color_map[btn_type]};
                border-radius: 8px;
                border: 2px solid rgba(255,255,255,150);
            }}
            QPushButton:hover {{
                background-color: {color_map[btn_type]}DD;
            }}
            QPushButton:pressed {{
                background-color: {color_map[btn_type]}BB;
            }}
        """)
        return btn

    def start_move(self, direction):
        self.current_direction = direction
        self.move_timer.timeout.connect(self.handle_movement)
        self.move_timer.start()

    def stop_move(self):
        self.move_timer.stop()
        self.move_timer.timeout.disconnect()

    def handle_movement(self):
        step = self.video_thread.step_size
        if self.current_direction == 'up':
            self.video_thread.move_gimbal(0, step)
        elif self.current_direction == 'down':
            self.video_thread.move_gimbal(0, -step)
        elif self.current_direction == 'left':
            self.video_thread.move_gimbal(-step, 0)
        elif self.current_direction == 'right':
            self.video_thread.move_gimbal(step, 0)

    @pyqtSlot(QImage)
    def update_image(self, image):
        pixmap = QPixmap.fromImage(image)
        scaled_pixmap = pixmap.scaled(
            self.video_label.size(),
            Qt.KeepAspectRatioByExpanding,
            Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled_pixmap)

    @pyqtSlot(str)
    def update_target_info(self, info):
        self.info_label.setText(info)

    @pyqtSlot(bool)
    def update_tracking_status(self, is_tracking):
        self.btn_start_track.setEnabled(not is_tracking)
        self.btn_stop_track.setEnabled(is_tracking)

    @pyqtSlot(bool)
    def update_selection_status(self, has_selection):
        self.btn_start_track.setEnabled(has_selection and not self.btn_stop_track.isEnabled())

    def get_pixel_position(self, event):
        x = event.pos().x()
        y = event.pos().y()
        if self.video_label.pixmap():
            pixmap = self.video_label.pixmap()
            frame_width = self.video_thread.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            frame_height = self.video_thread.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

            img_width = pixmap.width()
            img_height = pixmap.height()
            label_width = self.video_label.width()
            label_height = self.video_label.height()

            dx = (img_width - label_width) // 2
            dy = (img_height - label_height) // 2

            adj_x = x + dx
            adj_y = y + dy

            frame_x = int(adj_x * frame_width / img_width)
            frame_y = int(adj_y * frame_height / img_height)
            self.video_thread.mouse_callback(frame_x, frame_y)

    def closeEvent(self, event):
        self.video_thread.stop()
        self.video_thread.wait()
        event.accept()


# ============================== 主程序 ==============================
if __name__ == "__main__":
    RTSP_URL = "rtsp://192.168.144.25:8554/main.264"
    SERIAL_PORT = "COM15"
    MODEL_PATH = "yolov8n-face.pt"

    app = QApplication(sys.argv)
    window = MainWindow(MODEL_PATH, RTSP_URL, SERIAL_PORT)
    window.show()
    sys.exit(app.exec_())
