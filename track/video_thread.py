import cv2, os
import time
import serial
import sys
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QReadWriteLock, QWriteLocker
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from gimbal_control import control_gimbal, send_home_command


# ============================== 视频处理线程 ==============================
class VideoThread(QThread):
    change_pixmap = pyqtSignal(QImage)
    update_info = pyqtSignal(str)
    tracking_status = pyqtSignal(bool)
    target_selected = pyqtSignal(bool)

    def __init__(self, model_path, rtsp_url, serial_port):
        super().__init__()
        if getattr(sys, 'frozen', False):
            model_path = os.path.join(sys._MEIPASS, "yolov8n-face.pt")
        else:
            model_path = "yolov8n-face.pt"
        self.tracks = None
        self.ser = serial.Serial(serial_port, 115200, timeout=2)
        self.serial_lock = QReadWriteLock()
        self.model = YOLO(model_path).to('cuda')
        self.tracker = DeepSort(max_age=30)
        self.cap = cv2.VideoCapture(rtsp_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_ANY)
        self.selected_id = None
        self.running = True
        self.current_yaw = 0
        self.current_pitch = 0
        self.step_size = 5
        self.tracking_enabled = False
        self.target_center = None
        self.centered_threshold = 30  # 阈值设为30像素
        self.auto_track_timer = QTimer()
        self.auto_track_timer.timeout.connect(self.auto_track)

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
        time.sleep(1)

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
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"Tracking ID: {track_id}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
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
        with QWriteLocker(self.serial_lock):
            control_gimbal(self.ser, self.current_yaw, self.current_pitch)

    def return_home(self):
        self.current_yaw = 0
        self.current_pitch = 0
        send_home_command(self.ser)

    def auto_track(self):
        # if not self.tracking_enabled or not self.target_center:
        #     self.stop_auto_track()
        #     return
        if not self.tracking_enabled:
            return

        W, H = 1920, 1080
        frame_center = (W // 2, H // 2)

        if self.target_center:
            delta_x = self.target_center[0] - frame_center[0]
            delta_y = self.target_center[1] - frame_center[1]
        else:
            send_home_command(self.ser)

        # 判断是否达到阈值
        # if abs(delta_x) < self.centered_threshold and abs(delta_y) < self.centered_threshold:
        #     self.stop_auto_track()
        #     return

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

    def start_auto_track(self):
        if self.selected_id is None:
            return
        self.tracking_enabled = True
        self.tracking_status.emit(True)
        self.auto_track()
        self.auto_track_timer.start(2000)  # 启动2秒定时器

    def stop_auto_track(self):
        self.auto_track_timer.stop()
        self.tracking_enabled = False
        self.target_center = None
        self.selected_id = None
        self.tracking_status.emit(False)
        self.target_selected.emit(False)

    def stop(self):
        self.running = False
        self.cap.release()
        self.ser.close()
