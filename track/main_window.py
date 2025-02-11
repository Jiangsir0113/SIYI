import cv2
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QMainWindow, QWidget, QLabel, QPushButton,
                             QVBoxLayout, QHBoxLayout, QGridLayout,
                             QSizePolicy, QFrame)
from video_thread import VideoThread


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
