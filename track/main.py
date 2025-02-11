import sys
from PyQt5.QtWidgets import QApplication
from main_window import MainWindow

if __name__ == "__main__":
    RTSP_URL = "rtsp://192.168.144.25:8554/main.264"
    SERIAL_PORT = "COM15"
    MODEL_PATH = "yolov8n-face.pt"

    app = QApplication(sys.argv)
    window = MainWindow(MODEL_PATH, RTSP_URL, SERIAL_PORT)
    window.show()
    sys.exit(app.exec_())
