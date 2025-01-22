from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import cv2


class YOLOv8DeepSORTTracker:
    def __init__(self, model_path, rtsp_url, max_age=30):
        """
        初始化 YOLOv8 + DeepSORT 跟踪器。

        Args:
            model_path (str): YOLOv8 模型路径。
            rtsp_url (str): RTSP 视频流地址。
            max_age (int): DeepSORT 跟踪器的 max_age 参数。
        """
        # 加载 YOLOv8 模型
        self.model = YOLO(model_path)
        self.model.to('cuda')  # 使用 GPU 加速

        # 初始化 DeepSORT 跟踪器
        self.tracker = DeepSort(max_age=max_age)

        # 打开 RTSP 视频流
        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            raise ValueError("无法打开 RTSP 视频流，请检查 URL 或网络连接。")

        # 全局变量：选中的目标 ID
        self.selected_track_id = None

        # 创建窗口并绑定鼠标事件
        cv2.namedWindow('YOLOv8 + DeepSORT Tracking')
        cv2.setMouseCallback('YOLOv8 + DeepSORT Tracking', self.select_target)

    def select_target(self, event, x, y, flags, param):
        """
        鼠标点击事件回调函数。

        Args:
            event: 鼠标事件类型。
            x, y: 鼠标点击坐标。
            flags: 事件标志。
            param: 额外参数。
        """
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键点击
            self.selected_track_id = None  # 先清空选中的目标
            for track in self.tracks:
                if not track.is_confirmed():
                    continue
                ltrb = track.to_ltrb()  # 获取目标的边界框
                x1, y1, x2, y2 = map(int, ltrb)
                if x1 <= x <= x2 and y1 <= y <= y2:  # 判断点击位置是否在目标框内
                    self.selected_track_id = track.track_id  # 记录选中的目标 ID
                    print(f"选中目标 ID: {self.selected_track_id}")
                    # 输出目标位置信息
                    print(f"目标位置信息： X1: {x1}, Y1: {y1}, X2: {x2}, Y2: {y2}")
                    break

    def process_frame(self, frame):
        """
        处理视频帧：目标检测 + 目标跟踪。

        Args:
            frame: 输入的视频帧。

        Returns:
            frame: 处理后的视频帧。
        """
        # 使用 YOLOv8 进行目标检测
        results = self.model(frame, verbose=False)  # verbose=False 关闭冗余输出

        # 提取检测结果
        detections = []
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # 检测框坐标 (x1, y1, x2, y2)
            confidences = result.boxes.conf.cpu().numpy()  # 置信度
            class_ids = result.boxes.cls.cpu().numpy()  # 类别 ID

            for box, confidence, class_id in zip(boxes, confidences, class_ids):
                detections.append((box, confidence, class_id))

        # 将检测结果转换为 DeepSORT 所需的格式
        deepsort_detections = []
        for box, confidence, class_id in detections:
            x1, y1, x2, y2 = box
            width = x2 - x1
            height = y2 - y1
            deepsort_detections.append(([x1, y1, width, height], confidence, class_id))

        # 使用 DeepSORT 进行目标追踪
        self.tracks = self.tracker.update_tracks(deepsort_detections, frame=frame)

        # 绘制追踪结果
        for track in self.tracks:
            if not track.is_confirmed():  # 只绘制已确认的目标
                continue
            track_id = track.track_id  # 目标 ID
            ltrb = track.to_ltrb()  # 获取边界框 (left, top, right, bottom)

            # 如果选中了目标，则只绘制选中的目标
            if self.selected_track_id is not None and track_id != self.selected_track_id:
                continue

            # 绘制边界框
            x1, y1, x2, y2 = map(int, ltrb)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绿色框

            # 在框上方显示目标 ID
            cv2.putText(frame, f"ID: {track_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame

    def run(self):
        """
        主循环：读取视频流并处理每一帧。
        """
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("无法读取视频帧，视频流可能已结束。")
                break

            # 处理当前帧
            processed_frame = self.process_frame(frame)

            # 显示结果
            cv2.imshow('YOLOv8 + DeepSORT Tracking', processed_frame)

            # 按下 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 释放资源
        self.cap.release()
        cv2.destroyAllWindows()


