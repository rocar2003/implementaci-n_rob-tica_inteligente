import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
from yolo_msg.msg import InferenceResult, Yolov8Inference

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.model = YOLO('/home/robertog/ros2_ws/src/yolov8_ros2/yolov8_ros2/best (50-2).pt')
        self.yolov8_inference = Yolov8Inference()
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        #self.img_pub = self.create_publisher(Image, '/debug_yolo', 10)
        self.img = None
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        #self.detect_pub = self.create_publisher(String, "/signals", 10)
        self.turn_pub = self.create_publisher(String, "/turn_instruction", 10)
        self.semaphore_pub = self.create_publisher(String, "/semaphore_state", 10)
        self.signs_pub = self.create_publisher(String, "/signs_on_road", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.img is not None:
            results = self.model(self.img)
            self.yolov8_inference.header.frame_id = "inference"
            self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

            semaphore_state = "none"
            detected_signals = []

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                    c = box.cls
                    class_name = self.model.names[int(c)]
                    detected_signals.append(class_name)

                    if class_name == "Red":
                        semaphore_state = "Red"
                    elif class_name == "Green":
                        semaphore_state = "Green"
                    elif class_name == "Yellow":
                        semaphore_state = "Yellow"

                    #self.detect_pub.publish(String(data=class_name))
                    inference_result = InferenceResult()
                    inference_result.class_name = self.model.names[int(c)]
                    inference_result.top = int(b[0])
                    inference_result.left = int(b[1])
                    inference_result.bottom = int(b[2])
                    inference_result.right = int(b[3])
                    self.yolov8_inference.yolov8_inference.append(inference_result)

            self.semaphore_pub.publish(String(data=semaphore_state))

            if "Giveway" in detected_signals:
                self.signs_pub.publish(String(data="Giveway"))
            elif "Stop" in detected_signals:
                self.signs_pub.publish(String(data="Stop"))
            elif "work_in_progress" in detected_signals:
                self.signs_pub.publish(String(data="work_in_progress"))
            else:
                self.signs_pub.publish(String(data="none"))


            if "right" in detected_signals:
                self.turn_pub.publish(String(data="right"))
            elif "Left" in detected_signals:
                self.turn_pub.publish(String(data="Left"))
            elif "Frente" in detected_signals:
                self.turn_pub.publish(String(data="Frente"))
            else:
                self.turn_pub.publish(String(data="none"))

            # Ensure the image is in uint8 format
            annotated_frame = results[0].plot()
            if annotated_frame.dtype != np.uint8:
                annotated_frame = (annotated_frame * 255).astype(np.uint8)

            try:
                img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
               #self.img_pub.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Error converting annotated frame to Image message: {e}")

            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = img
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert from '{data.encoding}' to 'bgr8': {e}")

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()