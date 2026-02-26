import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import json
import numpy as np
import onnxruntime as ort
from ultralytics import YOLO

# Deployment details
ONNX_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnet.onnx"
POSE_ESTIMATOR = "yolo26n-pose.pt"

# Gesture commands. The ordering is crucial.
CLASSES = [
    "fetch-a-gas-mask",
    "come-to-me",
    "ok-to-go",
    "move-away-from-here",
    "operation-finished",
    "freeze",
    "emergency-situation",
    "i-need-help",
    "evacuate-the-area",
    "i-lost-connection",
    "fetch-a-shovel",
    "fetch-an-axe"
]

class Classification_Node(Node):

    def __init__(self, onnx_path:str, classes:list[str]=CLASSES, pose_estimator:str=POSE_ESTIMATOR, fg0:float=80.09993757800315, d0:float=4612.0):
        super().__init__("classification_node")
        self.__onnx_path = onnx_path
        self.__classes = classes
        self.create_subscription(
            msg_type=Image,
            topic="/camera_front/raw_image",
            callback=self.prediction_callback,
            qos_profile=10
        )
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = "gesture_command",
            qos_profile = 10
        )
        self.__session = ort.InferenceSession(onnx_path)
        self.get_logger().info(f"Successfully initialized the classification node, with weights {self.__onnx_path} for {len(self.__classes)} classes.")
        self.__pose_estimator = YOLO(pose_estimator)
        self.__fg0 = fg0
        self.__d0 = d0

    def __estimate_depth(self, image): # np.array H x W x 3
        result = self.__pose_estimator(image)
        if len(result.keypoints.data)==0:
            return None
        # WORKS ONLY FOR ONE PERSON!!
        self.get_logger().warning("Works for a single person!!!")
        ls = result.keypoints.data[0][5][[0,1]].numpy().astype(int) # left shoulder
        rs = result.keypoints.data[0][6][[0,1]].numpy().astype(int) # right shoulder
        fg = ((ls-rs)**2).sum().item() ** (1/2)
        d = (self.__fg0 * self.__d0) / fg
        return d.item()
    
    def __estimate_location(self, depth):
        pass

    def prediction_callback(self,in_data:Image): # 3 (=RGB channels) x 480 (=H) x 640 (=W)
        image = np.asarray(in_data.data, dtype=np.float32).reshape((in_data.height, in_data.width, 3))
        depth = self.__estimate_depth(image)
        if depth is None:
            return
        self.get_logger().info(f"recieved: {image.shape} detected person in distance: {depth}")
        probs = self.__session.run(None, {"input":cv2.resize(image, dsize=(640, 480)).reshape((1,3,480,640))})
        argmax_idx = np.asarray(probs[0]).argmax()
        pred_class = self.__classes[argmax_idx]
        confidence = probs[0][argmax_idx].item()
        self.__publisher.publish(String(data=json.dumps({
            "type": "FeatureCollection",
            "features":[
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [0, 0]
                    },
                    "properties": {
                        "class":pred_class,
                        "confidence":confidence,
                        "depth":depth
                    }
                }
            ]
        })))
        self.get_logger().info(f"published \'{pred_class}\' and confidence {confidence}")

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Classification_Node(onnx_path=ONNX_MODEL))
    except (ExternalShutdownException, KeyboardInterrupt):
        pass

if __name__ == '__main__':
    main()
