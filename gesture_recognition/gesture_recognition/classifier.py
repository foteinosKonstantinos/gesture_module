import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import json
import numpy as np
import onnxruntime as ort

# Deployment details
ONNX_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnet.onnx"

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

    def __init__(self, onnx_path:str, classes:list[str]=CLASSES):
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

    def prediction_callback(self,in_data:Image): # 3 (=RGB channels) x 480 (=H) x 640 (=W)

        image = np.asarray(in_data.data, dtype=np.float32).reshape((in_data.height, in_data.width, 3))

        self.get_logger().info("recieved "+str(image.shape))

        probs = self.__session.run(None, {"input":cv2.resize(image, dsize=(640, 480)).reshape((1,3,480,640))})

        argmax_idx = np.asarray(probs[0]).argmax()
        pred_class = self.__classes[argmax_idx]

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
                        "confidence":probs[0][argmax_idx].item()
                    }
                }
            ]
        })))

        self.get_logger().info(f"published \'{pred_class}\' and confidence")

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Classification_Node(onnx_path=ONNX_MODEL))
    except (ExternalShutdownException, KeyboardInterrupt):
        pass

if __name__ == '__main__':
    main()
