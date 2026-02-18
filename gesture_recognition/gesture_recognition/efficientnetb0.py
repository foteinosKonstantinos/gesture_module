# from utils import *

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import json
from std_msgs.msg import String
import numpy as np
import onnxruntime as ort

# [1,3,480,640]

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

def probs_to_result(probs):
    # probabilities = torch.nn.functional.softmax(logits)
    return {
        "class": CLASSES[np.asarray(probs[0]).argmax()],
        "probabilities": {
            CLASSES[i]:probs[0][i].item() for i in range(len(CLASSES))
        }
    }

class EfficientNetB0_Node(Node):

    def __init__(self):
        super().__init__("efficientnetb0_node")
        self.create_subscription(
            msg_type=String,
            topic="rgb_stream",
            callback=self.callback,
            qos_profile=10
        )
        self.publisher=self.create_publisher(
            msg_type = String,
            topic = "gesture_command",
            qos_profile = 10
        )
        self.session = ort.InferenceSession("/home/konstantinosf/Projects/gesture_module/gesture_recognition/gesture_recognition/efficientnet.onnx")

    def transform(self,data:String):
        return np.asarray(json.loads(data.data))

    def infer(self, data): # 480 640 three channels
        probs = self.session.run(None, {
            "input":np.asarray(data,dtype=np.float32).reshape((1,3,480,640))
        })
        return probs_to_result(probs)

    def callback(self,in_data:String):
        in_data = self.transform(in_data)
        self.get_logger().info("recieved "+str(in_data.shape))
        out_data = json.dumps(self.infer(in_data))
        self.publisher.publish(String(data=out_data))

def main() -> None:
    try:
        rclpy.init()
        efficientnent = EfficientNetB0_Node()
        rclpy.spin(node=efficientnent)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
