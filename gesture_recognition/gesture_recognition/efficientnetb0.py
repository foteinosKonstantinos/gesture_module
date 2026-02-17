# from utils import *

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import json
from std_msgs.msg import String
import numpy as np

# [1,3,480,640]

class EfficientNetB0_Node(Node):

    def __init__(self):
        super().__init__("efficientnetb0_node")
        self.create_subscription(
            msg_type=String,
            topic="rgb_stream",
            callback=self.predict,
            qos_profile=10
        )
        self.publisher=self.create_publisher(
            msg_type = String,
            topic = "gesture_command",
            qos_profile = 10
        )

    def transform(self,data:String):
        return np.asarray(json.loads(data.data))

    def infer(self, data):
        return {"class":"1234","probabilities":{"1234":89.9120921}}

    def predict(self,in_data:String):
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
