#   PUBLISHER, WORKING AS A STUB
#   ros2 run my_package node_executable --ros-args ...
#   https://answers.ros.org/question/403914/

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import json
from std_msgs.msg import String
from PIL import Image
import numpy as np

class Stub_RGB_Stream(Node):
    
    def __init__(self):
        super().__init__('stub_rgb_stream')
        self.declare_parameters(
            namespace='',
            parameters=[
                ("path", "/home/konstantinosf/Projects/gesture_module/gesture_recognition/gesture_recognition/high_Evacuate-the-area_355_color.png"),
            ]
        )
        self.publisher = self.create_publisher(
            msg_type = String,
            topic = "rgb_stream",
            qos_profile = 10
        )
        self.timer = self.create_timer(1, self.generate_stream)
        self.data = np.asarray(Image.open(self.get_parameter("path").get_parameter_value().string_value))
        self.data = self.data.reshape((self.data.shape[2],self.data.shape[0],self.data.shape[1]))

    def generate_stream(self):
        data = json.dumps(self.data.tolist())
        self.get_logger().info("generating "+str(self.data.shape))
        self.publisher.publish(String(data=data))


def main() -> None:
    try:
        rclpy.init()
        stub_rgb_stream = Stub_RGB_Stream()
        rclpy.spin(node=stub_rgb_stream)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()