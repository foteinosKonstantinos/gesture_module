#   PUBLISHER, WORKING AS A STUB
#   ros2 run my_package node_executable --ros-args ...
#   https://answers.ros.org/question/403914/

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import json
from std_msgs.msg import String

class Stub_Consumer(Node):
    
    def __init__(self):
        super().__init__('stub_consumer')
        self.create_subscription(
            msg_type=String,
            topic="gesture_command",
            callback=self.show,
            qos_profile=10
        )

    def show(self, data:String):
        results = json.loads(data.data)
        self.get_logger().info("Class = "+results["class"]+" and probs "+results["probabilities"])

def main() -> None:
    try:
        rclpy.init()
        consumer = Stub_Consumer()
        rclpy.spin(node=consumer)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()