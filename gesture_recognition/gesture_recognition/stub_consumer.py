import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import json

# Node
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
        self.get_logger().info("Class = "+results["class"]+" and probs "+str(results["probabilities"]))

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Stub_Consumer())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass

if __name__ == "__main__":
    main()