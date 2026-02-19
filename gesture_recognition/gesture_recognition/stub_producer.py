import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import json
from PIL import Image
import numpy as np

# Deployemt
IMAGE =  "/app/gesture_recognition/gesture_recognition/high_Evacuate-the-area_355_color.png"

# Node
class Stub_RGB_Stream(Node):
    
    def __init__(self, path_img:str):
        super().__init__('stub_rgb_stream')
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ("path",path_img),
        #     ]
        # )
        self.publisher = self.create_publisher(
            msg_type = String,
            topic = "rgb_stream",
            qos_profile = 10
        )
        self.timer = self.create_timer(1, self.generate_stream)
        # self.data = np.asarray(Image.open(self.get_parameter("path").get_parameter_value().string_value))
        self.data = np.asarray(Image.open(path_img))
        self.data = self.data.reshape((self.data.shape[2],self.data.shape[0],self.data.shape[1])) # as required by the classifier

    def generate_stream(self):
        data = json.dumps(self.data.tolist())
        self.get_logger().info("generating "+str(self.data.shape))
        self.publisher.publish(String(data=data))


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Stub_RGB_Stream(path_img=IMAGE))
    except (ExternalShutdownException, KeyboardInterrupt):
        pass

if __name__ == "__main__":
    main()