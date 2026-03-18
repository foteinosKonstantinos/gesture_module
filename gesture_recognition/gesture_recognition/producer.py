import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as SensorImage, NavSatFix
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
import numpy as np
from rclpy.executors import ExternalShutdownException
import time

PATH = "/app"

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class Producer(Node):

    def __init__(self,path=PATH):
        super().__init__("producer_node")
        
        self.__color_publisher=self.create_publisher(
            msg_type = SensorImage,
            topic = "/camera_front/color",
            qos_profile = 10
        )
        self.__depth_publisher=self.create_publisher(
            msg_type = SensorImage,
            topic = "/camera_front/depth",
            qos_profile = 10
        )
        self.__info_publisher=self.create_publisher(
            msg_type=CameraInfo,
            topic="/camera_front/camera_info",
            qos_profile = 10
        )
        self.__gps_publisher=self.create_publisher(
            msg_type=NavSatFix,
            topic="/fix",
            qos_profile = 10
        )
        self.__odo_publisher=self.create_publisher(
            msg_type=Odometry,
            topic="/dog_odom",
            qos_profile = 10
        )

        depth_frames = ["low_Operation-finished_675_depth.png","low_Operation-finished_675_depth.png","low_Operation-finished_675_depth.png"]
        rgb_frames = ["low_Operation-finished_675_color.png","dummy.png","high_Evacuate-the-area_355_color.png"]

        assert len(depth_frames) == len(rgb_frames)

        c = 0.0
        idx = 0

        while True:

            time.sleep(1)

            
            depth_path = f"{path}/{depth_frames[idx]}"
            color_path = f"{path}/{rgb_frames[idx]}"
            self.get_logger().info(f"Exporting {depth_path} and {color_path} ...")
            idx = (idx + 1) % 3

            depth = np.asarray(PILImage.open(depth_path),dtype=np.uint16)
            color = np.asarray(PILImage.open(color_path).convert("RGB"))
            

            msg = SensorImage()
            msg.height = depth.shape[0]
            msg.width = depth.shape[1]
            msg.encoding = "16UC1"
            msg.is_bigendian = False
            msg.step = 2 * depth.shape[1]
            msg.data = depth.tobytes()
            self.__depth_publisher.publish(msg)

            msg = SensorImage()
            msg.height = color.shape[0]
            msg.width = color.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = 3 * color.shape[1]
            msg.data = color.tobytes()
            self.__color_publisher.publish(msg)

            msg = CameraInfo()
            msg.k = [500.0, 0.0, 640.0, 0.0, 500.0, 360.0, 0.0, 0.0, 1.0]
            self.__info_publisher.publish(msg)

            msg = NavSatFix()
            msg.latitude = 0.0
            msg.longitude = c # degrees
            c += 1e-5
            self.__gps_publisher.publish(msg)

            msg = Odometry()
            q = euler_to_quaternion(roll=0,pitch=0,yaw=np.pi/2)
            msg.pose.pose.orientation.x = q[0].item()
            msg.pose.pose.orientation.y = q[1].item()
            msg.pose.pose.orientation.z = q[2].item()
            msg.pose.pose.orientation.w = q[3].item()
            self.__odo_publisher.publish(msg)
            

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Producer())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
