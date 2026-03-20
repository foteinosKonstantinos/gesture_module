import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as SensorImage, NavSatFix
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from PIL import Image as PILImage
import numpy as np
from rclpy.executors import ExternalShutdownException
import time
import math

EARTH_RADIUS = 6378137.0 # in meters
PATH = "/app"

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def abs_xy_to_gps(x, y) -> tuple[float]:
    '''
    abs_xy -> GPS
    Parameters:
        x,y:        With origin the initial robot position and "orientation" the same with the "flatten" meridians/parallels (in mm)
    Returns:
        - longitude:  GPS (degrees)
        - latitude:   GPS (degrees)
    '''
    lat = ((y/1000) / EARTH_RADIUS) * (180.0 / math.pi)
    lon = ((x/1000) / (EARTH_RADIUS * math.cos(math.radians(0)))) * (180.0 / math.pi)
    return float(lon), float(lat)

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
        self.__broadcaster = TransformBroadcaster(self)

        depth_frames = [
            "frames/high_Come-to-me_2_depth.png",
            "frames/high_Come-to-me_98_depth.png",
            "frames/high_Come-to-me_1214_depth.png",
            "frames/high_Emergency-situation_101_depth.png",
            "frames/high_Evacuate-the-area_175_depth.png",
            "frames/high_Fetch-a-gas-mask_49_depth.png",
            "frames/high_Fetch-a-gas-mask_181_depth.png",
            "frames/high_Fetch-a-shovel_33_depth.png",
            "frames/high_Freeze_16_depth.png",
            "frames/high_Freeze_40_depth.png",
            "frames/high_Freeze_184_depth.png",
            "frames/high_Ok-to-go_203_depth.png",
            "frames/high_Ok-to-go_263_depth.png",
            "frames/high_Ok-to-go_263_depth.png", # dummy
            "frames/high_Ok-to-go_263_depth.png"  # dummy
        ]

        rgb_frames = [
            "frames/high_Come-to-me_2_color.png",
            "frames/high_Come-to-me_98_color.png",
            "frames/high_Come-to-me_1214_color.png",
            "frames/high_Emergency-situation_101_color.png",
            "frames/high_Evacuate-the-area_175_color.png",
            "frames/high_Fetch-a-gas-mask_49_color.png",
            "frames/high_Fetch-a-gas-mask_181_color.png",
            "frames/high_Fetch-a-shovel_33_color.png",
            "frames/high_Freeze_16_color.png",
            "frames/high_Freeze_40_color.png",
            "frames/high_Freeze_184_color.png",
            "frames/high_Ok-to-go_203_color.png",
            "frames/high_Ok-to-go_263_color.png",
            "frames/multi_person.png",
            "frames/no_person.png"
        ]

        total = len(rgb_frames)
        assert len(depth_frames) == total

        x_mm = 0.0
        idx = 0

        while True:

            time.sleep(1)

            
            depth_path = f"{path}/{depth_frames[idx]}"
            color_path = f"{path}/{rgb_frames[idx]}"
            self.get_logger().info(f"Publishing {color_path} and {depth_path}...")
            idx = (idx + 1) % total

            depth = np.asarray(PILImage.open(depth_path),dtype=np.uint16)
            color = np.asarray(PILImage.open(color_path).convert("RGB"))
            
            stamp = self.get_clock().now().to_msg()

            msg = SensorImage()
            msg.header.stamp = stamp
            msg.header.frame_id = "camera_depth_frame"
            msg.height = depth.shape[0]
            msg.width = depth.shape[1]
            msg.encoding = "16UC1"
            msg.is_bigendian = False
            msg.step = 2 * depth.shape[1]
            msg.data = depth.tobytes()
            self.__depth_publisher.publish(msg)

            msg = SensorImage()
            msg.header.stamp = stamp
            msg.header.frame_id = "camera_depth_frame"
            msg.height = color.shape[0]
            msg.width = color.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = 3 * color.shape[1]
            msg.data = color.tobytes()
            self.__color_publisher.publish(msg)

            msg = CameraInfo()
            msg.header.stamp = stamp
            msg.header.frame_id = "camera_depth_frame"
            msg.height = color.shape[0]
            msg.width = color.shape[1]
            msg.k = [500.0, 0.0, 640.0, 0.0, 500.0, 360.0, 0.0, 0.0, 1.0]
            self.__info_publisher.publish(msg)

            msg = NavSatFix()
            msg.header.stamp = stamp
            (msg.longitude, msg.latitude) = abs_xy_to_gps(x=x_mm,y=0)
            self.__gps_publisher.publish(msg)

            q = euler_to_quaternion(roll=0, pitch=0, yaw=np.pi/2)
            base_to_map = TransformStamped()
            base_to_map.header.stamp = stamp
            base_to_map.header.frame_id = 'map'
            base_to_map.child_frame_id = 'base_link'
            base_to_map.transform.translation.x = float(x_mm / 1000.0)
            base_to_map.transform.translation.y = 0.0
            base_to_map.transform.translation.z = 0.0
            base_to_map.transform.rotation.x = float(q[0].item())
            base_to_map.transform.rotation.y = float(q[1].item())
            base_to_map.transform.rotation.z = float(q[2].item())
            base_to_map.transform.rotation.w = float(q[3].item())
            self.__broadcaster.sendTransform(base_to_map)

            camera_to_base = TransformStamped()
            camera_to_base.header.stamp = stamp
            camera_to_base.header.frame_id = "base_link"
            camera_to_base.child_frame_id = "camera_depth_frame"
            camera_to_base.transform.translation.x = 0.0
            camera_to_base.transform.translation.y = 0.0
            camera_to_base.transform.translation.z = 0.0
            camera_to_base.transform.rotation.x = 0.0
            camera_to_base.transform.rotation.y = 0.0
            camera_to_base.transform.rotation.z = 0.0
            camera_to_base.transform.rotation.w = 1.0
            self.__broadcaster.sendTransform(camera_to_base)

            x_mm += 1000

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Producer())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
