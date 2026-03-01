# Foteinos Konstantinos (HUA)
# Contact: kfoteinos@hua.gr

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import json
import math
import numpy as np
import onnxruntime
from ultralytics import YOLO

EARTH_RADIUS = 6378137.0

# Deployment details
ONNX_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnet.onnx"
POSE_ESTIMATOR = "yolo26n-pose.pt"
FOCAL_LENGTH = 1 # typical values of focal length are <10 mm for ordinary cameras; therefore, the impact on the final results is negligible

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

KEYPOINTS = [
    "Nose",
    "Left Eye",
    "Right Eye",
    "Left Ear",
    "Right Ear",
    "Left Shoulder",
    "Right Shoulder",
    "Left Elbow",
    "Right Elbow",
    "Left Wrist",
    "Right Wrist",
    "Left Hip",
    "Right Hip",
    "Left Knee",
    "Right Knee",
    "Left Ankle",
    "Right Ankle"
]


class Gesture_Classifier(Node):

    def __init__(self, classifier_onnx:str=ONNX_MODEL, pose_estimator:str=POSE_ESTIMATOR, fg0:float=80.09993757800315, d0:float=4612.0):
        '''
        Parameters:
            classifier_onnx:    Path to an onnx model for classification; input image 3x480x640 and 12 classes
            pose_estimator:     YOLO (human) pose estimator, supported by ultralytics YOLO (will be downloaded automatically if is not already downloaded)
            classes:            The label for each one of the 12 gestures
            fg0:                Calibration distance between the left and right shoulder keypoints
            d0:                 Calibration distance between the signer and the camera
        Associated topics:
            Input:              topic /camera_front/raw_image and message type sensor_msgs/msg/Image
            Output:             topic /gesture_command and message type std_msgs/msg/String
        '''
        super().__init__("gesture_classifier")
        ApproximateTimeSynchronizer(
            fs=[Subscriber(self, Image, "/camera_front/raw_image"), Subscriber(self, CameraInfo, "/camera_front/camera_info"), Subscriber(self, NavSatFix, "/fix"), Subscriber(self, Odometry, "/dog_odom")],
            queue_size=10,
            slop=1e-2
        ).registerCallback(self.__main_callback)
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = "/gesture_command",
            qos_profile = 10
        )
        self.__recognition_session = onnxruntime.InferenceSession(classifier_onnx)
        self.__pose_estimator = YOLO(pose_estimator)
        self.__fg0 = fg0
        self.__d0 = d0
        self.__counter = 0
        self.__init_latitude = None
        self.__init_longitude = None
        self.get_logger().info(f"Successfully initialized the classification node, with weights {classifier_onnx}.")


    def __detect_keypoints(self, image, names = KEYPOINTS) -> list[dict]: # H x W x 3 nd array
        '''
        Parameters:
            image:  numpy array with dimensions height x width x 3 (H x W x 3)
        Returns:
            Keypoint name/vu coordinates pairs for every detected person
        '''
        result = self.__pose_estimator(image, verbose=False)[0].keypoints.data
        keypoints = []
        for person in range(len(result)):
            keypoints.append(dict({}))
            for i in range(len(names)):
                keypoints[-1][names[i]] = result[person][i][[0,1]].numpy().astype(int).tolist()
        return keypoints
    

    def __estimate_depth_from_keypoints(self, keypoints:dict) -> float:
        '''
        Returns: detection uv and depth d
        '''
        ls = np.asarray(keypoints["Left Shoulder"])
        rs = np.asarray(keypoints["Right Shoulder"])
        fg = ((ls-rs)**2).sum().item() ** (1/2)
        d = (self.__fg0 * self.__d0) / fg
        u = (ls[1]+rs[1])/2
        v = (ls[0]+rs[0])/2
        return d, u, v


    def __estimate_depths_geometrically(self, image):
        '''
        Returns the average depth of the two considered keypoints (left and right shoulder).
        Parameters:
            image:  numpy array with dimensions height x width x 3 (H x W x 3)
        Returns:
            d:      depth, i.e. distance from the optical center (scalar)
        "Closest person" assumption
        '''
        keypoints = self.__detect_keypoints(image=image)
        if len(keypoints)==0:
            return None, None, None, None
        min_depth = math.inf
        argmin_u = None
        argmin_v = None
        keypoints_and_depths = []
        for person in range(len(keypoints)):
            depth, u, v = self.__estimate_depth_from_keypoints(keypoints[person])
            if depth<min_depth:
                min_depth = depth
                argmin_u = u
                argmin_v = v
            keypoints_and_depths.append(keypoints[person])
            keypoints_and_depths[-1]["depth"] = depth
        return min_depth, argmin_u, argmin_v, keypoints


    def __estimate_relative_location(self, u, v, depth, intrinsics, f=FOCAL_LENGTH) -> np.ndarray:
        '''
        Backproject
        u = x' = horizontal
        v = y' = vertical
        '''
        Z = depth - f # focal length
        K = intrinsics
        P_2D_h = np.asarray([u, v, 1]) # homogeneous coordinates
        P_3D = np.linalg.inv(K) @ (Z * P_2D_h)
        return P_3D


    def __estimate_absolute_location(self, det_distance, cam_lat, cam_lon, orientation_x, orientation_y):
        '''
        Parameters:
            det_distance:           Distance between the detection and the camera (assume that the detection is in the line of the orientation of the camera)
            cam_global_position:    Current global position of the camera (which is considered to be the same with the robot's)
        '''
        cam_x, cam_y = self.__global_to_xy_position(lat=cam_lat, lon=cam_lon)
        norm = math.sqrt(orientation_x ** 2 + orientation_y ** 2)
        det_x = cam_x + det_distance * orientation_x / norm
        det_y = cam_y + det_distance * orientation_y / norm
        return self.__xy_to_global_position(x=det_x, y=det_y)


    def __xy_to_global_position(self, x, y) -> list:
        '''
        Based on: https://github.com/AnnaMi0/triffid-perception/blob/master/src/triffid_ugv_perception/triffid_ugv_perception/geojson_bridge.py
        '''
        lat = self.__init_latitude + (y / EARTH_RADIUS) * (180.0 / math.pi)
        lon = self.__init_longitude + (x / (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude)))) * (180.0 / math.pi)
        return lon, lat
    

    def __global_to_xy_position(self, lat, lon):
        '''
        The inverse of the previous
        '''
        y = (lat - self.__init_latitude) * (math.pi / 180.0) * EARTH_RADIUS
        x = (lon - self.__init_longitude) * (math.pi / 180.0) * (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude)))
        return x, y


    def __register_initial_global_position(self, position:NavSatFix):
        if self.__init_latitude is None or self.__init_longitude is None:
            self.__init_latitude = position.latitude
            self.__init_longitude = position.longitude
            self.get_logger().info(f"Initial position in (latitude, longtitude)) = ({self.__init_latitude}, {self.__init_longitude})")

        
    def __predict_from_image(self, image, classes=CLASSES):
        probabilities = self.__recognition_session.run(None, {"input":cv2.resize(image, dsize=(640, 480)).reshape((1,3,480,640))})[0]
        argmax = np.asarray(probabilities).argmax()
        pred_class = classes[argmax]
        confidence = probabilities[argmax].item()
        return {
            "class": pred_class,
            "confidence": confidence
        }
    

    def __quaternion_to_rpy(self, qx, qy, qz, qw):
        return {
            "roll": math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz),
            "pitch":math.asin(-2.0*(qx*qz - qw*qy)),
            "yaw":  math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        }


    def __main_callback(self, image:Image, intrinsics:CameraInfo, global_position:NavSatFix, odometry:Odometry):
        '''
        Parameters:
            in_data:    colored image represnted as sensor_msgs/msg/Image of arbitrary dimensions
        '''
        self.get_logger().info(f"Recieved image of size {image.height} x {image.width} (H x W)")
        self.__register_initial_global_position(global_position)
        image = np.asarray(image.data, dtype=np.float32).reshape((image.height, image.width, 3)) # H x W x 3
        depth, u, v, keypoints_and_depths = self.__estimate_depths_geometrically(image)
        if depth is None:
            return
        relative_position = self.__estimate_relative_location(u=u,v=v,depth=depth,intrinsics=np.asarray(intrinsics.k).reshape((3,3)))
        angle = self.__quaternion_to_rpy(odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w)["yaw"]
        global_position = self.__estimate_absolute_location(depth, global_position.latitude, global_position.longitude, math.cos(angle), math.sin(angle))
        prediction = self.__predict_from_image(image)
        self.__publisher.publish(String(data=json.dumps({
            "type": "FeatureCollection",
            "features":[
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": global_position
                    },
                    "properties": {
                        "class":prediction["class"],
                        "confidence":prediction["confidence"],
                        "depth":depth,
                        "id":self.__counter,
                        "timestamp":self.get_clock().now().nanoseconds,
                        "keypoints_and_depths": keypoints_and_depths,
                        "relative_position": relative_position
                    }
                }
            ]
        })))
        self.__counter += 1
        self.get_logger().info(f"published \'{prediction['class']}\' with confidence {prediction['confidence']} at depth {depth}")


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Gesture_Classifier())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
