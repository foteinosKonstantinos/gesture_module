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
# import onnxruntime
import torch
import torchvision
from ultralytics import YOLO

EARTH_RADIUS = 6378137.0 # in meters

# Deployment details

# ONNX_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnet.onnx"
CLASSIFICATION_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnetb0_color_pretrained.pt"
POSE_ESTIMATOR = "yolo26n-pose.pt"

CLASSIFICATION_THRESHOLD = 0.99
POSE_ESTIMATION_THRESHOLD = 0.9

NAV_TOPIC = "/fix"
ODOM_TOPIC = "/dog_odom"
DEPTH_TOPIC = "/camera_front/depth"
RGB_TOPIC = "/camera_front/color"
CAMERA_INFO = "/camera_front/camera_info"
OUTPUT_TOPIC = "/gesture_command"

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

    def __init__(self, 
                # classifier_onnx:str=ONNX_MODEL, 
                classifier:str=CLASSIFICATION_MODEL,
                pose_estimator:str=POSE_ESTIMATOR, 
                color_topic:str=RGB_TOPIC, 
                depth_topic:str=DEPTH_TOPIC, 
                camera_info:str=CAMERA_INFO, 
                nav_fix_topic:str=NAV_TOPIC, 
                odom_topic:str=ODOM_TOPIC, 
                output_topic:str=OUTPUT_TOPIC,
                classification_threshold:float = CLASSIFICATION_THRESHOLD,
                pose_estimation_threshold:float = POSE_ESTIMATION_THRESHOLD):
        super().__init__("gesture_classifier")
        ApproximateTimeSynchronizer(
            fs=[
                Subscriber(self, Image, color_topic), 
                Subscriber(self, Image, depth_topic), 
                Subscriber(self, CameraInfo, camera_info),
                Subscriber(self, NavSatFix, nav_fix_topic), 
                Subscriber(self, Odometry, odom_topic)
            ],
            queue_size=10,
            slop=1e-2
        ).registerCallback(self.__main_callback)
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = output_topic,
            qos_profile = 10
        )
        # self.__recognition_session = onnxruntime.InferenceSession(classifier_onnx)
        self.__device = "cuda" if torch.cuda.is_available() else "cpu"
        self.__classifier = torchvision.models.efficientnet_b0(num_classes=len(CLASSES))
        self.__classifier.load_state_dict(torch.load("/content/efficientnetb0_color_pretrained.pt",map_location=torch.device(self.__device)))
        self.__classification_threshold = classification_threshold
        self.__pose_estimator = YOLO(pose_estimator)
        self.__pose_estimation_threshold = pose_estimation_threshold
        self.__counter = 0
        self.__init_latitude = None
        self.__init_longitude = None
        self.get_logger().info(f"Successfully initialized the classification node, with weights {classifier} running on {self.__device}.")


    def __detect_keypoints(self, image, depth_map, names = KEYPOINTS) -> list[dict]:
        '''
        Parameters:
            image:      numpy array with dimensions height x width x 3 (H x W x 3)
            depth_map:  numpy array with dimensions height x width x 1 (H x W x 1)
        Returns:
            [
                {
                    Keys:       Keypoint name
                    Values:     [u coordinate (pixels), v coordinate (pixels), confidence ([0,1]), depth (units similar to the depth_map)] (primitives, not np arrays)
                },
                for every detected person
                ...
            ]
        '''
        result = self.__pose_estimator(image, verbose=False)[0].keypoints.data
        keypoints = []
        for person in range(len(result)):
            keypoints.append(dict({}))
            for i in range(len(names)):
                uvcd:list = result[person][i][[0,1]].cpu().numpy().astype(int).tolist()
                uvcd.append(result[person][i][2].item())
                try:
                    uvcd.append(depth_map[uvcd[1],uvcd[0]].item())
                except IndexError:
                    uvcd.append(0)
                keypoints[-1][names[i]] = uvcd
        return keypoints

    
    def __aggregate(self, keypoints:dict) -> float:
        '''
        Parameters:
            keypoints:  Keypoints of a single person in the format of __detect_keypoints output
        Returns:
            d:      average depth of the keypoints (in the given depth measurement units)
            u:      average u (in pixels)
            v:      average v (in pixels)
            c:      average confidence ([0,1])
        '''
        # d = u = v = c = total = 0
        # for keypoint_name in keypoints.keys():
        #     if keypoints[keypoint_name][3] == 0:
        #         continue
        #     total += 1
        #     u += keypoints[keypoint_name][0]
        #     v += keypoints[keypoint_name][1]
        #     c += keypoints[keypoint_name][2]
        #     d += keypoints[keypoint_name][3]
        # d /= total
        # u /= total
        # v /= total
        # c /= total
        # return d, u, v, c
        ls = keypoints["Left Shoulder"]
        rs = keypoints["Right Shoulder"]
        if ls[3] !=0 and rs[3] !=0:
            u = (ls[0]+rs[0])/2
            v = (ls[1]+rs[1])/2
            c = (ls[2]+rs[2])/2
            d = (ls[3]+rs[3])/2
            return d, u, v, c
        return None

    def __estimate_relative_location(self, u, v, depth, intrinsics) -> np.ndarray:
        '''
        Backprojects a point to 3D space
        Parameters:
            u:          x' (horizontal) (in pixels)
            v:          y' (vertical) (in pixels)
            depth:      disantce from the optical center (in mm)
            intrinsics: camera intrinsics (in pixels)
        Returns:
            the position in 3D space (in mm)
        '''
        p_2D_h = np.asarray([u, v, 1]) # homogeneous coordinates
        p_3D = depth * (np.linalg.inv(intrinsics) @ p_2D_h)
        return p_3D


    def __estimate_absolute_location(self, det_distance, cam_lat, cam_lon, orientation_x, orientation_y):
        '''
        Parameters:
            det_distance:   Distance between the detection and the camera (assume that the detection is in the line of the orientation of the camera) (in mm)
            cam_lat:        Current global latitude of the camera/robot
            cam_lon:        Current global longitude of the camera/robot
            orientation_x:  W.r.t. to the xy coordinate system that satisfies xx' // parallels, yy' // meridians (approximately) (*)
            orientation_y:  The same with orientation_x
        Returns:
            longitude:      GPS (degrees)
            latitude:       GPS (degrees)
        '''
        cam_x, cam_y = self.__global_to_xy_position(lat=cam_lat, lon=cam_lon) # in mm
        norm = math.sqrt(orientation_x ** 2 + orientation_y ** 2) * 1000 # in mm TODO !!!
        det_x = cam_x + det_distance * orientation_x / norm
        det_y = cam_y + det_distance * orientation_y / norm
        return self.__xy_to_global_position(x=det_x, y=det_y)


    def __xy_to_global_position(self, x, y) -> list:
        '''
        Parameters:
            x,y:        With origin the initial robot position and "orientation" the same with the "flatten" meridians/parallels (in mm)
        Returns:
            longitude:  GPS (degrees)
            latitude:   GPS (degrees)
        '''
        lat = self.__init_latitude + ((y/1000) / EARTH_RADIUS) * (180.0 / math.pi)
        lon = self.__init_longitude + ((x/1000) / (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude)))) * (180.0 / math.pi)
        return lon, lat
    

    def __global_to_xy_position(self, lat, lon):
        '''
        The inverse of the previous
        '''
        y = (lat - self.__init_latitude) * (math.pi / 180.0) * EARTH_RADIUS # in meters
        x = (lon - self.__init_longitude) * (math.pi / 180.0) * (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude))) # in meters
        return x * 1000, y * 1000 # (in mm)


    def __register_initial_global_position(self, position:NavSatFix):
        if self.__init_latitude is None or self.__init_longitude is None:
            self.__init_latitude = position.latitude
            self.__init_longitude = position.longitude
            self.get_logger().info(f"Initial position in (latitude, longitude) = ({self.__init_latitude}, {self.__init_longitude})")

        
    def __predict_from_image(self, image, classes=CLASSES):
        # probabilities = self.__recognition_session.run(None, {"input":cv2.resize(image, dsize=(640, 480)).reshape((1,3,480,640))})[0]
        probabilities = self.__classifier(torch.as_tensor(cv2.resize(image, dsize=(640, 480)).reshape((1,3,480,640))).to(self.__device))[0].detach().cpu().numpy()
        argmax = probabilities.argmax()
        pred_class = classes[argmax]
        confidence = probabilities[argmax].item()
        return {
            "class": pred_class,
            "confidence": confidence
        }
    

    def __quaternion_to_rpy(self, qx, qy, qz, qw):
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        return {
            "roll": math.atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy)),
            "pitch": math.asin(2.0*(qw*qy - qz*qx)),
            "yaw": math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        }


    def __main_callback(self, color_image:Image, depth_map:Image, intrinsics:CameraInfo, global_position:NavSatFix, odometry:Odometry):
        '''
        Parameters:
            color_image:    8-bit RGB image (H x W x 3)
            depth_map:      16UC1 in mm depth map (H x W x 2) of the same dimensions and aligned to the color_image
            intrinsics:     Camera intrinsics (the code uses only K matrix)
            global_position:Longitude/latitude (degrees)
            odomometry:     The code needs the "absolute" orientation (w.r.t. to the "standard" xy plane), as described in (*)
        Publishes:
            See README
        '''

        assert color_image.height == depth_map.height and color_image.width == depth_map.width

        self.__register_initial_global_position(global_position)
        angle = math.radians(self.__quaternion_to_rpy(odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w)["yaw"])
        self.get_logger().info(f"Received RGBD frames of size {color_image.height} x {color_image.width} (H x W) at {self.__global_to_xy_position(lat=global_position.latitude, lon=global_position.longitude)} (mm) and angle {angle} (radians)")
        
        color_image_array = np.asarray(color_image.data, dtype=np.float32).reshape((color_image.height, color_image.width, 3)) # H x W x 3
        depth_map_array = np.asarray(np.frombuffer(depth_map.data,dtype=np.uint16), dtype=np.float32).reshape((depth_map.height, depth_map.width, 1)) # H x W x 1
        
        all_keypoints = self.__detect_keypoints(image=color_image_array, depth_map=depth_map_array)

        if len(all_keypoints) == 0:
            return
        
        argmin_u = None
        argmin_v = None
        argmin_idx = None
        min_depth = math.inf
        for idx, single_person_keypoints in enumerate(all_keypoints):
            agg = self.__aggregate(single_person_keypoints)
            if agg is None:
                continue
            depth, u, v, c = agg[0], agg[1], agg[2], agg[3]
            if depth < min_depth:
                argmin_u = u
                argmin_v = v
                min_depth = depth
                argmin_idx = idx
        
        if argmin_u is None:
            return
        
        relative_position = self.__estimate_relative_location(u=argmin_u,v=argmin_v,depth=min_depth,intrinsics=np.asarray(intrinsics.k).reshape((3,3)))
        det_global_position = self.__estimate_absolute_location(min_depth, global_position.latitude, global_position.longitude, math.cos(angle), math.sin(angle))
        
        prediction = self.__predict_from_image(color_image_array)
        if prediction["confidence"] < self.__classification_threshold:
            self.get_logger().warning(f"Detection position: {det_global_position} Class: {prediction['class']} - LOW CONFIDENCE (<{self.__classification_threshold})")
            return
        
        self.__publisher.publish(String(data=json.dumps({
            "type": "FeatureCollection",
            "features":[
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": det_global_position
                    },
                    "properties": {
                        "class":prediction["class"],
                        "confidence":prediction["confidence"],
                        "depth":depth,
                        "id":self.__counter,
                        "timestamp":self.get_clock().now().nanoseconds,
                        "keypoints_and_depths": all_keypoints[argmin_idx],
                        "relative_position": relative_position.tolist()
                    }
                }
            ]
        })))
        self.__counter += 1
        self.get_logger().info(f"Detection position: {det_global_position} Class: {prediction['class']} with confidence {prediction['confidence']} at depth {depth}")


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Gesture_Classifier())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
