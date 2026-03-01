# Foteinos Konstantinos (HUA)
# Contact: kfoteinos@hua.gr

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
import cv2
import json
import numpy as np
import onnxruntime
from ultralytics import YOLO


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

class Gesture_Classifier(Node):

    def __init__(self, classifier_onnx:str=ONNX_MODEL, pose_estimator:str=POSE_ESTIMATOR, classes:list[str]=CLASSES, fg0:float=80.09993757800315, d0:float=4612.0):
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
        self.__classes = classes
        
        # self.create_subscription(
        #     msg_type=Image,
        #     topic="/camera_front/raw_image",
        #     callback=self.__prediction_callback,
        #     qos_profile=10
        # )

        ApproximateTimeSynchronizer(
            fs=[Subscriber(self, Image, "/camera_front/raw_image"), Subscriber(self, CameraInfo, "/camera_front/camera_info"), Subscriber(self, NavSatFix, "/fix")],
            queue_size=10,
            slop=1e-2
        ).registerCallback(self.__prediction_callback)

        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = "/gesture_command",
            qos_profile = 10
        )
        self.__session = onnxruntime.InferenceSession(classifier_onnx)
        self.__pose_estimator = YOLO(pose_estimator)
        self.__fg0 = fg0
        self.__d0 = d0
        self.__counter = 0
        self.get_logger().info(f"Successfully initialized the classification node, with weights {classifier_onnx} for {len(self.__classes)} classes.")

    def __estimate_depth_geometrically(self, image):
        '''
        Returns the average depth of the two considered keypoints (left and right shoulder).
        Parameters:
            image:  numpy array with dimensions height x width x 3 (H x W x 3)
        **IMPORTANT**: Assumes that at most one person is present (selects the first prediction, if exists)
        '''
        self.get_logger().warning("TODO: Recognize gesture in the presence of multiple humans.")
        result = self.__pose_estimator(image, verbose=False)[0]
        if len(result.keypoints.data)==0:
            return None
        person = 0 # TODO: Define a policy for selecting persons (e.g. the closest one)
        ls = result.keypoints.data[person][5][[0,1]].numpy().astype(int) # left shoulder
        rs = result.keypoints.data[person][6][[0,1]].numpy().astype(int) # right shoulder
        fg = ((ls-rs)**2).sum().item() ** (1/2)
        d = (self.__fg0 * self.__d0) / fg
        u = (ls[1]+rs[1])/2
        v = (ls[0]+rs[0])/2
        return d, u, v
    
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

    def __estimate_absolute_location(self, relative_position, latitude, longitude) -> tuple[float]:
        pass

    def __prediction_callback(self, in_data:Image, intrinsics:CameraInfo, global_position:NavSatFix):
        '''
        Parameters:
            in_data:    colored image represnted as sensor_msgs/msg/Image of arbitrary dimensions
        '''
        
        # WARNING: Position estimation w.r.t to the initial image resolution

        image = np.asarray(in_data.data, dtype=np.float32).reshape((in_data.height, in_data.width, 3)) # H x W x 3
        depth, u, v = self.__estimate_depth_geometrically(image)
        if depth is None:
            self.get_logger().info(f"recieved: {in_data.height} x {in_data.width} no detected person")
            return
        relative_position = self.__estimate_relative_location(u, v, depth=depth, intrinsics=np.asarray(intrinsics.k).reshape((3,3)))
        global_position = self.__estimate_absolute_location(self, relative_position, global_position.latitude, global_position.longitude)

        self.get_logger().info(f"recieved: {in_data.height} x {in_data.width} detected person in distance: {depth} and position: {global_position}")
        probs = self.__session.run(None, {"input":cv2.resize(image, dsize=(640, 480)).reshape((1,3,480,640))})
        argmax_idx = np.asarray(probs[0]).argmax()
        pred_class = self.__classes[argmax_idx]
        confidence = probs[0][argmax_idx].item()
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
                        "class":pred_class,
                        "confidence":confidence,
                        "depth":depth,
                        "id":self.__counter,
                        "relative_position":relative_position.tolist()
                    }
                }
            ]
        })))
        self.__counter += 1
        self.get_logger().info(f"published \'{pred_class}\' and confidence {confidence}")

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Gesture_Classifier())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass

if __name__ == '__main__':
    main()
