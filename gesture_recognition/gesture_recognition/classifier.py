import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose, Transform
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import json
import math
import numpy as np
import torch
import torchvision
import torchvision.transforms as transforms
from ultralytics import YOLO
from torch.nn.functional import softmax
from robal_interfaces.action import NavigateTo, Trigger, ReturnToBaseFetch, HelpRequest, ReturnToBase
import time

# Come to me            => NavigateTo           (/b2/local/trigger_navigation)
# Unfreeze (ok to go)   => Trigger              (/b2/local/trigger_freeze)
# Move away from here   => Trigger              (/b2/local/trigger_retreat)
# Operation finished    => ReturnToBase         (/b2/local/trigger_return_to_base)
# Freeze                => Trigger              (/b2/local/trigger_freeze)
# Stop                  => Trigger              (/b2/local/trigger_stop)
# Emergency situation   => Trigger              (/b2/global/trigger_emergency)
# I need help           => HelpRequest          (/b2/local/trigger_help_request)]           [or NavigateTo (/b2/local/trigger_navigation) ?]
# Evacuate the area     => ReturnToBase         (/b2/local/trigger_return_to_base)          [TODO]
# I lost connection     => HelpRequest          (/b2/local/trigger_help_request)
# Fetch a gas mask      => ReturnToBaseFetch    (/b2/local/trigger_return_to_base_fetch)
# Featch a shovel       => ReturnToBaseFetch    (/b2/local/trigger_return_to_base_fetch)
# Fetch an axe          => ReturnToBaseFetch    (/b2/local/trigger_return_to_base_fetch)

NO_UNDERLYING_IMPL = True # Change this to False during integration with the UPC

EARTH_RADIUS = 6378137.0 # in meters

CLASSIFICATION_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnetb0_color_pretrained_ext.pt"
POSE_ESTIMATOR = "yolo26n-pose.pt"

CLASSIFICATION_THRESHOLD = 0.95
POSE_ESTIMATION_THRESHOLD = 0.95
DEPTH_THRESHOLD = 7000 # in mm
TARGET_TIMEOUT_SECONDS = 1e-1
SLOP = 1e-1
MAX_FPS = 1
MIN_OCCURS = MAX_FPS

NAV_TOPIC = "/fix"
DEPTH_TOPIC = "/camera_front/depth"
RGB_TOPIC = "/camera_front/color"
CAMERA_INFO = "/camera_front/camera_info"
OUTPUT_TOPIC = "/gesture_command"

# Gesture commands. The ordering is crucial.
CLASSES = [
    "fetch-a-gas-mask", # G0
    "come-to-me", # G1
    "unfreeze", # G10 (previously named "ok-to-go")
    "move-away-from-here", # G11
    "stop", # G12
    "operation-finished", # G2
    "freeze", # G3
    "emergency-situation", # G4
    "i-need-help", # G5
    "evacuate-the-area", # G6
    "i-lost-connection", # G7
    "fetch-a-shovel", # G8
    "fetch-an-axe" # G9
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
            classifier:str=CLASSIFICATION_MODEL,
            pose_estimator:str=POSE_ESTIMATOR, 
            color_topic:str=RGB_TOPIC, 
            depth_topic:str=DEPTH_TOPIC, 
            camera_info:str=CAMERA_INFO, 
            nav_fix_topic:str=NAV_TOPIC,
            output_topic:str=OUTPUT_TOPIC,
            classification_threshold:float = CLASSIFICATION_THRESHOLD,
            pose_estimation_threshold:float = POSE_ESTIMATION_THRESHOLD,
            depth_threshold:float = DEPTH_THRESHOLD
        ):
        super().__init__("gesture_classifier")
        self.__time_synchronizer = ApproximateTimeSynchronizer(
            fs=[
                Subscriber(self, Image, color_topic), 
                Subscriber(self, Image, depth_topic), 
                Subscriber(self, CameraInfo, camera_info),
                Subscriber(self, NavSatFix, nav_fix_topic)
            ],
            queue_size=10,
            slop=SLOP
        )
        self.__time_synchronizer.registerCallback(self.__main_callback)
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = output_topic,
            qos_profile = 10
        )
        self.__device = "cuda" if torch.cuda.is_available() else "cpu"
        self.__classifier = torchvision.models.efficientnet_b0(num_classes=len(CLASSES))
        self.__classifier.load_state_dict(torch.load(classifier,map_location=torch.device(self.__device)))
        self.__classifier = self.__classifier.to(self.__device)
        self.__classifier.eval()
        self.__to_tensor = transforms.ToTensor()
        self.__resize = transforms.Resize((224,224))
        self.__classification_threshold = classification_threshold
        self.__pose_estimator = YOLO(pose_estimator)
        self.__pose_estimation_threshold = pose_estimation_threshold
        self.__depth_threshold = depth_threshold
        self.__counter = 0
        
        self.__init_latitude = None
        self.__init_longitude = None
        self.__tf_buffer = tf2_ros.Buffer()
        self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer, self)

        self.__stop = ActionClient(self, Trigger, "/b2/local/trigger_stop")
        self.__help = ActionClient(self, HelpRequest, "/b2/local/trigger_help_request")
        self.__fetch = ActionClient(self, ReturnToBaseFetch, "/b2/local/trigger_return_to_base_fetch")
        self.__freeze = ActionClient(self, Trigger, "/b2/local/trigger_freeze")
        self.__retreat = ActionClient(self, Trigger, "/b2/local/trigger_retreat")
        self.__emergency = ActionClient(self, Trigger, "/b2/global/trigger_emergency")
        self.__return_bos = ActionClient(self, ReturnToBase, "/b2/local/trigger_return_to_base")
        self.__navigation = ActionClient(self, NavigateTo, "/b2/local/trigger_navigation")
        
        self.get_logger().info(f"Successfully initialized the classification node, with weights {classifier} running on {self.__device}.")
        self.__log_counter = 0
        self.__last = None

        self.__previous_command = None
        self.__counter_command = 0

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
            Only the left & right shoulder are considered
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


    def __uvd_to_rel_xyz(self, u, v, depth, intrinsics) -> np.ndarray:
        '''
        uvd -> rel_xyz (Backprojects a point to 3D space)
        Parameters:
            u:          x' (horizontal) (in pixels)
            v:          y' (vertical) (in pixels)
            depth:      disantce from the optical center (in mm)
            intrinsics: camera intrinsics (in pixels)
        Returns:
            the relative position in 3D space (in mm) w.r.t. to camera frame
        '''
        p_2D_h = np.asarray([u, v, 1]) # homogeneous coordinates
        p_3D = depth * (np.linalg.inv(intrinsics) @ p_2D_h)
        return p_3D


    def __rel_xyz_to_base_xyz(self, xyz:np.ndarray, stamp) -> tuple[float]:
        '''xyz in mm'''
        msg = PointStamped()
        msg.header.frame_id = "camera_depth_frame"
        msg.header.stamp = stamp
        msg.point.x = xyz[0].item() / 1000
        msg.point.y = xyz[1].item() / 1000
        msg.point.z = xyz[2].item() / 1000
        transform = self.__tf_buffer.transform(msg,"base_link",timeout=rclpy.duration.Duration(seconds=TARGET_TIMEOUT_SECONDS))
        return float(transform.point.x) * 1000,float(transform.point.y) * 1000,float(transform.point.z) * 1000


    def __base_xyz_to_abs_xyz(self, xyz:tuple[float], stamp) -> tuple[float]:
        '''xyz in mm'''
        msg = PointStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = stamp
        msg.point.x = xyz[0] / 1000
        msg.point.y = xyz[1] / 1000
        msg.point.z = xyz[2] / 1000
        transform = self.__tf_buffer.transform(msg,"map",timeout=rclpy.duration.Duration(seconds=TARGET_TIMEOUT_SECONDS)) # map or odom
        return float(transform.point.x) * 1000,float(transform.point.y) * 1000,float(transform.point.z) * 1000
    

    def __abs_xy_to_gps(self, x, y) -> tuple[float]:
        '''
        abs_xy -> GPS
        Parameters:
            x,y:        With origin the initial robot position and "orientation" the same with the "flatten" meridians/parallels (in mm)
        Returns:
            - longitude:  GPS (degrees)
            - latitude:   GPS (degrees)
        '''
        lat = self.__init_latitude + ((y/1000) / EARTH_RADIUS) * (180.0 / math.pi)
        lon = self.__init_longitude + ((x/1000) / (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude)))) * (180.0 / math.pi)
        return float(lon), float(lat)
    

    def __gps_to_abs_xy(self, lat, lon) -> tuple[float]:
        '''
        GPS -> abs_xy (the inverse of the previous)
        '''
        y = (lat - self.__init_latitude) * (math.pi / 180.0) * EARTH_RADIUS # in meters
        x = (lon - self.__init_longitude) * (math.pi / 180.0) * (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude))) # in meters
        return float(x * 1000), float(y * 1000) # (in mm)


    def __register_initial_gps(self, position:NavSatFix):
        if self.__init_latitude is None or self.__init_longitude is None:
            self.__init_latitude = position.latitude
            self.__init_longitude = position.longitude
            self.get_logger().info(f"Initial position in (latitude, longitude) = ({self.__init_latitude}, {self.__init_longitude})")

        
    def __predict_from_image(self, image, classes=CLASSES):
        probabilities = softmax(self.__classifier(self.__resize((self.__to_tensor(image)/255).unsqueeze(dim=0)).to(self.__device))[0]).detach().cpu().numpy()
        argmax = probabilities.argmax()
        pred_class = classes[argmax]
        confidence = probabilities[argmax].item()
        return {
            "class": pred_class,
            "confidence": confidence
        }


    def __ignore_command(self, gesture_command:str) -> bool:
        if self.__previous_command is None or self.__previous_command != gesture_command:
            self.__previous_command = gesture_command
            self.__counter_command = 1
            return False
        # previous command = current
        self.__counter_command += 1
        if self.__counter_command < MIN_OCCURS:
            return False
        # previous = current and it occured many times succesively
        elif self.__counter_command == int(MIN_OCCURS):
            return True
        # previous = current and the action has already been called 
        else:
            return False

    def __action_calls(self, gesture_command:str, **args): # args in mm
        
        # https://asantamarianavarro.gitlab.io/code/projects/triffid/aurops/sections/triffid/ugv_planning.html#gesture-commander
        
        if self.__ignore_command(gesture_command):
            return

        if gesture_command == "come-to-me":
            msg = NavigateTo.Goal()
            msg.goal_pose = Pose() # map frame
            msg.goal_pose.position.x = float(args["x"]) / 1000 # convert to meters
            msg.goal_pose.position.y = float(args["y"]) / 1000
            msg.goal_pose.position.z = float(args["z"]) / 1000
            msg.goal_pose.orientation.x = float(args["q0"])
            msg.goal_pose.orientation.y = float(args["q1"])
            msg.goal_pose.orientation.z = float(args["q2"])
            msg.goal_pose.orientation.w = float(args["q3"])
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__navigation.wait_for_server()
            self.__navigation.send_goal_async(msg)
        
        elif gesture_command == "unfreeze": # Previously named "ok-to-go"
            msg = Trigger.Goal()
            msg.activate = False
            if not NO_UNDERLYING_IMPL:
                self.__freeze.wait_for_server()
            self.__freeze.send_goal_async(msg)
        
        elif gesture_command == "move-away-from-here":
            msg = Trigger.Goal()
            msg.activate = True
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__retreat.wait_for_server()
            self.__retreat.send_goal_async(msg)
        
        elif gesture_command == "operation-finished":
            msg = ReturnToBase.Goal()
            msg.activate = True
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__return_bos.wait_for_server()
            self.__return_bos.send_goal_async(msg)
        
        elif gesture_command == "freeze":
            msg = Trigger.Goal()
            msg.activate = True
            if not NO_UNDERLYING_IMPL:
                self.__freeze.wait_for_server()
            self.__freeze.send_goal_async(msg)
        
        elif gesture_command == "stop":
            msg = Trigger.Goal()
            msg.activate = True
            if not NO_UNDERLYING_IMPL:
                self.__stop.wait_for_server()
            self.__stop.send_goal_async(msg)

        elif gesture_command == "emergency-situation":
            msg = Trigger.Goal()
            msg.activate = True
            if not NO_UNDERLYING_IMPL:
                self.__emergency.wait_for_server()
            self.__emergency.send_goal_async(msg)

        elif gesture_command == "i-need-help":
            msg = HelpRequest.Goal()
            msg.target_transform = Transform()
            msg.target_transform.translation.x = float(args["x"]) / 1000
            msg.target_transform.translation.y = float(args["y"]) / 1000
            msg.target_transform.translation.z = float(args["z"]) / 1000
            msg.target_transform.rotation.x = float(args["q0"])
            msg.target_transform.rotation.y = float(args["q1"])
            msg.target_transform.rotation.z = float(args["q2"])
            msg.target_transform.rotation.w = float(args["q3"])
            msg.help_type = "aids"
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__help.wait_for_server()
            self.__help.send_goal_async(msg)
        
        elif gesture_command == "evacuate-the-area": # TODO: map this command to an action
            msg = ReturnToBase.Goal()
            msg.activate = True
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__return_bos.wait_for_server()
            self.__return_bos.send_goal_async(msg)
        
        elif gesture_command == "i-lost-connection":
            msg = HelpRequest.Goal()
            msg.target_transform = Transform()
            msg.target_transform.translation.x = float(args["x"]) / 1000
            msg.target_transform.translation.y = float(args["y"]) / 1000
            msg.target_transform.translation.z = float(args["z"]) / 1000
            msg.target_transform.rotation.x = float(args["q0"])
            msg.target_transform.rotation.y = float(args["q1"])
            msg.target_transform.rotation.z = float(args["q2"])
            msg.target_transform.rotation.w = float(args["q3"])
            msg.help_type = "technical"
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__help.wait_for_server()
            self.__help.send_goal_async(msg)
        
        elif gesture_command == "fetch-a-gas-mask":
            msg = ReturnToBaseFetch.Goal()
            msg.activate = True
            msg.object = "gas_mask"
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:    
                self.__fetch.wait_for_server()
            self.__fetch.send_goal_async(msg)
        
        elif gesture_command == "fetch-a-shovel":
            msg = ReturnToBaseFetch.Goal()
            msg.activate = True
            msg.object = "shovel"
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:
                self.__fetch.wait_for_server()
            self.__fetch.send_goal_async(msg)
        
        elif gesture_command == "fetch-an-axe":
            msg = ReturnToBaseFetch.Goal()
            msg.activate = True
            msg.object = "axe"
            msg.timeout = -1.0
            if not NO_UNDERLYING_IMPL:    
                self.__fetch.wait_for_server()
            self.__fetch.send_goal_async(msg)

        else:
            self.get_logger().error(f"Unknown command: {gesture_command}")


    def __main_callback(self, color_image:Image, depth_map:Image, intrinsics:CameraInfo, global_position:NavSatFix):
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


        if self.__last is None: # first frame
            self.__last = time.time()
        else:
            current = time.time()
            approx_fps = 1 / (current - self.__last) # 1/sec
            if approx_fps > MAX_FPS:
                self.get_logger().info(f"Omit: {approx_fps:.2f} > {MAX_FPS:.2f} FPS")
                return
            self.__last = current
            self.get_logger().info(f"[{self.__log_counter}] Current approximated FPS: {approx_fps:.2f}")

        self.__log_counter += 1

        try:

            # if color_image is None or depth_map is None or intrinsics is None or global_position is None or odometry is None or color_image.height != depth_map.height or color_image.width != depth_map.width:
            #     self.get_logger().error("Invalid input (e.g. RGBD frames dimensions may mismatch).")
            #     return

            self.__register_initial_gps(global_position)
            self.get_logger().info(f"[{self.__log_counter}] Received RGBD frames of size {color_image.height} x {color_image.width} (H x W) at {self.__gps_to_abs_xy(lat=global_position.latitude, lon=global_position.longitude)} (mm) ((0,0) is the initial)")
            
            color_image_array = np.asarray(color_image.data, dtype=np.float32).reshape((color_image.height, color_image.width, 3)) # H x W x 3
            depth_map_array = cv2.resize(np.asarray(np.frombuffer(depth_map.data,dtype=np.uint16), dtype=np.float32),dsize=(color_image.width, color_image.height)).reshape((color_image.height, color_image.width, 1)) # H x W x 1
            
            all_keypoints = self.__detect_keypoints(image=color_image_array, depth_map=depth_map_array)
            self.get_logger().info(f"[{self.__log_counter}] {len(all_keypoints)} detected humans")

            if len(all_keypoints) == 0:
                # self.get_logger().info("No detected person")
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
            
            assert argmin_u is not None
            
            if min_depth > self.__depth_threshold:
                self.get_logger().warn(f"[{self.__log_counter}] Distance from camera ({min_depth}) exceeds threshold ({self.__depth_threshold}) (mm)")
                return
            
            rel_xyz = self.__uvd_to_rel_xyz(u=argmin_u,v=argmin_v,depth=min_depth,intrinsics=np.asarray(intrinsics.k).reshape((3,3)))
            base_xyz = self.__rel_xyz_to_base_xyz(rel_xyz,color_image.header.stamp)
            abs_xyz = self.__base_xyz_to_abs_xyz(base_xyz,color_image.header.stamp)
            gps = self.__abs_xy_to_gps(x=abs_xyz[0],y=abs_xyz[1]) # lon, lat
            
            prediction = self.__predict_from_image(color_image_array)
            
            self.get_logger().info(f"[{self.__log_counter}] Detection position: {gps} (GPS) [or ({self.__gps_to_abs_xy(lat=gps[1],lon=gps[0])}) (xy in mm)] Depth: {min_depth} (mm) Class: {prediction['class']} Confidence: {prediction['confidence']}")

            if prediction["confidence"] < self.__classification_threshold:
                self.get_logger().warn(f"[{self.__log_counter}] Low confidence.")
                return

            self.get_logger().info(f"[{self.__log_counter}] High confidence, actions are triggered.")
            
            self.__action_calls(prediction["class"], x=abs_xyz[0], y=abs_xyz[1], z=abs_xyz[2], q0=0, q1=0, q2=0, q3=1)

            self.__publisher.publish(String(data=json.dumps({
                "type": "FeatureCollection",
                "features":[
                    {
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": list(gps)
                        },
                        "properties": {
                            "class":prediction["class"],
                            "confidence":prediction["confidence"],
                            "depth":min_depth,
                            "id":self.__counter,
                            "timestamp":self.get_clock().now().nanoseconds,
                            "keypoints_and_depths": all_keypoints[argmin_idx],
                            "camera_frame_position": {
                                "rel_x":rel_xyz[0],
                                "rel_y":rel_xyz[1],
                                "rel_z":rel_xyz[2]
                            }
                        }
                    }
                ]
            })))
            self.__counter += 1
        
        except BaseException as e:
            self.get_logger().error(f"[{self.__log_counter}] Error occured: {e}")


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Gesture_Classifier())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
