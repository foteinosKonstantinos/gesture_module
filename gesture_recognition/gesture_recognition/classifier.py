import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import onnxruntime as ort

# Deployment details
ONNX_MODEL = "/app/gesture_recognition/gesture_recognition/efficientnet.onnx"

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

class Classification_Node(Node):

    def __init__(self, onnx_path:str, classes:list[str]=CLASSES):
        super().__init__("classification_node")
        self.__onnx_path = onnx_path
        self.__classes = classes
        self.create_subscription(
            msg_type=String,
            topic="rgb_stream",
            callback=self.prediction_callback,
            qos_profile=10
        )
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = "gesture_command",
            qos_profile = 10
        )
        self.__session = ort.InferenceSession(onnx_path)
        self.get_logger().info(f"Successfully initialized the classification node, with weights {self.__onnx_path} for {len(self.__classes)} classes.")

    def prediction_callback(self,in_data:String): # 3 (=RGB channels) x 480 (=H) x 640 (=W)
        in_array = np.asarray(json.loads(in_data.data), dtype=np.float32)
        self.get_logger().info("recieved "+str(in_array.shape))
        probs = self.__session.run(None, {"input":in_array.reshape((1,3,480,640))})
        pred_class = self.__classes[np.asarray(probs[0]).argmax()]
        self.__publisher.publish(String(data=json.dumps({
            "class": pred_class,
            "probabilities": {
                self.__classes[i]:probs[0][i].item() for i in range(len(self.__classes))
            }
        })))
        self.get_logger().info(f"published \'{pred_class}\' and per-classes probavilities")

def main():
    try:
        rclpy.init()
        rclpy.spin(node=Classification_Node(onnx_path=ONNX_MODEL))
    except (ExternalShutdownException, KeyboardInterrupt):
        pass

if __name__ == '__main__':
    main()