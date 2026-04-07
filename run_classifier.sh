source /opt/ros/humble/setup.bash

echo "Build the interface"
mkdir robal_interfaces_pkg
mv robal_interfaces robal_interfaces_pkg/robal_interfaces
cd robal_interfaces_pkg
colcon build --packages-select robal_interfaces
source ./install/local_setup.bash
cd ..

echo "Build gesture recognition package"
colcon build --packages-select gesture_recognition
source ./install/local_setup.bash
echo "Run the classifier"
ros2 run gesture_recognition gesture_classifier