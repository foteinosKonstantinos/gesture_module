source /opt/ros/humble/setup.bash

echo "Removing build trash ..."
rm -rf build install log

echo "Setup UPC interface ..."
source /home/triffid/upc_ws/install/setup.bash

echo "Building gesture_recognition package ..."
colcon build --packages-select gesture_recognition

echo "Installing package ..."
source ./install/local_setup.bash

echo "Running the classification node ..."
ros2 run gesture_recognition gesture_classifier