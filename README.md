ros2 run my_package node_executable --ros-args ...

Every command requires sudo:

To build the image
sudo docker build -t gesture_container .

To create & enter the container
sudo docker run -it gesture_container /bin/bash

To see the running containers
sudo docker container ps -a

To see the existing images:
sudo docker images -a

To remove an image:
sudo docker rmi gesture_container

To enter the container
sudo docker exec -it &lt;container ID&gt; bash

Build the package
colcon build --packages-select gesture_recognition

Run the following to anounce the package
source ./install/local_setup.zsh

Run the three nodes:
ros2 run gesture_recognition classifier
ros2 run gesture_recognition stub_consumer
ros2 run gesture_recognition stub_producer