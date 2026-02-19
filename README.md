### Gesture Recognition Module (T4.3)

**Foteinos Konstantinos (kfoteinos@hua.gr)**

The image needs 1.26GB.

### Instructions for docker

To build the image:
```bash
sudo docker build -t gesture_container .
```

> To view the existing images:
> ```bash
> sudo docker images -a
> ```
>
> To remove an image:
> ```bash
> sudo docker rmi gesture_container
> ```

To create & enter the container:
```bash
sudo docker run -it gesture_container /bin/bash
```

To view the running containers:
```bash
sudo docker container ps -a
```

To enter the container:
```bash
sudo docker exec -it <container ID> bash
```

### Instructions for ROS:

After entering the container:

Activate ROS (Humble) (zsh, bash, ... according to your terminal):
```bash
source /opt/ros/humble/setup.bash
```

Build the package:
```bash
colcon build --packages-select gesture_recognition
```

Run the following before use the package:
```bash
source ./install/local_setup.bash
```

Run the three nodes (in different terminals):

```bash
ros2 run gesture_recognition classifier
ros2 run gesture_recognition stub_consumer
ros2 run gesture_recognition stub_producer
```