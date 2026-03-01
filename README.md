### Gesture Recognition Module (T4.3)

**Foteinos Konstantinos (kfoteinos@hua.gr)**

The image needs 2.73GB.

### General information:

Input: Messages of type `Image` published on the topic `/camera_front/raw_image`, `CameraInfo` published on `/camera_front/camera_info`, `NavSatFix` published on `/fix` and `Odometry` on `/dog_odom`.

Output: Messages of type `String` containing a GeoJSON published on the topic `/gesture_command`.

The schema of the exported GeoJSON has as follows:

```json
{
    "type": "FeatureCollection",
    "features":[
        {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [<longitude>, <latitude>]
            },
            "properties": {
                "class": <predicted class>,
                "confidence": <confidence of the prediction>,
                "depth": <estimated distance between the signer and the camera>,
                "id": <serial number>,
                "timestamp": <time in nanoseconds>,
                "keypoints_and_depths": [
                    {
                        "Nose": [<u>, <v>],
                        "Left Eye": [<u>, <v>],
                        "Right Eye": [<u>, <v>],
                        "Left Ear": [<u>, <v>],
                        "Right Ear": [<u>, <v>],
                        "Left Shoulder": [<u>, <v>],
                        "Right Shoulder": [<u>, <v>],
                        "Left Elbow": [<u>, <v>],
                        "Right Elbow": [<u>, <v>],
                        "Left Wrist": [<u>, <v>],
                        "Right Wrist": [<u>, <v>],
                        "Left Hip": [<u>, <v>],
                        "Right Hip": [<u>, <v>],
                        "Left Knee": [<u>, <v>],
                        "Right Knee": [<u>, <v>],
                        "Left Ankle": [<u>, <v>],
                        "Right Ankle": [<u>, <v>],
                        "depth": <depth value>
                    },
                    ...
                ],
                "relative_position": <relative position>
            }
        }
    ]
}
```

**Important:** If the pose estimator doesn't detect any person, the node doesn't publish any message. No GPU support.

ROS BAG: https://drive.google.com/file/d/1Sw_Gl6oxveW6lPsleHsdZHQrkLQjHBT3/view?usp=sharing

### Instructions for docker:

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

To create & enter the container (the `--net host` is mandatory to enable access to outside topics):
```bash
sudo docker run --net host -it gesture_container /bin/bash
```

To view the running containers:
```bash
sudo docker container ps -a
```

> To remove all containers:
> ```bash
> sudo docker container prune
> ```

To enter the container:
```bash
sudo docker exec -it <container ID> bash
```

### Instructions for playing the ROS2 bag:

Extract everything from the .zip file and run:
```bash
ros2 bag play <db3 file>
```
This ROS bag should have a `/camera_front/raw_image` topic, which produces messages of type `Image`.

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

Run the node:

```bash
ros2 run gesture_recognition gesture_classifier
```

View the detections topic:
```bash
ros2 topic echo gesture_command
```

### TODOs:

1. Handle multiple humans (>1)
2. Determine relative position
3. Transform it to absolute
