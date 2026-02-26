### Gesture Recognition Module (T4.3)

**Foteinos Konstantinos (kfoteinos@hua.gr)**

The image needs 1.26GB.

### General information:

Input: Messages of type `Image` published on the topic `/camera_front/raw_image`.

Output: Messages of type `String` containing a GeoJSON published on the topic `gesture_command`.

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
                "depth": <estimated distance between the signer and the camera>
            }
        }
    ]
}
```

**Important:** Works only for one person. If the pose estimator doesn't detect any person, the node doesn't publish any message. 

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

Run the three nodes (in different terminals):

```bash
ros2 run gesture_recognition classifier
ros2 run gesture_recognition stub_consumer
```
