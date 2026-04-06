### Gesture Recognition Module (T4.3)

**Harokopio University of Athens**

**Contact info: kfoteinos@hua.gr**

### General information

This module takes as input aligned RGBD frames and the (global) pose of the camera (i.e. orientation and 2D position) at that timestamp (approximately) and performs gesture classication and pose estimation simultaneously; if a gesture is detected with high confidence and sufficiently close to the camera, it publishes, for a particular human considered to be the signer (e.g. the closest one), the pixel coordinates (uv) of her/his keypoints (e.g. ankles, shoulders), their depth and estimation confidence. Further, it estimates the uv position and depth of his/her by getting the average of the uv and depth correspond to the two shoulders. This information is utilized to predict the (global) longitude and latitude coordinates. Robot actions are triggered.

Transitions between coordinate systems:
- uvd: uv (image plane) & depth
- rel_xyz: (optical) camera frame xyz (backprojection) (camera_depth_frame)
- base_xyz: base link
- gps: GPS or global or absolute (longitude, latitude)
- abs_xy: Absolute xy (i.e. the "tangent" plane, oriented by the meridians and parallels)

> ***TODO: Integrate with UPC***

`docker exec -it <container ID> bash -c "source /opt/ros/humble/setup.bash;source ./install/local_setup.bash;ros2 run gesture_recognition producer"`

### Provided interface

| Topic name | Message type | Usage | Details |
| --- | --- | --- | --- |
| /camera_front/color | Image | Input | 8-bit RGB (H x W x 3) |
| /camera_front/depth | Image | Input | 16UC1 in mm (H x W x 2) aligned to the RGB |
| /camera_front/camera_info | CameraInfo | Input | - |
| /fix | NavSatFix | Input | - |
<!-- | /dog_odom | Odometry | Input | Orientation should be expresses wr.t. to a global coordinate system (the "standard" xy plane aligned to parallels and meridians) | -->
| /gesture_command | String | Output | Stringified GeoJSON, see below |


The exact message format has as follows:

```json
{
    "type": "FeatureCollection",
    "features":[
        {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates":      [<longitude>, <latitude>]
            },
            "properties": {
               "class":             <predicted class>,
               "confidence":        <confidence of the prediction (0-1)>,
               "depth":             <distance between the signer and the camera (mm)>,
               "id":                <serial number>,
               "timestamp":         <time in nanoseconds>,
               "keypoints_and_depths": {
                  "Nose":           [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Eye":       [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Eye":      [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Ear":       [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Ear":      [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Shoulder":  [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Shoulder": [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Elbow":     [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Elbow":    [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Wrist":     [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Wrist":    [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Hip":       [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Hip":      [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Knee":      [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Knee":     [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Left Ankle":     [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>],
                  "Right Ankle":    [<u (pixels)>, <v (pixels)>, <confidence (0-1)>, <depth (mm)>]
               },
               "relative_position": <relative position (mm)>
            }
        }
    ]
}
```

> Zero depth means u or v exceeds the limits of the depth frame, i.e. the keypoint falls out the image

See also the example below, produced by the command `ros2 topic echo /gesture_command --once --full`:

```json
{
   "features" : [
      {
         "geometry" : {
            "coordinates" : [
               0.018801084661978,
               0
            ],
            "type" : "Point"
         },
         "properties" : {
            "class" : "emergency-situation",
            "confidence" : 0.816117346286774,
            "depth" : 1882.53333333333,
            "id" : 3,
            "keypoints_and_depths" : {
               "Left Ankle" : [
                  340,
                  480,
                  0.00234119477681816,
                  0
               ],
               "Left Ear" : [
                  303,
                  167,
                  0.855729699134827,
                  2044
               ],
               "Left Elbow" : [
                  358,
                  152,
                  0.996827900409698,
                  1910
               ],
               "Left Eye" : [
                  289,
                  156,
                  0.991767883300781,
                  1949
               ],
               "Left Hip" : [
                  316,
                  372,
                  0.991478025913239,
                  1727
               ],
               "Left Knee" : [
                  333,
                  479,
                  0.241897374391556,
                  1727
               ],
               "Left Shoulder" : [
                  321,
                  214,
                  0.996646463871002,
                  1960
               ],
               "Left Wrist" : [
                  311,
                  90,
                  0.991696298122406,
                  1886
               ],
               "Nose" : [
                  280,
                  164,
                  0.991436660289764,
                  1935
               ],
               "Right Ankle" : [
                  234,
                  480,
                  0.00165498582646251,
                  0
               ],
               "Right Ear" : [
                  260,
                  166,
                  0.861356258392334,
                  1975
               ],
               "Right Elbow" : [
                  208,
                  154,
                  0.98670756816864,
                  1903
               ],
               "Right Eye" : [
                  272,
                  156,
                  0.994033992290497,
                  1953
               ],
               "Right Hip" : [
                  257,
                  369,
                  0.992672383785248,
                  1659
               ],
               "Right Knee" : [
                  249,
                  473,
                  0.301383495330811,
                  1736
               ],
               "Right Shoulder" : [
                  242,
                  209,
                  0.996383309364319,
                  1971
               ],
               "Right Wrist" : [
                  242,
                  104,
                  0.979750037193298,
                  1903
               ]
            },
            "relative_position" : [
               -1344.41828444444,
               -495.470444444444,
               1881.53333333333
            ],
            "timestamp" : 1773571893108678133
         },
         "type" : "Feature"
      }
   ],
   "type" : "FeatureCollection"
}

```

**Important:** No messages are produced if the pose estimator fails and/or the gesture prediction confidence is less than a threshold.


### Instructions for setup (Docker)

To build the image:
```bash
docker build -t gesture_container .
```

> To view the existing images:
> ```bash
> docker images -a
> ```
>
> To remove an image:
> ```bash
> docker rmi gesture_container
> ```

To create & enter the container (the `--net host` is mandatory to enable access to outside topics and `--gpus all` for accessing the available GPUs):
```bash
docker run --net host --gpus all -it gesture_container /bin/bash
```

To view the running containers:
```bash
docker container ps -a
```

> To remove all containers:
> ```bash
> docker container prune
> ```

To enter the container:
```bash
docker exec -it <container ID> bash
```

<!-- ### Instructions for playing the ROS2 bag:

Extract everything from the .zip file and run:
```bash
ros2 bag play <db3 file>
```
-->

### Instructions for setup (ROS)

After entering the container:

Activate ROS (Humble) (zsh, bash, ... according to your terminal):
```bash
source /opt/ros/humble/setup.bash
```

Clone, build and install the UPC interface: https://gitlab.com/asantamarianavarro/code/projects/triffid/robal_interfaces.

> Use `docker cp` to copy the folder within the container

Build the package:
```bash
colcon build --packages-select gesture_recognition
```

Run the following before use the package:
```bash
source ./install/local_setup.bash
```

Run the classification node:

```bash
ros2 run gesture_recognition gesture_classifier
```

Run the stub producer:

```bash
ros2 run gesture_recognition producer
```

View the detections topic:
```bash
ros2 topic echo gesture_command --once --full
```