## To launch the driver
```
ros2 launch spot_driver spot_driver.launch.py
```

By default, startup force-takes Spot's body lease from the current owner. To avoid force-taking the lease:

```
ros2 launch spot_driver spot_driver.launch.py take_lease:=false
```

```
ros2 run tf2_ros static_transform_publisher -0.10795 0.0 0.1397 -1.57 -1.57 0 world wall
```

```
ros2 service call /get_fiducial_transform spot_srvs/srv/GetTransform "{fiducial_name: 'wall'}"
```

```
ros2 run map_localization map_localizer_node
```

```
ros2 run nav_goal_listener nav_goal_listener
```

## Camera topics

The driver publishes body fisheye cameras as JPEG `sensor_msgs/CompressedImage` by default:

```
/camera/frontleft_fisheye/image/compressed
/camera/frontleft_fisheye/camera_info
/camera/frontright_fisheye/image/compressed
/camera/frontright_fisheye/camera_info
```

On Spot variants with an arm/gripper, enable the gripper camera. The driver opens the gripper for camera capture,
publishes JPEG plus dynamic camera TF, then closes the gripper during shutdown:

```
/camera/hand_color/image/compressed
/camera/hand_color/camera_info
```

The gripper camera defaults to disabled. Enable it with default 1920x1080 resolution and 10 Hz rate:

```
ros2 launch spot_driver spot_driver.launch.py gripper_camera:=true
```

## To control spot
```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```

```
ros2 action send_goal /move_relative_xy spot_action/action/MoveRelativeXY "{x: 1.0, y: 0.0, yaw: 90.0}"
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Setup the docker container on Spot CORE I/O
### From local machine
```
docker build -t spot_coreio_ros2 --platform linux/arm64 -f Dockerfile.l4t .
docker save spot_coreio_ros2:latest | pigz > spot_coreio_ros2.tgz
scp -r -P 20022 spot_coreio_ros2.tgz spot@192.168.80.3:/home/spot/
```
### From CORE I/O
```
sudo docker load -i spot_coreio_ros2.tgz
```