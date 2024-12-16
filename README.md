# Installation


## ros-jazzy-ur

```bash
sudo apt install ros-jazzy-ur-client-library
```

```bash
sudo apt-get install ros-jazzy-ur
```

### Robot Setup

```
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
Preferred DNS server: 192.168.1.1
Alternative DNS server: 0.0.0.0
```

> [!NOTE]
> *PC SETTINGS*
> IPv4\
> Manual\
> Address: 192.168.1.101\
> Netmask: 255.255.255.0\
> Gateway: 192.168.1.1


### Dependencies
- realsense2_camera ( [repo](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu))
- Eigen3 (```sudo apt install libeigen3-dev```)
- git clone https://gitlab.com/libeigen/eigen.git (do this in ros_ws. not inside src or ros_brics)
- moveit2 (```sudo apt install ros-jazzy-moveit```)
- nlohmann_json (```sudo apt install nlohmann-json3-dev```)


### Extract calliberation

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=192.168.1.102 target_filename:="${HOME}/my_robot_calibration.yaml"
```


## Launching the package

```bash
ros2 launch ros_brics brics.launch
```

> [!NOTE]
> 'headless_mode': 'true'. This param is important for the external control. If it's false, you have to press play button on the teach pendant to send commands from ROS

> [!NOTE]
> For troubleshooting, refer [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html#troubleshooting)

## To move the robot with Moveit2

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

### Building

```bash
colcon build --packages-select ros_brics
```



