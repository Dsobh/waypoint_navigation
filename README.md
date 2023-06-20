# waypoint_navigation

Waypoint navigation for ROS2

### Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/MERLIN2-ARCH/waypoint_navigation.git
$ cd ~/ros2_ws
$ colcon build
```

## Usage

Run simulator:

```shell
ros2 launch rb1_sandbox hospital.launch.py 
```

Run action service:

```shell
ros2 launch waypoint_navigation waypoint_navigation_hospital.launch.py 
```

Run action client node:

```shell
ros2 run waypoint_navigation waypoint_navigation_hospital_node 
```



