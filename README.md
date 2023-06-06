# waypoint_navigation

Waypoint navigation for ROS2

### Installation

```shell
$ cd ~/ros2_ws/src
$ git clone <<url>>
$ cd ~/ros2_ws
$ colcon build
```

## Usage

### Launch

```shell
$ ros2 launch waypoint_navigation waypoint_navigation.launch.py
```

### Shell Examples

```shell
$ ros2 service call /waypoint_navigation/add_wp waypoint_navigation_interfaces/srv/AddWp "{'wp':{'id':'kitchen', 'pose':{'position':{'x':3.79, 'y':6.77, 'z':0.0}, 'orientation':{'x':0.0, 'y':0.0, 'z':0.99, 'w':0.12}}}}"
$ ros2 service call /waypoint_navigation/get_wp waypoint_navigation_interfaces/srv/GetWp "{'wp_id':'kitchen'}"
$ ros2 service call /waypoint_navigation/get_wps waypoint_navigation_interfaces/srv/GetWps {}
$ ros2 action send_goal /waypoint_navigation/navigate_to_wp waypoint_navigation_interfaces/action/NavigateToWp "{'wp_id':'kitchen'}"
```
