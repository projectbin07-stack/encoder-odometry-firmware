# ROS 2 bridge example

`odom_bridge_node.py` connects the firmware to ROS 2: `cmd_vel` in,
`odom` + `odom -> base_link` TF out.

**Status: example only.** CI lints this file with ruff but never runs it
(ROS 2 is not installed in CI), and it has not been run against real
hardware. Test it on your robot before relying on it.

```sh
source /opt/ros/<distro>/setup.bash
pip install -e ./host            # odobridge + pyserial
python3 host/examples/ros2/odom_bridge_node.py --ros-args -p port:=/dev/ttyUSB0
```

The node does not set covariances; fill `pose.covariance` /
`twist.covariance` for your robot if `robot_localization` or Nav2 consumes
the topic.
