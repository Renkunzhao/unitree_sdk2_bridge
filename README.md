# Description
According to [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2.git), [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) is compatible with ROS2 because both use DDS as the underlying communication protocol.

Therefore, running ‘ros2 topic list‘ should directly display the topics published by the real robot or the simulator.

However, I frequently encounter an empty topic list. This package is intended to explicitly forward the topics.

# Dependencies
- unitree_sdk2
- ros2
- unitree_ros2

# Build
colcon build --packages-select unitree_sdk2_bridge

# Usage
ros2 run unitree_sdk2_bridge unitree_sdk2_bridge_node

# TODO
- [x] LowState
- [ ] LowCmd
- [ ] SportModeState
- [ ] WirelessController