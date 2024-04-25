# Prereqs
* Clone "pupper_interfaces" package (https://github.com/Nate711/pupper_interfaces) into your ROS2 workspace src folder
* Install mujoco v2.3.1

# Install
Clone this repo into your ROS2 workspace src folder
```bash
source /opt/ros/humble/setup.bash
cd ${YOUR_WORKSPACE_FOLDER}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source /install/local_setup.bash
```

# Usage
Using the simulator as a hardware interface
```bash
ros2 launch pupper_mujoco_sim test_hw_interface.launch.py
```

Using the simulator as a regular ros node with subscription to joint commands
```bash
ros2 launch pupper_mujoco_sim floating_base_launch.py 
```

# TODO / Limitations
* Make mujoco_core_interactive.hpp load model xml based on package and model parameter rather than hardcode to `pupper_v3_description/description/mujoco_xml/pupper_v3_complete.xml`
* Make hardware interface set model package and xml file based on parameter 