# Prereqs
Clone "pupper_interfaces" package (https://github.com/Nate711/pupper_interfaces) into your ROS2 workspace src folder

# Install
Clone this repo into your ROS2 workspace src folder
```bash
source /opt/ros/humble/setup.bash
cd ${YOUR_WORKSPACE_FOLDER}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source /install/local_setup.bash
```

# Usage
```bash
ros2 launch pupper_mujoco_sim floating_base_launch.py 
```

# Updating the model
1. `source /install/local_setup.bash`
2. `ros2 run pupper_mujoco_sim simulate`
3. Load the urdf
4. Click convert to MJCF button to generate .xml model file
5. Update pupper_v3_2.xml with content from generated .xml
6. Reorder bodies in pupper_v3_2.xml to match actuator order (FR, FL, BR, BL)
