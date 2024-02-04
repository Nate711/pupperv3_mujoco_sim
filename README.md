# Prereqs
Clone "pupper_interfaces" package into your ROS2 workspace src folder

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