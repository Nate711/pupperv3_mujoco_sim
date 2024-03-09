from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("pupper_v3_description")
    return LaunchDescription(
        [
            Node(
                package="pupper_mujoco_sim",
                executable="mujoco_interactive_test",
                name="mujoco_interactive_test",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "publish_rate": 500.0,
                        "model_xml": share_dir
                        + "/description/mujoco_xml/pupper_v3_complete.mjx.fixed_base.xml",
                        "timestep": 0.001,
                    }
                ],
            )
        ]
    )
