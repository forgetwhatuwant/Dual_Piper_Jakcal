from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_piper", package_name="dual_piper_moviet_3")
        .ros2_control()
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
