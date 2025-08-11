from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("synbio_env", package_name="synbio_moveit_config")
        .to_moveit_configs()
    )

    # Add trajectory_execution parameters for time parameterization
    extra_parameters = [{
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "moveit_manage_controllers": True,
            "trajectory_duration_monitoring": True,
            "trajectory_time_parameterization": {
                "enabled": True,
                "type": "IterativeParabolicTimeParameterization"
            }
        }
    }]

    return generate_move_group_launch(moveit_config)