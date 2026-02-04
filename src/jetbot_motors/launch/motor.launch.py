from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    left_gain = LaunchConfiguration("left_motor_gain")
    right_gain = LaunchConfiguration("right_motor_gain")
    velocity_scale = LaunchConfiguration("velocity_scale_factor")

    return LaunchDescription([
        DeclareLaunchArgument(
            "left_motor_gain",
            default_value="1.13",
            description="Gain multiplier for left motor PWM (must be > 0.0).",
        ),
        DeclareLaunchArgument(
            "right_motor_gain",
            default_value="1.0",
            description="Gain multiplier for right motor PWM (must be > 0.0).",
        ),
        DeclareLaunchArgument(
            "velocity_scale_factor",
            default_value="3.5",
            description="Calibration scale for velocity/distance (increase so it travels further, must be > 0.0).",
        ),
        Node(
            package="jetbot_motors",
            executable="jetbot_motors_node",
            name="jetbot_motors",
            output="screen",
            parameters=[{
                "left_motor_gain": left_gain,
                "right_motor_gain": right_gain,
                "velocity_scale_factor": velocity_scale,
            }],
        ),
    ])
