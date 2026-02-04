from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    left_gain = LaunchConfiguration("left_motor_gain")
    right_gain = LaunchConfiguration("right_motor_gain")
    distance_scale = LaunchConfiguration("distance_scale_factor")
    turn_min_pwm = LaunchConfiguration("turn_min_pwm_duty")

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
            "distance_scale_factor",
            default_value="3.1",
            description="Calibration scale for travel distance (increase to travel further, must be > 0.0).",
        ),
        DeclareLaunchArgument(
            "turn_min_pwm_duty",
            default_value="0.5",
            description="Minimum PWM duty (0-1) to overcome stiction when turning in place.",
        ),
        Node(
            package="jetbot_motors",
            executable="jetbot_motors_node",
            name="jetbot_motors",
            output="screen",
            parameters=[{
                "left_motor_gain": left_gain,
                "right_motor_gain": right_gain,
                "distance_scale_factor": distance_scale,
                "turn_min_pwm_duty": turn_min_pwm,
            }],
        ),
    ])
