from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description() -> LaunchDescription:
    cwd = os.getcwd()
    left_gain = LaunchConfiguration("left_motor_gain")
    right_gain = LaunchConfiguration("right_motor_gain")
    distance_scale = LaunchConfiguration("distance_scale_factor")

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
            default_value="2.7",
            description="Calibration scale for travel distance (increase to travel further, must be > 0.0).",
        ),
        Node(
            package="gscam",
            executable="gscam_node",
            name="camera",
            output="screen",
            parameters=[{
                "gscam_config": (
                    "nvarguscamerasrc sensor-id=0 ! "
                    "video/x-raw(memory:NVMM),width=1280,height=720 ! "
                    "nvvidconv ! "
                    "videorate ! "
                    "video/x-raw,framerate=30/1,format=BGRx ! "
                    "videoconvert ! "
                    "video/x-raw,format=BGR"
                ),
                "use_sensor_data_qos": True,
                "camera_info_url": f"file://{cwd}/ost.yaml", # Note launch from top level of repo
            }],
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
            }],
        ),

        Node(
            package="image_rectifier",
            executable="image_rectifier_node",
            name="image_rectifier",
            output="screen",
            parameters=[{
                "input_image_topic": "/camera/image_raw",
                "input_camera_info_topic": "/camera/camera_info",
                "output_image_topic": "/camera/image_rect",
                "output_camera_info_topic": "/camera/camera_info_rect",
                "alpha": 0.5,
                "interpolate": True,
            }],
        ),
    ])
