from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
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
            }],
        ),

        Node(
            package="jetbot_motors",
            executable="jetbot_motors_node",
            name="jetbot_motors",
            output="screen",
            parameters=[
            ],
        ),
    ])
