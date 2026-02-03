from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for image rectifier node."""
    
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/camera/image_raw',
        description='Input raw image topic'
    )
    
    input_camera_info_topic_arg = DeclareLaunchArgument(
        'input_camera_info_topic',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )
    
    output_image_topic_arg = DeclareLaunchArgument(
        'output_image_topic',
        default_value='/camera/image_rect',
        description='Output rectified image topic'
    )
    
    output_camera_info_topic_arg = DeclareLaunchArgument(
        'output_camera_info_topic',
        default_value='/camera/camera_info_rect',
        description='Output rectified camera info topic'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.0',
        description='Free scaling parameter (0.0 = keep all valid pixels, 1.0 = keep all source pixels)'
    )
    
    interpolate_arg = DeclareLaunchArgument(
        'interpolate',
        default_value='true',
        description='Enable interpolation during rectification'
    )
    
    image_rectifier_node = Node(
        package='image_rectifier',
        executable='image_rectifier_node',
        name='image_rectifier',
        output='screen',
        parameters=[{
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'input_camera_info_topic': LaunchConfiguration('input_camera_info_topic'),
            'output_image_topic': LaunchConfiguration('output_image_topic'),
            'output_camera_info_topic': LaunchConfiguration('output_camera_info_topic'),
            'alpha': LaunchConfiguration('alpha'),
            'interpolate': LaunchConfiguration('interpolate'),
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('input_image_topic')),
            ('/camera/camera_info', LaunchConfiguration('input_camera_info_topic')),
            ('/camera/image_rect', LaunchConfiguration('output_image_topic')),
            ('/camera/camera_info_rect', LaunchConfiguration('output_camera_info_topic')),
        ]
    )
    
    return LaunchDescription([
        input_image_topic_arg,
        input_camera_info_topic_arg,
        output_image_topic_arg,
        output_camera_info_topic_arg,
        alpha_arg,
        interpolate_arg,
        image_rectifier_node,
    ])
