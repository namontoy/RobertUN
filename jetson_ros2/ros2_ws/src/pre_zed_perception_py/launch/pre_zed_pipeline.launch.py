"""Launch the complete local-video pre-ZED perception workflow."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory('pre_zed_perception_py')
    default_config = os.path.join(
        package_share, 'config', 'pre_zed_params.yaml'
    )
    config_file = LaunchConfiguration('config_file')
    video_path = LaunchConfiguration('video_path')
    output_video_path = LaunchConfiguration('output_video_path')

    common = {
        'output': 'screen',
        'parameters': [config_file],
    }
    video_node = Node(
        package='pre_zed_perception_py',
        executable='video_file_publisher',
        name='video_file_publisher',
        output='screen',
        parameters=[
            config_file,
            {
                'video_path': video_path,
                'loop': ParameterValue(
                    LaunchConfiguration('loop'), value_type=bool
                ),
                'sequential_mode': ParameterValue(
                    LaunchConfiguration('sequential_mode'), value_type=bool
                ),
                'frame_stride': ParameterValue(
                    LaunchConfiguration('frame_stride'), value_type=int
                ),
                'completion_timeout_seconds': ParameterValue(
                    LaunchConfiguration('completion_timeout_seconds'),
                    value_type=float,
                ),
            },
        ],
    )
    overlay_node = Node(
        package='pre_zed_perception_py',
        executable='overlay_node',
        name='pre_zed_overlay_node',
        output='screen',
        parameters=[
            config_file,
            {
                'output_video_path': output_video_path,
                'output_video_fps': ParameterValue(
                    LaunchConfiguration('output_video_fps'),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Absolute path to the pre-ZED ROS parameter file.',
        ),
        DeclareLaunchArgument(
            'video_path',
            default_value=(
                'datasets/depth_anything_v3_small/input_videos/'
                'car_video.mp4'
            ),
            description='Input video processed by the source node.',
        ),
        DeclareLaunchArgument(
            'output_video_path',
            default_value='final_preZED/outputs/pre_zed_overlay.mp4',
            description='Annotated MP4 written by the overlay node.',
        ),
        DeclareLaunchArgument(
            'output_video_fps',
            default_value='1.0',
            description='Playback FPS stored in the output MP4.',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Repeat the input video after its final frame.',
        ),
        DeclareLaunchArgument(
            'sequential_mode',
            default_value='false',
            description=(
                'Wait for each overlay before publishing the next frame.'
            ),
        ),
        DeclareLaunchArgument(
            'frame_stride',
            default_value='1',
            description='Process every Nth source frame.',
        ),
        DeclareLaunchArgument(
            'completion_timeout_seconds',
            default_value='150.0',
            description='Maximum wait for one frame to reach the overlay.',
        ),
        video_node,
        Node(
            package='pre_zed_perception_py',
            executable='depth_anything_http_node',
            name='depth_anything_http_node',
            **common,
        ),
        Node(
            package='pre_zed_perception_py',
            executable='yolo_http_detector_node',
            name='yolo_http_detector_node',
            **common,
        ),
        Node(
            package='pre_zed_tracking_cpp',
            executable='simple_tracker_node',
            name='simple_tracker_node',
            **common,
        ),
        Node(
            package='pre_zed_tracking_cpp',
            executable='track_depth_fusion_node',
            name='track_depth_fusion_node',
            **common,
        ),
        overlay_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=video_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='Video source process completed.'
                        )
                    )
                ],
            )
        ),
    ])
