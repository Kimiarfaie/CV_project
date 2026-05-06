"""
=====================================================================
 * MIT License
 * 
 * Copyright (c) 2025 Omni Instrument Inc.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===================================================================== 
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    Shutdown
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    home = os.path.expanduser("~")
    default_config_path = os.path.join(home, "ros2_ws", "src", "tsdf_saver", "config", "custom.yaml")
    default_bag_path = os.path.join(home, "dataset", "VIO_stripped")
    qos_yaml = os.path.join(home, "dataset", "qos_override.yaml")


    bag_path = LaunchConfiguration("bag")
    save_threshold = LaunchConfiguration("save_time_threshold")
    config_file_path = LaunchConfiguration("dbtsdf_config")

    # ==============================================================
    #  Stereo Depth
    # ==============================================================
    stereo_depth_node = Node(
        package="stereo_depth",
        executable="stereo_depth_node",
        name="stereo_depth_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"algorithm": "sgbm"},
        ],
        remappings=[
            ("left/image", "/zed/zedxm/left/gray/rect/image"),
            ("right/image", "/zed/zedxm/right/gray/rect/image"),
            ("left/camera_info", "/zed/zedxm/left/gray/rect/image/camera_info"),
            ("right/camera_info", "/zed/zedxm/right/gray/rect/image/camera_info"),
            ("depth/image", "/stereo_depth/depth"),
            ("depth/camera_info", "/stereo_depth/camera_info"),
        ],
    )

    # ==============================================================
    #  PointCloud Node (XYZ)
    # ==============================================================

    pointcloud_component = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzNode",
        name="depth_image_pointcloud",
        parameters=[
            {"use_sim_time": True},
            {"queue_size": 20}
        ],
        remappings=[
            ("/zed/zedxm/depth/camera_info", "/stereo_depth/camera_info"),
            ("image_rect", "/stereo_depth/depth"),
            ("points", "/stereo/points")
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ==============================================================
    #  TSDF Saver Component
    # ==============================================================

    tsdf_saver_component = ComposableNode(
        package="tsdf_saver",
        plugin="tsdf_saver::ExactTimeSaver",
        name="tsdf_saver",
        parameters=[
            {"use_sim_time": True},
            {"save_time_threshold": save_threshold}
        ],
        remappings=[
            ("cloud_in", "/stereo/points")
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ==============================================================
    #  DB-TSDF Node (Normal ROS2 Node)
    # ==============================================================

    db_tsdf_node = Node(
        package='db_tsdf',
        executable='db_tsdf_node',
        name='db_tsdf_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            config_file_path
        ],
        remappings=[
            ("cloud", "/tsdf/local_cloud")
        ]
    )

    # ==============================================================
    #  Container
    # ==============================================================

    container = ComposableNodeContainer(
        name="stereo_tsdf_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info'],
        output="screen",
        parameters=[{"use_sim_time": True}],
        composable_node_descriptions=[
            pointcloud_component,
            tsdf_saver_component,
        ]
    )

    # ==============================================================
    #  Bag Playback Process
    # ==============================================================

    bag_proc = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play",
            bag_path,
            "--clock",
            "--qos-profile-overrides-path", qos_yaml
        ],
        output="screen"
    )

    # ==============================================================
    #  Shutdown logic: 20 seconds after bag finishes
    # ==============================================================

    delayed_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_proc,
            on_exit=[
                TimerAction(
                    period=20.0,
                    actions=[
                        Shutdown(reason="Bag finished; shutting down after 20 seconds")
                    ]
                )
            ]
        )
    )

    # ==============================================================
    #  Launch Description
    # ==============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            "bag",
            default_value=default_bag_path,
            description="Path to rosbag directory"
        ),

        DeclareLaunchArgument(
            "save_time_threshold",
            default_value="1765058989.0",
            description="When pointcloud time exceeds this value, TSDF saver triggers"
        ),

        DeclareLaunchArgument(
            "dbtsdf_config",
            default_value=default_config_path,
            description="Path to DB-TSDF config directory"
        ),

        bag_proc,
        container,
        db_tsdf_node,
        delayed_shutdown
    ])