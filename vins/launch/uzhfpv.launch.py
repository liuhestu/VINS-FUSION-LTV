#!/usr/bin/env python3

"""Launch VINS-Fusion and play one UZH-FPV ROS 2 bag."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, Shutdown,
                            TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SEQUENCE_CONFIGS = (
    ("indoor_forward_", "uzhfpv_indoor"),
    ("indoor_45_", "uzhfpv_indoor_45"),
    ("outdoor_forward_", "uzhfpv_outdoor"),
    ("outdoor_45_", "uzhfpv_outdoor_45"),
)


def classify_sequence(sequence):
    matches = [category for prefix, category in SEQUENCE_CONFIGS
               if sequence.startswith(prefix)]
    if len(matches) != 1:
        supported = ", ".join(prefix + "*" for prefix, _ in SEQUENCE_CONFIGS)
        raise RuntimeError(
            f"unsupported UZH-FPV sequence '{sequence}'; expected one of: {supported}")
    return matches[0]


def _launch_setup(context):
    sequence = LaunchConfiguration("sequence").perform(context)
    dataset_root = os.path.abspath(
        os.path.expanduser(LaunchConfiguration("dataset_root").perform(context)))
    rate_text = LaunchConfiguration("rate").perform(context)
    delay_text = LaunchConfiguration("start_delay").perform(context)

    if not sequence or os.path.basename(sequence) != sequence:
        raise RuntimeError("sequence must be a non-empty directory name")
    try:
        rate = float(rate_text)
        start_delay = float(delay_text)
    except ValueError as error:
        raise RuntimeError("rate and start_delay must be numbers") from error
    if rate <= 0.0:
        raise RuntimeError("rate must be greater than zero")
    if start_delay < 0.0:
        raise RuntimeError("start_delay must not be negative")

    category = classify_sequence(sequence)
    bag_path = os.path.join(dataset_root, category, sequence)
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.isdir(bag_path):
        raise RuntimeError(f"UZH-FPV bag directory does not exist: {bag_path}")
    if not os.path.isfile(metadata_path):
        raise RuntimeError(f"ROS 2 bag metadata does not exist: {metadata_path}")

    package_share = get_package_share_directory("vins")
    config_path = os.path.join(
        package_share, "config", category, "uzhfpv_stereo_imu_config.yaml")
    if not os.path.isfile(config_path):
        raise RuntimeError(f"VINS configuration does not exist: {config_path}")

    # This matches output_path in all four UZH-FPV configurations.
    os.makedirs("/home/he/output/uzhfpv", exist_ok=True)

    vins_node = Node(
        package="vins",
        executable="vins_node",
        name="vins_estimator",
        output="screen",
        arguments=[config_path],
        remappings=[
            ("imu_propagate", "/vins_estimator/imu_propagate"),
            ("path", "/vins_estimator/path"),
            ("odometry", "/vins_estimator/odometry"),
            ("point_cloud", "/vins_estimator/point_cloud"),
            ("margin_cloud", "/vins_estimator/margin_cloud"),
            ("key_poses", "/vins_estimator/key_poses"),
            ("camera_pose", "/vins_estimator/camera_pose"),
            ("camera_pose_visual", "/vins_estimator/camera_pose_visual"),
            ("keyframe_pose", "/vins_estimator/keyframe_pose"),
            ("keyframe_point", "/vins_estimator/keyframe_point"),
            ("extrinsic", "/vins_estimator/extrinsic"),
            ("image_track", "/vins_estimator/image_track"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(package_share, "config", "vins_rviz_config.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    bag_player = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--rate", rate_text],
        name="uzhfpv_bag_player",
        output="screen",
    )
    delayed_bag_player = TimerAction(period=start_delay, actions=[bag_player])

    # Give subscriptions time to drain after ros2 bag play has returned, then
    # stop the estimator and RViz so the launch command terminates on its own.
    bag_finished = RegisterEventHandler(OnProcessExit(
        target_action=bag_player,
        on_exit=[TimerAction(
            period=2.0,
            actions=[Shutdown(reason="UZH-FPV bag playback finished")],
        )],
    ))
    estimator_finished = RegisterEventHandler(OnProcessExit(
        target_action=vins_node,
        on_exit=[Shutdown(reason="VINS estimator exited")],
    ))
    return [vins_node, rviz_node, delayed_bag_player,
            bag_finished, estimator_finished]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sequence",
            description="UZH-FPV sequence directory name (required)"),
        DeclareLaunchArgument(
            "dataset_root",
            default_value="/home/he/datasets/uzhfpv",
            description="Root containing UZH-FPV category directories"),
        DeclareLaunchArgument(
            "rate", default_value="1.0",
            description="ros2 bag playback rate"),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="Start RViz with the VINS configuration"),
        DeclareLaunchArgument(
            "start_delay", default_value="2.0",
            description="Seconds to wait before starting bag playback"),
        OpaqueFunction(function=_launch_setup),
    ])
