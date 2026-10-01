"""Launch the published Stage7 configuration with an independent output directory."""
from pathlib import Path
import re
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    source = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    output = Path(LaunchConfiguration('output_dir').perform(context)).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    text = source.read_text()
    for key, value in [('output_path', str(output)), ('ltv_debug_csv_path', str(output/'ltv_debug.csv'))]:
        text, count = re.subn(r'^'+key+r'\s*:.*$', key+': "'+value+'"', text, flags=re.M)
        if count != 1:
            raise RuntimeError(f'configuration must contain exactly one {key}')
    for key in ('cam0_calib','cam1_calib'):
        value = re.search(r'^'+key+r':\s*"([^"]+)"', text, re.M).group(1)
        calibration = (source.parent/value).resolve()
        target = output/calibration.name
        if calibration != target:
            shutil.copy2(calibration, target)
        text = re.sub(r'^'+key+r':.*$',key+': "'+calibration.name+'"',text,flags=re.M)
    effective = output/'effective_config.yaml'
    effective.write_text(text)
    output_topics = ('imu_propagate','path','odometry','point_cloud','margin_cloud',
        'key_poses','camera_pose','camera_pose_visual','keyframe_pose',
        'keyframe_point','extrinsic','image_track')
    actions = [Node(package='vins', executable='vins_node', name='vins_estimator',
                    arguments=[str(effective)], output='screen',
                    remappings=[(topic, '/vins_estimator/'+topic) for topic in output_topics])]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', str(Path(get_package_share_directory('vins'))/'config/tuning/stage7.rviz')],
            output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('vins'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/tuning/euroc_stage7.yaml')),
        DeclareLaunchArgument('output_dir', default_value=str(Path.home()/'output/vins_stage7_live')),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=start),
    ])
