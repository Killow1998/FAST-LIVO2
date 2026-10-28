from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def frontend(context):
    with open(LaunchConfiguration('config_file').perform(context)) as stream:
        parameters = yaml.safe_load(stream)
    with open(LaunchConfiguration('camera_file').perform(context)) as stream:
        camera = yaml.safe_load(stream)
    parameters['pcd_save']['pcd_save_en'] = False
    parameters['pcd_save']['colmap_output_en'] = False
    parameters['image_save'] = {'img_save_en': False}
    parameters['evo']['pose_output_en'] = False
    return [Node(package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
                 parameters=[parameters, camera, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                 additional_env={'OMP_WAIT_POLICY': 'PASSIVE'}, output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file'),
                              DeclareLaunchArgument('camera_file'),
                              DeclareLaunchArgument('use_sim_time', default_value='false'),
                              OpaqueFunction(function=frontend)])
