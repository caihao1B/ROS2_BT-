import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,SetEnvironmentVariable,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    p=Path(get_package_share_directory('dino_sensors'))/'generated';c=Path(get_package_share_directory('dino_control'))/'generated'
    dual=LaunchConfiguration('dual_lidar').perform(context).lower()=='true';ekf=LaunchConfiguration('enable_ekf').perform(context).lower()=='true'
    validation=LaunchConfiguration('validation_world').perform(context).lower()=='true'
    world=p/(('validation_' if validation else 'runtime_')+('dual' if dual else 'single')+'.sdf')
    custom_world=LaunchConfiguration('world_file').perform(context)
    if custom_world:
        world=Path(custom_world).resolve()
        if not world.is_file():raise ValueError('world_file does not exist')
    actions=[SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',get_package_prefix('dino_control')+'/lib'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.pathsep.join(filter(None,[str(Path(get_package_share_directory('dino_description')).parent),os.environ.get('GZ_SIM_RESOURCE_PATH','')]))),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE',os.environ.get('DINO_SOFTWARE_RENDERING','0')),SetEnvironmentVariable('OPENBLAS_NUM_THREADS','1'),
        ExecuteProcess(cmd=['gz','sim','--force-version','8','-s','-r','-v','3',str(world)],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':(p/'dino_sensors.urdf').read_text()}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(p/'bridge.yaml')}]),
        Node(package='dino_control',executable='drive_node.py',parameters=[{'use_sim_time':True,'contract_file':str(c/'contract.json'),'publish_odom_tf':False}]),
        Node(package='dino_sensors',executable='scan_filter.py',parameters=[{'use_sim_time':True,'generated_dir':str(p),'enable_rear':dual}])]
    if ekf:
        actions.extend([Node(package='robot_localization',executable='ekf_node',name='dino_ekf',parameters=[str(p/'ekf.yaml')],remappings=[('odometry/filtered','/dino/estimator/rear_odometry'),('/tf','/dino/estimator/unused_tf')]),
                        Node(package='dino_sensors',executable='estimator_io.py',parameters=[{'use_sim_time':True,'contract_file':str(p/'contract.json')}])])
    return actions

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('dual_lidar',default_value='false'),DeclareLaunchArgument('enable_ekf',default_value='true'),
        DeclareLaunchArgument('validation_world',default_value='false'),DeclareLaunchArgument('world_file',default_value=''),OpaqueFunction(function=setup)])
