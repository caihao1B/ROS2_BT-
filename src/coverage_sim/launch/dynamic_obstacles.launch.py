from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default files inside this workspace
    default_params = PathJoinSubstitution(
        [FindPackageShare('coverage_sim'), 'params', 'dynamic_obstacles_agri_suite.yaml']
    )

    default_plan = PathJoinSubstitution(
        [FindPackageShare('farm_gate_bringup'), 'plans', 'plan_farm_gate_exec.json']
    )

    params_file = LaunchConfiguration('params_file')
    plan = LaunchConfiguration('plan')
    scenario = LaunchConfiguration('scenario')
    scenario_file = LaunchConfiguration('scenario_file')

    node = Node(
        package='coverage_sim',
        executable='dynamic_obstacles',
        name='dynamic_obstacles',
        output='screen',
        parameters=[
            params_file,
            {
                # these allow quick CLI override without editing params_file
                'plan_path': plan,
                'scenario': scenario,
                'scenario_file': scenario_file,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument('plan', default_value=default_plan),
            DeclareLaunchArgument('scenario', default_value='agri_suite'),
            DeclareLaunchArgument('scenario_file', default_value=''),
            node,
        ]
    )
