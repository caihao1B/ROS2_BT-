from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- common args ---
    plan = LaunchConfiguration('plan')
    mode = LaunchConfiguration('mode')
    use_smoother = LaunchConfiguration('use_smoother')
    resume = LaunchConfiguration('resume')

    controller_id = LaunchConfiguration('controller_id')
    goal_checker_id = LaunchConfiguration('goal_checker_id')
    progress_checker_id = LaunchConfiguration('progress_checker_id')

    tb3_model = LaunchConfiguration('tb3_model')
    headless = LaunchConfiguration('headless')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    start_initial_pose_delay = LaunchConfiguration('start_initial_pose_delay')
    start_executor_delay = LaunchConfiguration('start_executor_delay')

    # NEW: allow splitting dynamic obstacles to a separate terminal
    start_dynamic_obstacles = LaunchConfiguration('start_dynamic_obstacles')
    obstacles_params_file = LaunchConfiguration('obstacles_params_file')

    # --- package resources ---
    executor_params = PathJoinSubstitution(
        [FindPackageShare('coverage_manager'), 'params', 'executor_params.yaml']
    )
    metrics_params = PathJoinSubstitution(
        [FindPackageShare('coverage_metrics'), 'params', 'metrics_params.yaml']
    )
    default_obstacles_params = PathJoinSubstitution(
        [FindPackageShare('coverage_sim'), 'params', 'dynamic_obstacles_params.yaml']
    )

    default_plan = PathJoinSubstitution(
        [FindPackageShare('farm_gate_bringup'), 'plans', 'plan_micro_gate_bt_4seg.json']
    )
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare('farm_gate_bringup'), 'rviz', 'farm_gate_coverage.rviz']
    )

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    tb3_sim_launch = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'tb3_simulation_launch.py']
    )

    farm_models_path = PathJoinSubstitution(
        [FindPackageShare('farm_gate_world'), 'models']
    )

    set_tb3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model)
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':', farm_models_path],
    )

    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_sim_launch),
        launch_arguments={
            'slam': 'False',
            'use_sim_time': 'True',
            'autostart': 'True',
            'headless': headless,
            'world': world,
            'map': map_yaml,
            'params_file': nav2_params_file,
            'rviz_config_file': rviz_cfg,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw': yaw,
        }.items(),
    )

    initial_pose_pub = Node(
        package='farm_gate_bringup',
        executable='publish_initial_pose',
        name='initial_pose_publisher',
        output='screen',
        parameters=[
            {
                'x': x_pose,
                'y': y_pose,
                'yaw': yaw,
                'frame_id': 'map',
                'publish_count': 6,
                'publish_rate_hz': 1.0,
            }
        ],
    )

    dynamic_obstacles = Node(
        package='coverage_sim',
        executable='dynamic_obstacles',
        name='dynamic_obstacles',
        output='screen',
        parameters=[
            obstacles_params_file,
            {
                'mode': mode,
                'plan_path': plan,
            },
        ],
        condition=IfCondition(start_dynamic_obstacles),
    )

    executor = Node(
        package='coverage_manager',
        executable='coverage_executor',
        name='coverage_executor',
        output='screen',
        parameters=[
            executor_params,
            {
                'plan_path': plan,
                'use_smoother': use_smoother,
                'resume_from_state': resume,
                'controller_id': controller_id,
                'goal_checker_id': goal_checker_id,
                'progress_checker_id': progress_checker_id,
            },
        ],
    )

    metrics = Node(
        package='coverage_metrics',
        executable='coverage_metrics',
        name='coverage_metrics',
        output='screen',
        parameters=[metrics_params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('plan', default_value=default_plan),
            DeclareLaunchArgument('mode', default_value='low'),  # low/high
            DeclareLaunchArgument('use_smoother', default_value='false'),
            DeclareLaunchArgument('resume', default_value='true'),

            DeclareLaunchArgument('controller_id', default_value=''),
            DeclareLaunchArgument('goal_checker_id', default_value=''),
            DeclareLaunchArgument('progress_checker_id', default_value=''),

            DeclareLaunchArgument('tb3_model', default_value='waffle'),
            DeclareLaunchArgument('headless', default_value='False'),

            DeclareLaunchArgument('x_pose', default_value='1.2'),
            DeclareLaunchArgument('y_pose', default_value='1.4'),
            DeclareLaunchArgument('z_pose', default_value='0.03'),
            DeclareLaunchArgument('yaw', default_value='0.0'),

            DeclareLaunchArgument(
                'world',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('farm_gate_world'), 'worlds', 'farm_field.world']
                ),
            ),
            DeclareLaunchArgument(
                'map',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('farm_gate_world'), 'maps', 'map.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'nav2_params_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml']
                ),
            ),

            DeclareLaunchArgument('start_initial_pose_delay', default_value='4.0'),
            DeclareLaunchArgument('start_executor_delay', default_value='10.0'),

            # NEW
            DeclareLaunchArgument('start_dynamic_obstacles', default_value='true'),
            DeclareLaunchArgument('obstacles_params_file', default_value=default_obstacles_params),

            set_tb3_model_env,
            set_gazebo_model_path,
            nav2_sim,

            TimerAction(period=start_initial_pose_delay, actions=[initial_pose_pub]),
            TimerAction(period=start_executor_delay, actions=[dynamic_obstacles, executor, metrics]),
        ]
    )
