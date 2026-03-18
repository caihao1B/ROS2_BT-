from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    plan = LaunchConfiguration('plan')
    mode = LaunchConfiguration('mode')
    headless = LaunchConfiguration('headless')
    tb3_model = LaunchConfiguration('tb3_model')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    tb3_sim_launch = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'tb3_simulation_launch.py']
    )

    rviz_cfg = PathJoinSubstitution([FindPackageShare('farm_gate_bringup'), 'rviz', 'farm_gate_coverage.rviz'])
    default_plan = PathJoinSubstitution([FindPackageShare('farm_gate_bringup'), 'plans', 'plan_micro_gate_bt_4seg.json'])

    set_tb3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model)

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
        parameters=[{'x': x_pose, 'y': y_pose, 'yaw': yaw, 'frame_id': 'map', 'publish_count': 6, 'publish_rate_hz': 1.0}],
    )

    # Existing dynamic obstacles generator (optional)
    obstacles_params = PathJoinSubstitution([FindPackageShare('coverage_sim'), 'params', 'dynamic_obstacles_params.yaml'])
    dynamic_obstacles = Node(
        package='coverage_sim',
        executable='dynamic_obstacles',
        name='dynamic_obstacles',
        output='screen',
        parameters=[obstacles_params, {'mode': mode, 'plan_path': plan}],
    )

    # New pipeline (stubs for now)
    scan_tracker = Node(
        package='coverage_perception',
        executable='scan_tracker',
        name='scan_tracker',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('coverage_perception'), 'params', 'scan_tracker.yaml'])],
    )

    predictor = Node(
        package='coverage_prediction',
        executable='cv_predictor',
        name='cv_predictor',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('coverage_prediction'), 'params', 'cv_predictor.yaml'])],
    )

    region_state = Node(
        package='coverage_replanner',
        executable='region_state',
        name='coverage_region_state',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('coverage_replanner'), 'params', 'region_state.yaml']),
            {'plan_path': plan},
        ],
    )

    d1_advisor = Node(
        package='coverage_replanner',
        executable='d1_advisor',
        name='coverage_d1_advisor',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('coverage_replanner'), 'params', 'd1_advisor.yaml'])],
    )

    d2_advisor = Node(
        package='coverage_replanner',
        executable='d2_advisor',
        name='coverage_d2_advisor',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('coverage_replanner'), 'params', 'd2_advisor.yaml'])],
    )

    selector = Node(
        package='coverage_decision',
        executable='strategy_selector',
        name='strategy_selector',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('coverage_decision'), 'params', 'strategy_selector.yaml'])],
    )

    replanner = Node(
        package='coverage_replanner',
        executable='local_replanner',
        name='local_replanner',
        output='screen',
    )

    supervisor = Node(
        package='coverage_bt_supervisor',
        executable='coverage_supervisor',
        name='coverage_bt_supervisor',
        output='screen',
        parameters=[{'plan_path': plan}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('plan', default_value=default_plan),
        DeclareLaunchArgument('mode', default_value='low'),
        DeclareLaunchArgument('tb3_model', default_value='waffle'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('x_pose', default_value='1.2'),
        DeclareLaunchArgument('y_pose', default_value='1.4'),
        DeclareLaunchArgument('z_pose', default_value='0.03'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([FindPackageShare('farm_gate_world'), 'worlds', 'farm_field.world'])
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([FindPackageShare('farm_gate_world'), 'maps', 'map.yaml'])
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml'])
        ),
        set_tb3_model_env,
        nav2_sim,
        TimerAction(period=4.0, actions=[initial_pose_pub]),
        TimerAction(period=10.0, actions=[dynamic_obstacles, scan_tracker, predictor, region_state, d1_advisor, d2_advisor, selector, replanner, supervisor]),
    ])
