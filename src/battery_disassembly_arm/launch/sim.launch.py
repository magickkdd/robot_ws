"""
统一的仿真 launch 文件
- 默认模式：使用 joint_state_publisher_gui 进行简单测试
- Ignition 模式：启动完整的 Ignition Gazebo 仿真环境
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 声明参数
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur10e',
            description='UR type to load'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_ignition',
            default_value='true',
            description='Use Ignition Gazebo simulation (true) or joint_state_publisher_gui (false)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz?'
        )
    )

    ur_type = LaunchConfiguration('ur_type')
    use_ignition = LaunchConfiguration('use_ignition')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # 控制器配置文件路径（Ignition 模式使用）
    controllers_file = PathJoinSubstitution([
        FindPackageShare('battery_disassembly_arm'),
        'config',
        'ur_controllers.yaml'
    ])

    # 清理旧进程
    cleanup_processes = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f "rviz2.*sim.rviz|joint_state_publisher_gui|ign gazebo|gz sim" 2>/dev/null || true'],
        output='screen',
        shell=False
    )

    # ========== 普通模式的 URDF（不启用仿真插件） ==========
    description_cmd_normal = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('battery_disassembly_arm'),
            'urdf',
            'environment.xacro'
        ])
    ])

    # ========== Ignition 模式的 URDF（启用仿真插件） ==========
    description_cmd_ignition = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('battery_disassembly_arm'),
            'urdf',
            'environment.xacro'
        ]),
        ' sim_ignition:=true',
        ' simulation_controllers:=', controllers_file,
    ])

    # ========== 普通模式节点 ==========
    # robot_state_publisher（普通模式）
    rsp_node_normal = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': ParameterValue(description_cmd_normal, value_type=str),
                    'publish_frequency': 30.0
                }],
                condition=UnlessCondition(use_ignition)
            )
        ]
    )

    # joint_state_publisher_gui（仅普通模式）
    jsp_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
                condition=UnlessCondition(use_ignition)
            )
        ]
    )

    # RViz（普通模式，延迟启动）
    rviz_node_normal = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('battery_disassembly_arm'),
                    'rviz',
                    'sim.rviz'
                ])],
                output='screen',
                condition=UnlessCondition(use_ignition)
            )
        ]
    )

    # ========== Ignition 模式节点 ==========
    # robot_state_publisher（Ignition 模式）- 立即启动，确保 /robot_description 可用
    rsp_node_ignition = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'publish_frequency': 30.0},
                    {'robot_description': ParameterValue(description_cmd_ignition, value_type=str)}
                ],
                condition=IfCondition(use_ignition)
            )
        ]
    )

    # Clock 话题桥接 - 在 RSP 之后启动
    gz_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
                condition=IfCondition(use_ignition)
            )
        ]
    )

    # Ignition Gazebo - 在桥接之后启动
    gz_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('ros_gz_sim'),
                    '/launch/gz_sim.launch.py'
                ]),
                launch_arguments={
                    'gz_args': ['-r -v 4 ', PathJoinSubstitution([
                        FindPackageShare('battery_disassembly_arm'),
                        'worlds',
                        'empty.sdf'
                    ])]
                }.items(),
            )
        ],
        condition=IfCondition(use_ignition)
    )

    # 在 Gazebo 中生成机器人（延迟启动，等待 Gazebo 就绪）
    # 使用 -topic 从 robot_state_publisher 获取 URDF，而不是 -string
    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'workcell',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        condition=IfCondition(use_ignition)
    )

    gz_spawn_entity = TimerAction(
        period=10.0,
        actions=[gz_spawn_entity_node],
        condition=IfCondition(use_ignition)
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ignition)
    )

    # Joint Trajectory Controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ignition)
    )

    # 延迟启动控制器（在spawn完成后启动，给更多时间让gz_ros2_control初始化）
    delayed_joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[joint_state_broadcaster_spawner],
        condition=IfCondition(use_ignition)
    )

    delayed_joint_trajectory_controller = TimerAction(
        period=18.0,
        actions=[joint_trajectory_controller_spawner],
        condition=IfCondition(use_ignition)
    )

    # RViz（Ignition 模式，使用 sim_time）
    rviz_node_ignition = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('battery_disassembly_arm'),
                    'rviz',
                    'sim.rviz'
                ])],
                parameters=[{'use_sim_time': True}],
            )
        ],
        condition=IfCondition(use_ignition)
    )


    return LaunchDescription(
        declared_arguments + [
            cleanup_processes,
            # 普通模式节点
            rsp_node_normal,
            jsp_node,
            rviz_node_normal,
            # Ignition 模式节点
            rsp_node_ignition,
            gz_bridge,
            gz_launch,
            gz_spawn_entity,
            rviz_node_ignition,
            delayed_joint_state_broadcaster,
            delayed_joint_trajectory_controller,
        ]
    )
