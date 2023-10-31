from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "using_left_arm",
            default_value="true",
            description="Arguement to determine wether to use left arm.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "using_right_arm",
            default_value="true",
            description="Arguement to determine wether to use right arm.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "using_fake_hardware",
            default_value="false",
            description="Arguement to determine wether to use fake hardware.",
        )
    )

    # initialize arguments
    using_left_arm = LaunchConfiguration("using_left_arm")
    using_right_arm = LaunchConfiguration("using_right_arm")
    using_fake_hardware =  LaunchConfiguration("using_fake_hardware")

    # variables
    robot_description_package = FindPackageShare("separable_jaws_description")
    robot_bringup_package = FindPackageShare("separable_jaws_bringup")
    robot_description_file = PathJoinSubstitution([
        robot_description_package,
        "urdf",
        "separable_jaws.xacro"
    ])
    robot_controllers_file = PathJoinSubstitution([
        robot_bringup_package,
        "config",
        "separable_jaws_controllers.yaml"
    ])
    rviz_config_file = PathJoinSubstitution([
        robot_description_package,
        "config",
        "urdf.rviz"
    ])
    # convert xacro into urdf
    robot_description = Command([
        PathJoinSubstitution([
            FindExecutable(name="xacro")
        ]),
        " ",
        robot_description_file,
        " ",
        "using_left_arm:=",
        using_left_arm,
        " ",
        "using_right_arm:=",
        using_right_arm,
        " ",
        "using_fake_hardware:=",
        using_fake_hardware,
        " ",
        "using_gazebo:=false",
    ])

    # declare nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ],
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers_file
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
    )
    left_arm_controller_spawner = Node(
        condition=IfCondition(using_left_arm),
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    right_arm_controller_spawner = Node(
        condition=IfCondition(using_right_arm),
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )
    # joint_position_controller for testing purposes
    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_position_controller",
            "--controller-manager", "/controller_manager"
        ],
    )
    # velocity_controller for testing purposes
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "velocity_controller",
            "--controller-manager", "/controller_manager"
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    delay_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                left_arm_controller_spawner,
                right_arm_controller_spawner,
                rviz_node,
            ],
        )
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(arguments + nodes)
