from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare shared arguments
    declared_args = [
        DeclareLaunchArgument("robot_dof", default_value="7"),
        DeclareLaunchArgument("robot_type", default_value="gen3"),
        DeclareLaunchArgument("robot_gripper", default_value="robotiq_2f_85"),
        DeclareLaunchArgument("robot_launch", default_value="kortex_sim.launch.py"),

        # AprilTag arguments
        DeclareLaunchArgument("name", default_value="april_tag_box"),
        DeclareLaunchArgument("size", default_value="0.77, 0.77, 0.77"),
        DeclareLaunchArgument("color", default_value="0.0 1.0 0.0"),
        DeclareLaunchArgument("gazebo_color", default_value="Gazebo/Red"),
        DeclareLaunchArgument("pose", default_value="0.3 0 0 0 0 0"),
        DeclareLaunchArgument("id", default_value="0"),
        
    ]

    kortex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("kortex_bringup"),
            "/launch/kortex_sim_control.launch.py",
        ]),
        launch_arguments={
            "robot_type": LaunchConfiguration("robot_type"),
            "dof": LaunchConfiguration("robot_dof"),
            "gripper": LaunchConfiguration("robot_gripper"),
            "sim_gazebo": "true",
        }.items(),
    )

    box_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("tams_apriltags"),
            "/launch/tams_boxes.launch.py"
        ]),
        launch_arguments={
            "name": LaunchConfiguration("name"),
            "size": LaunchConfiguration("size"),
            "color": LaunchConfiguration("color"),
            "gazebo_color": LaunchConfiguration("gazebo_color"),
            "pose": LaunchConfiguration("pose"),
            "id": LaunchConfiguration("id"),
        }.items(),
    )

    return LaunchDescription(declared_args + [box_spawn_launch, kortex_launch])
