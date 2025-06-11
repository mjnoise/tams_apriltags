import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def launch_setup(context, *args, **kwargs):
    # Read arguments
    #name sx sy sz color gazebo_color ID
    name = LaunchConfiguration("name").perform(context)
    # sx, sy, sz = LaunchConfiguration("size").perform(context).split()
    # color = LaunchConfiguration("color").perform(context)
    # gazebo_color = LaunchConfiguration("gazebo_color").perform(context)
    pose = LaunchConfiguration("pose").perform(context).split()


    # Locate sdf file
    package_path = get_package_share_directory("tams_apriltags")
    file = os.path.join(package_path, "urdf", "table.sdf")

    with open(file, 'r') as infp:
        robot_desc = infp.read()

    package_prefix_path = get_package_prefix("tams_apriltags")
    gz_cube_env_var_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(package_prefix_path, "share")
    )

    # Robot State Publisher for the table
    table_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=name,
        name='table_state_publisher',
        output="both",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": True,
        }],
    )

    table_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='table_tf_pub',    
        arguments=[
            str(pose[0]), str(pose[1]), str(pose[2]),      # x y z
            str(pose[3]), str(pose[4]), str(pose[5]),      # roll pitch yaw (radians)
            'world', 'link'                                # parent frame, child frame
        ],
    )
    

    # Spawn directly into Gazebo (no robot_state_publisher)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", name,
            "-file", file,
            "-x", pose[0], "-y", pose[1], "-z", pose[2],
            "-R", pose[3], "-P", pose[4], "-Y", pose[5]
        ]
    )

    return [gz_cube_env_var_resource_path, table_state_publisher, table_tf, spawn_entity]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="table"),
        # DeclareLaunchArgument("size", default_value="0.77, 0.77, 0.77"),
        # DeclareLaunchArgument("color", default_value="0.0 1.0 0.0 1.0"),
        # DeclareLaunchArgument("gazebo_color", default_value="0.0 1.0 0.0"),
        DeclareLaunchArgument("pose", default_value="-0.8 0 0.0 0 0 0"),
        OpaqueFunction(function=launch_setup)
    ])