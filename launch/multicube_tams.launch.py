import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import PushRosNamespace

from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    # Launch arguments
    num_cubes_arg = DeclareLaunchArgument('num_cubes', default_value='2')
    poses_arg = DeclareLaunchArgument('poses', default_value='-0.8 0 0.0 0 0 0;-0.5 0.3 0.0 0 0 0')  # semicolon-separated

    tams_box_launch_path = os.path.join(
        get_package_share_directory('tams_apriltags'),
        'launch',
        'tams_boxes.launch.py'
    )

    package_prefix_path = get_package_prefix("tams_apriltags")

    num_cubes = LaunchConfiguration('num_cubes')
    poses = LaunchConfiguration('poses')

    def create_launch_group(context):
        n = int(num_cubes.perform(context))
        pose_list = poses.perform(context).split(';')
        launch_group = []

        for i in range(n):
            pose_str = pose_list[i].strip()
            name = f"cube_{i}"
            id_str = str(i)

            group = GroupAction([
                PushRosNamespace(name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(tams_box_launch_path),
                    launch_arguments={
                        'name': TextSubstitution(text=name),
                        'size': TextSubstitution(text="0.1 0.1 0.1"),
                        'color': TextSubstitution(text="0.0 1.0 0.0 1.0"),
                        'gazebo_color': TextSubstitution(text="Gazebo/Red"),
                        'pose': TextSubstitution(text=pose_str),
                        'id': TextSubstitution(text=id_str),
                    }.items()
                )
            ])
            launch_group.append(group)
            launch_group.append(
                AppendEnvironmentVariable(
                    "GZ_SIM_RESOURCE_PATH", os.path.join(package_prefix_path, "share")
                )
            )

        return launch_group

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        num_cubes_arg,
        poses_arg,
        OpaqueFunction(function=create_launch_group)
    ])
