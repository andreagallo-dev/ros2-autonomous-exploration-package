# turtlebot3_my_slam/launch/tb3_slam_nav_rviz.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    tb3_nav2_dir = get_package_share_directory("turtlebot3_navigation2")
    my_pkg_dir = get_package_share_directory("turtlebot3_my_slam")

    # RViz config inside this package
    rviz_config = os.path.join(my_pkg_dir, "rviz", "slam.rviz")

    # Nav2 params: TurtleBot3 Humble Burger
    params_file = os.path.join(
        tb3_nav2_dir,
        "param",
        "humble",
        "burger.yaml",
    )

    # SLAM Toolbox publishes /map and map->odom TF
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Nav2 in navigation-only mode (for SLAM). No map_server / no AMCL.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )

    # RViz (nav2_bringup helper launch) using your custom config
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock (Gazebo) as time source",
            ),
            slam,
            nav2,
            rviz,
        ]
    )