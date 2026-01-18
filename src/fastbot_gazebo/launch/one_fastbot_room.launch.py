from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution


def generate_launch_description():
    headless = LaunchConfiguration("headless")

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without gzclient"
    )

    dir_path = ThisLaunchFileDir()
    launch_first_path = PathJoinSubstitution(
        [dir_path, "botbox_world_room.launch.xml"])
    launch_second_path = PathJoinSubstitution(
        [dir_path, "spawn_fastbot.launch.xml"])

    launch_first_action = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_first_path),
        launch_arguments={
            "headless": headless,
        }.items(),
    )

    launch_second_action = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_second_path),
        launch_arguments={
            "headless": headless,
        }.items(),
    )

    delay_second_launch = TimerAction(
        period=20.0, actions=[launch_second_action])

    return LaunchDescription([
        declare_headless,
        launch_first_action,
        delay_second_launch,
    ])
