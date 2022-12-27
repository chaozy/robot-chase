import launch
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('robot_chase')

    world = PathJoinSubstitution([package_dir, 'worlds', 'world.wbt'])
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode='pause'
    )
    robot_name = "robot0"

    return LaunchDescription([
        webots,
        # IncludeLaunchDescription(
        #     launch_description_source=PythonLaunchDescriptionSource([package_dir, '/launch/robot_launch.py']),
        #     launch_arguments={'robot_name': robot_name}.items()
        # ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
