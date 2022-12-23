from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mtt_file = PathJoinSubstitution([FindPackagePrefix('mtt_checker'), "tp-mtt.yaml"])
    mtt_file_arg = DeclareLaunchArgument("mtt_file",
                                           default_value='',
                                           description='mtt file')
    periodic_time_arg = DeclareLaunchArgument("periodic_time",
                                      default_value='100',
                                      description='periodic time (msec)')
    deadline_time_arg = DeclareLaunchArgument("deadline_time",
                                      default_value='200',
                                      description='deadline time (msec)')
    mtt_checker = Node(package='mtt_checker',
                                 executable='mtt_checker',
                                 output='screen',
                                 parameters = [{
                                 "periodic_time": LaunchConfiguration('periodic_time'),
                                 "deadline_time": LaunchConfiguration('deadline_time'),
                                 "mtt_file": LaunchConfiguration('mtt_file'),
                                 }]
                                )

    return LaunchDescription([
        periodic_time_arg,
        deadline_time_arg,
        mtt_file_arg,
        mtt_checker
    ])

