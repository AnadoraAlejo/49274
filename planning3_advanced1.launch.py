from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'degradation':('none',str),'severity':('0',int),
              'use_improved':('false',bool),'planning_enabled':('true',bool),
              'use_sim_time':('true',bool),'output_directory':('~/cave_results',str)}
    arguments=[DeclareLaunchArgument(k,default_value=v[0]) for k,v in defaults.items()]
    parameters={k:ParameterValue(LaunchConfiguration(k),value_type=v[1]) for k,v in defaults.items()}
    return LaunchDescription(arguments+[Node(package='cave_explorer',executable='cave_mission',
                                            output='screen',parameters=[parameters])])
