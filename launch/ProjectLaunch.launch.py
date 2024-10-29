from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        #launch world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('smart_factory'),'launch','factory_world.launch.py'))
        ),

        #launch nav2
        # ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/charles/roboStu1_ros2_ws/src/sprint2/map/my_map.yaml
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'map:=/home/charles/roboStu1_ros2_ws/src/sprint3/map/my_map.yaml'],
            output='screen'
        ),

        #may need to be a delay before startig this node, but works for now
        Node(
            package='smart_factory',
            executable='MasterControlNodeExe',
            name='MasterControlNode',
            output='screen',
        ),

        Node(
            package='smart_factory',
            executable='CylinderDetectionExe',
            name='CylinderDetectionNode',
            output='screen',
        )
    ])


#ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/charles/roboStu1_ros2_ws/src/sprint2/map/my_map.yaml