from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    ld = LaunchDescription()

    param_config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'number_app.yaml'
    )

    number_publisher = Node(
        package='my_first_activity_py_pkg',
        executable='number_publisher',
        #remappings=[('number', 'number_topic')],
        # parameters=[{'number': 1000, 'timer_period': 0.5}]
        namespace='/test',
        parameters=[param_config],
    )

    number_counter = Node(
        package='my_first_activity_cpp_pkg',
        #remappings=[('number', 'number_topic')],
        executable='number_counter',
        namespace='/test',
    )

    ld.add_action(number_publisher) 
    ld.add_action(number_counter)

    return ld

