from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubb_pkg',
            namespace=None,
            executable='pub_node',
            name='pub_node'
        ),
        Node(
            package='pubb_pkg',
            namespace=None,
            executable='sub_n_pub_node',
            name='sub_n_pub_node'
        ),
        Node(
            package='turtlesim',
            namespace=None,
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='pubb_pkg',
            namespace=None,
            executable='tservice_node',
            name='tservice_node'
        ),
        Node(
        package='pubb_pkg',
        namespace=None,
        executable='taction_node',
       
        ),
        # Node(
        # package='pubb_pkg',
        # namespace=None,
        # executable='muti_node',
       
        # ),

        
    ])
