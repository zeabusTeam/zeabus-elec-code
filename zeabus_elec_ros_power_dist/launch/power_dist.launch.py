import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='zeabus_elec_ros_power_dist',
            node_executable='main',
            node_name='PowerDist',
            output='screen', 
            parameters=[{
                'ul_io_direction': 65535,
                'ul_io_state': 7936
            }]
        )
    ])
