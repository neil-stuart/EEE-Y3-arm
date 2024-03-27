import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='arm_pkg',
            executable='image_processor',
            name='image_processor'),
            
        launch_ros.actions.Node(
            package='arm_pkg',
            executable='controls_generator',
            name='controls_generator')

    ])
