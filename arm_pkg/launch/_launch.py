import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='arm_pkg',
            executable='camera_interface',
            name='camera_interface'),
        launch_ros.actions.Node(
            package='arm_pkg',
            executable='image_processor',
            name='image_processor'),
        launch_ros.actions.Node(
            package='arm_pkg',
            executable='controls_generator',
            name='controls_generator'),
        launch_ros.actions.Node(
            package='arm_pkg',
            executable='hardware_interface',
            name='hardware_interface'),
  ])