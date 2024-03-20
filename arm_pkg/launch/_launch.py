import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([

        # Adding the realsense2_camera_node with remapping
        launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='MOVEO',  # Set the namespace to robot1
            name='RS_CAM',  # Remap the node name to D455_1
            remappings=[
                # Add specific topic remappings here if needed
                # For example: ('/old/topic/name', '/new/topic/name'),
            ],
            parameters=[{
                'depth_module.profile': '848x480x60',
                'rgb_camera.profile': '848x480x60',
            }],
        ),

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
