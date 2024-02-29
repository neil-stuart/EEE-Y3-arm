# Real-Time Object Detection and Reactive Motor Control for Dynamic Object Interaction 
## Project Overview 📌

This project aims to develop and implement a system that integrates advanced sensor technologies and real-time object detection algorithms with an open-source robotic arm. The goal is to enable the arm to react to and attempt to catch objects thrown towards it, demonstrating the practical application of sensor technologies, computer vision, and control systems in dynamic and real-time object interaction.

### Contributors 👥
- Neil Stuart
- Jake van de Beek

### Supervisors 👨‍🏫
- Dr. Brian Deegan
- Co-Assessor: Dr. Soumyajyoti Maji

## System Components 🛠️

- **Robotic Arm**: BCN3D Moveo, chosen for its affordability, precision, and adaptability to project needs.
- **Sensing and Vision**: Utilizes an Intel RealSense Camera for 3D object location and tracking.
- **Computing Platform**: Runs on a Raspberry Pi 4, equipped to handle real-time image processing and motor control tasks.
- **Software Stack**:
  - **ROS2 (Robot Operating System 2)**: For orchestrating the system’s components, including motor control and sensor data integration.
  - **OpenCV in Python**: For object detection and vision processing.
  - **Custom PID Control**: Implemented for dynamic object interaction, aiming for precision in motor responses based on sensor inputs.

## Workspace Structure 📂

The ROS workspace is organized to facilitate development, testing, and deployment of the project components. Below is the structure of the workspace **UPDATE**

## Installation and Running 🚀

1. **Dependencies**: Ensure all dependencies are installed, including ROS2, OpenCV, and Python libraries specific to vision processing and motor control.
2. **Build Workspace**: Navigate to `/ros_workspace/` and run `colcon build` to compile the project.
3. **Launch**: Use ROS2 launch files to start the system components. Example: `ros2 launch moveo_ros moveo_arm.launch`.

## Future Work 🔮

- **Machine Learning for Object Detection**: Explore integrating machine learning models for enhanced object detection and classification.
- **Enhanced Motor Control**: Investigate more advanced algorithms for motor control to improve precision and response time.
- **Wider Object Interaction**: Expand the system’s capabilities to interact with a broader range of object types and trajectories.

## Acknowledgments 🙏

Special thanks to Dr. Brian Deegan and Dr. Soumyajyoti Maji for their guidance throughout the project, and to Mr. Myles Meehan and Mr. Darragh Mullins for their assistance in sourcing parts.
