### Installation and Running
1. Upload stepper_ctrlv2.ino to arduino nano. 
2. Connect arduino pins as they are indicated in stepper_ctrlv2.ino.
3. Install the python package:

`cd moveo_python_package`

`pip install .`

5. Find the COM port that arduino is connected to and set that value at the head of the client.py script.
6. Run client.py.

`python ./client/client.py`

# Real-Time Object Detection and Reactive Motor Control for Dynamic Object Interaction 
## Project Overview 📌

This project aims to develop and implement a system that integrates advanced sensor technologies and real-time object detection algorithms with an open-source robotic arm. The goal is to enable the arm to map and interact with objects in its environment, demonstrating the practical application of sensor technologies, computer vision, and control systems in dynamic and real-time object interaction.



### Contributors 👥
- Neil Stuart
- Jake van de Beek

### Supervisors 
- Dr. Brian Deegan
- Co-Assessor: Dr. Soumyajyoti Maji

## Components 🛠️

- **Robotic Arm**: BCN3D Moveo, chosen for its affordability, precision, and adaptability to project needs.
- **Sensing and Vision**: Utilizes an Intel RealSense Camera for 3D object location and tracking.


## Future Work

- **Machine Learning for Object Detection**: Explore integrating machine learning models for enhanced object detection and classification.
- **Enhanced Motor Control**: Investigate more advanced algorithms for motor control to improve precision and response time.
- **Wider Object Interaction**: Expand the system’s capabilities to interact with a broader range of object types and trajectories.

## Acknowledgments 

Special thanks to Dr. Brian Deegan and Dr. Soumyajyoti Maji for their guidance throughout the project, and to Mr. Myles Meehan and Mr. Darragh Mullins for their assistance in sourcing parts.
