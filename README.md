### Installation and Running
1. Upload stepper_ctrlv2.ino to arduino nano. 
2. Connect arduino pins as they are indicated in stepper_ctrlv2.ino.
3. Install the python package:

`cd moveo_python_package`

`pip install .`

5. Find the COM port that arduino is connected to and set that value at the head of the client.py script.
6. Run client.py.

`python ./client/client.py`

*Stepper drivers take 24V supply.
*Arm doesn't have any feedback so it needs to be started straight up as its initial position. 

## Real-Time Object Detection and Reactive Motor Control for Dynamic Object Interaction 
### Overview 📌

This project aims to develop and implement a system that integrates advanced sensor technologies and real-time object detection algorithms with an open-source robotic arm. The goal is to enable the arm to map and interact with objects in its environment, demonstrating the practical application of sensor technologies, computer vision, and control systems in dynamic and real-time object interaction.

### Contributors 
- Neil Stuart
- Jake van de Beek
