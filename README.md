# 7dof_arm_tracker
This repository contains files 3D CAD models and source code for 7 DOF arm tracker developed in ALARIS Lab at Nazarbayev University

The code is written to run on ROS and was tested on ROS Kinetic, but should be compatible with later versions of ROS as well. To install ROS follow installation instructions on [ros.org](http://handlebarsjs.com/).

## Setting up
Create a ROS workspace. In a terminal window run:  
`mkdir -p ~/catkin_ws/src`  
`cd ~/catkin_ws/`  
`catkin_make`  

Put the ***serial_comm*** folder into ***catkin_ws/src*** folder.  
Run the following command from workspace root directory: `rosdep install --from-paths src --ignore-src -r -y`  
Run **catkin_make** from workspace root directory to build the code.

Change mod of ***controller_node.py*** to *'executable'* with `chmod +x controller_node.py` to be able to run the program.  
Add yourself to `dialout` group with command `sudo adduser $USER dialout`. Log out and log in again for the command to take effect. This is needed for you to have the rights to open serial ports.  
Assign to tracker ports static port names as in *serial_comm.cpp* lines 36-38. Specifically, call them *port_shoulder, port_elbow, port_wrist*. To find out which port belongs to which sensor use `sudo cat /dev/ttyACMx`, with `x` replaced by port number, to start listening to the port, then turn on each sensor one by one and observe. Then follow [this](https://msadowski.github.io/linux-static-port/") guide to link every port to a static name. For assurance, after finishing the guide unplug and plug in the USB hub and check  with `ls /dev | grep port_` if there are corresponding ports.

Finally, you will need to install [CoppeliaSim](https://www.coppeliarobotics.com/)

## Run the code
TODO
