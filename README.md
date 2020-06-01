# 7dof_arm_tracker
This repository contains 3D CAD models and source code for 7 DOF arm tracker developed in ALARIS Lab at Nazarbayev University. For more information see [link](https://www.alaris.kz/research/intelligent_assist_system/)  

Please cite the below work if you utilize our arm tracker design in your academic work:
### *Shintemirov, A.; Taunyazov, T.; Omarali, B.; Nurbayeva, A.; Kim, A.; Bukeyev, A.; Rubagotti, M.* **An Open-Source 7-DOF Wireless Human Arm Motion-Tracking System for Use in Robotics Research.** *Sensors* 2020, 20, 3082. ([link](https://www.mdpi.com/1424-8220/20/11/3082))  

The code is written to run on ROS and was tested on ROS Kinetic, but should be compatible with later versions of ROS as well. To install ROS, follow installation instructions on [ros.org](http://handlebarsjs.com/).

## How to set up WIXEL

0. Install Pololu Wixel Configuration Utility - it can be downloaded [here](https://www.pololu.com/product/1336/resources)
 
### Follow these steps to upload programs to Wixel
1. Connect the Wixel controller to the PC via the USB port.
2. Run the Pololu Wixel Configuration Utility program, the serial number of your controller should appear in the "Wixels" window.

#### General parameters for wixel controllers that are associated with UM7
3. Click "Open" button to upload the file `wireless_serial.wxl`.
4. In the `Settings` table, `serial_mode` should be filled in different ways depending on which Wixel (receiver or transmitter) you are going to configure. For receivers it is 1, and for transmitters 2. 
5. `Baud rate` for everything is 115200.
6. nDTR_pin - 5, nRTS_pin - 11, nDRS_pin - 4, nCD_pin - 13, arduino_DTR_pin - 0, framing_error_ms - 0 for both transmitter and receiver.
7. For one pair of receiver and transmitter, one radio channel must be selected. The channel number is from 0 to 255 and determines what frequency to broadcast. The default value is 128. The wixels must be on the same channel to communicate with each other. To avoid interference, Wixels that aren’t supposed to talk to each other should be at least 2 channels away from each other. For example, you could have one pair of Wixels on channel 128 and another pair on 130.
8. After completing the table, click the `Write to Wixel` button.

#### General parameters for a wixel controller connected to a potentiometer
3. Upload `wireless_adc_tx.wxl` into the wixel transmitter and select one radio channel for pair.
4. Upload `wireless_adc_rx.wxl` to the wixel receiver and indicate the selected radio channel in the table.
5. After filling out the table, press the button `Write to Wixel`.

## Setting up the code
Create a ROS workspace. In a terminal window run:  
`mkdir -p ~/catkin_ws/src`  
`cd ~/catkin_ws/`  
`catkin_make`  

* Put the ***serial_comm*** folder into ***catkin_ws/src*** folder.  
* Run the following command from workspace root directory to install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`  
* Run **catkin_make** from workspace root directory to build the code.

* Change mod of ***controller_node.py*** to *'executable'* with `chmod +x controller_node.py` to be able to run the program.  
* Add yourself to `dialout` group with command `sudo adduser $USER dialout`. Log out and log in again for the command to take effect. This is needed for you to have the rights to open serial ports.  
* Assign to tracker ports static port names as in ***serial_comm/src/serial_comm.cpp*** lines 36-38. Specifically, call them *port_shoulder, port_elbow, port_wrist*. To find out which port belongs to which sensor use `sudo cat /dev/ttyACMx`, with `x` replaced by port number, to start listening to the port, then turn on each sensor one by one and observe if there is a feedback.
* Then follow [this](https://msadowski.github.io/linux-static-port) guide to link every port to a static name. As device attribute (ATTRS) use Wixel's serial number.
* After finishing the guide unplug and plug in the USB hub with Wixels and check  with `ls /dev | grep port_` if there are corresponding ports.

Finally, you will need to install [CoppeliaSim](https://www.coppeliarobotics.com/) to run the arm simulation.

## Run the code
1. Run CoppeliaSim
2. Open scene ***serial_comm/simple_arm.ttt***
3. Start *roscore*
4. Read sensors raw data and compute quaternions: `rosrun serial_comm serial_comm`  
For estimating quaternions we use IMU and AHRS sensor fusion algorithm by Sebastian Madgwick from [here](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
5. Send estimated quaternions to arm joints in CoppeliaSim and get end-effector pose: `rosrun serial_comm controller_node.py`  
End-effector pose is published to topic */end_effector*
6. Magdwick algorithm needs some time to converge. Wait until it converges and initialize it with `rostopic pub /initialize std_msgs/String "data: 'init'" -1`
7. Done.

Note: Depending on presense of metal in the environment, which disturbs magnetometer, the algorithm will give shifted quaternions. Tune orientation by playing with quaternions in **controller_node.py** lines 24-46.

