#!/usr/bin/env python
import rospy
import vrep
from geometry_msgs.msg import Quaternion  
from serial_comm.msg import quats
from pyquaternion import Quaternion
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelState , ModelStates

joint_handles = None
clientID = None

pub = rospy.Publisher("/end_effector", Float32MultiArray, queue_size=1)

def callback(quat):
    global joint_handles
    global clientID

    # Decompose message
    wrist = quat.wrist_quat
    shoulder = quat.shoulder_quat
    elbow_angle_in_rad = quat.elbow_angle_in_rad

    # Rotate shoulder quaternion +90 degrees around z-axis
    q_shoulder = Quaternion(shoulder.w, shoulder.x, shoulder.y, shoulder.z)
    rot = Quaternion(w=0.0, x=0.707, y=0.707, z=0.0)
    q_shoulder = rot * q_shoulder * rot.inverse
    rot_s = Quaternion(w=0.0, x=0.0, y=0.0, z=1.0)
    q_shoulder = rot_s*q_shoulder
    elements_s = q_shoulder.elements

    # Elbow angle
    q_lower = Quaternion(axis=(0.0, 0.0, 1.0), radians=elbow_angle_in_rad)
    elements_l = q_lower.elements

    q_wrist = Quaternion(wrist.w, wrist.z, wrist.x, wrist.y)
    rot_w_y = Quaternion(w=0.707, x=0.0, y=0.707, z=0.0)
    rot_w_x = Quaternion(w=0.707, x=0.707, y=0.0, z=0.0)
    q_wrist = rot_w_x*rot_w_y*q_wrist*rot_w_y.inverse*rot_w_x.inverse
    rot_w = Quaternion(w=0.707, x=0.707, y=0.0, z=0.0)
    q_wrist= rot_w * q_wrist
    rot_w = Quaternion(w=0.985, x=0.0, y=0.0, z=-0.184)
    q_wrist= rot_w * q_wrist
    rot_w2 = Quaternion(w=0.0, x=0.0, y=0.0, z=1.0)
    q_wrist= rot_w2 * q_wrist
    elements_w = q_wrist.elements

    # Send orientation to VREP
    if not joint_handles is None: 
        # send shoulder orientation
        vrep.simxSetObjectQuaternion(clientID, joint_handles[0], -1,
                                     [elements_s[1], elements_s[2], elements_s[3], elements_s[0]], vrep.simx_opmode_oneshot)
        # send lower arm orientation
        vrep.simxSetObjectQuaternion(clientID, joint_handles[2], joint_handles[1],
                                     [elements_l[1], elements_l[2], elements_l[3], elements_l[0]], vrep.simx_opmode_oneshot)
        # send wrist orientation
        vrep.simxSetObjectQuaternion(clientID, joint_handles[4], -1,
                                     [elements_w[1], elements_w[2], elements_w[3], elements_w[0]], vrep.simx_opmode_oneshot) 

        # get end effector orientation
        ret, q = vrep.simxGetObjectQuaternion(clientID, joint_handles[4], -1, vrep.simx_opmode_buffer)

        # calculate end effector position
        ret, posb = vrep.simxGetObjectPosition(clientID, joint_handles[0], -1, vrep.simx_opmode_buffer)
        ret, pos = vrep.simxGetObjectPosition(clientID, joint_handles[4], -1, vrep.simx_opmode_streaming)
        print(pos)

        end_eff = [pos[0], pos[1], pos[2], q[3], q[0], q[1], q[2]]
        end_eff_data = Float32MultiArray()
        end_eff_data.data = end_eff
        pub.publish(end_eff_data)
    
def listener():
    global joint_handles
    global clientID

    rospy.init_node('controller', anonymous=True)

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,500,5) # Connect to V-REP
    if clientID!=-1:
        print ('Connected to remote API server')

        vrep.simxSynchronous(clientID,True)

        joint_names = ['Shoulder_Joint', 'Upper_Arm_Joint','Lower_Arm_Joint', 'Wrist_Joint', 'Palm_Joint']
        joint_handles = [vrep.simxGetObjectHandle(clientID,name, vrep.simx_opmode_blocking)[1] for name in joint_names]

        dt = .1
        vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot)
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)

        # Wrist position and orientation
        q=vrep.simxGetObjectQuaternion(clientID, joint_handles[4], -1, vrep.simx_opmode_streaming)
        pos=vrep.simxGetObjectPosition(clientID, joint_handles[4], joint_handles[0], vrep.simx_opmode_streaming)

    rospy.Subscriber("/quats", quats, callback)

    rospy.spin()
    vrep.simxGetPingTime(clientID)
    vrep.simxFinish(clientID)

if __name__ == '__main__':
    listener()
