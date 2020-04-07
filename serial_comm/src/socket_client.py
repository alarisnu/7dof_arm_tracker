#!/usr/bin/env python

import numpy
import socket
import rospy
from serial_comm.msg import Pos_Rot

def main():
    pub = rospy.Publisher('/WristPosRot', Pos_Rot, queue_size=3)
    rospy.init_node('SocketClient', anonymous=True)
    HOST = '10.1.70.33'    # The remote host
    PORT = 45678              # The same port as used by the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    while True:
        data = s.recv(1024)
        data_splitted = data.split(';')
        pos_rot_msg = Pos_Rot()
        if len(data_splitted) == 8:
            pos_rot_msg.pos.x = float(data_splitted[1])*3
            pos_rot_msg.pos.y = float(data_splitted[2])*3
            pos_rot_msg.pos.z = float(data_splitted[3])*3+1
            
            pos_rot_msg.rot.w = float(data_splitted[4])
            pos_rot_msg.rot.x = float(data_splitted[5])
            pos_rot_msg.rot.y = float(data_splitted[6])
            pos_rot_msg.rot.z = float(data_splitted[7])
            pub.publish(pos_rot_msg);
        s.sendall(b'OK')
    s.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
