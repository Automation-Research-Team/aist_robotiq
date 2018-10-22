#!/usr/bin/env python
#from __future__ import print_function
import rospy
import can
import cv2
import numpy as np
import time

def send_one():
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

    msg = can.Message(arbitration_id=0x201, data=[7], extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        print("Message NOT sent")

    count = 0
    base = [0] * 48

    while True :
        flag = False
        id_list = [0] * 16
        data_list = [0] * 16

        for j in range(0, 16):
            recvmsg = bus.recv()
    
            if recvmsg.arbitration_id >= 1808: 
                id_list[recvmsg.arbitration_id - 1808] = recvmsg.arbitration_id
                if recvmsg.data[1] < 250 :
                    flag = True
                    data_list[recvmsg.arbitration_id - 1808] = recvmsg.data
                else :
                    flag = False
                    break
    
        if flag:
            j = 0
            for i in range(0,48,3) :
                base[i] = base[i] + (data_list[j][1] << 8 | data_list[j][2])
                base[i+1] = base[i+1] + (data_list[j][3] << 8 | data_list[j][4])
                base[i+2] = base[i+2] + (data_list[j][5] << 8 | data_list[j][6])
                j = j + 1
            count = count + 1
        
        if count > 100:
            break
        
    for i in range(0,48,3) :
        base[i] = base[i] / 100
        base[i+1] = base[i+1] / 100
        base[i+2] = base[i+2] / 100

    


    initialx = 225
    scale = 50
    
    x_axis = [0] * 16
    y_axis = [0] * 16
    z_axis = [0] * 16
    z_scale = 300
    img = np.zeros((680,680,3), np.uint8)
    
    tz = 12
    z = [tz] * 16
    
    while True:
        cv2.circle(img,(120,120), z[0], (0,255,0), -1)
        cv2.circle(img,(120,250), z[1], (0,255,0), -1)
        cv2.circle(img,(120,380), z[2], (0,255,0), -1)
        cv2.circle(img,(120,510), z[3], (0,255,0), -1)

        cv2.circle(img,(250,120), z[4], (0,255,0), -1)
        cv2.circle(img,(250,250), z[5], (0,255,0), -1)
        cv2.circle(img,(250,380), z[6], (0,255,0), -1)
        cv2.circle(img,(250,510), z[7], (0,255,0), -1)

        cv2.circle(img,(380,120), z[8], (0,255,0), -1)
        cv2.circle(img,(380,250), z[9], (0,255,0), -1)
        cv2.circle(img,(380,380), z[10], (0,255,0), -1)
        cv2.circle(img,(380,510), z[11], (0,255,0), -1)

        cv2.circle(img,(510,120), z[12], (0,255,0), -1)
        cv2.circle(img,(510,250), z[13], (0,255,0), -1)
        cv2.circle(img,(510,380), z[14], (0,255,0), -1)
        cv2.circle(img,(510,510), z[15], (0,255,0), -1)


        cv2.imshow("Image window", img)

        if cv2.waitKey(1) > 0:
            cv2.destroyAllWindows()
            break

        while True :
            flag = False
            id_list = [0] * 16
            data_list = [0] * 16
        
            for j in range(0, 16):
                recvmsg = bus.recv()
        
                if recvmsg.arbitration_id >= 1808:
                    id_list[recvmsg.arbitration_id - 1808] = recvmsg.arbitration_id
                    if recvmsg.data[1] < 250 :
                        flag = True
                        data_list[recvmsg.arbitration_id - 1808] = recvmsg.data
                    else :
                        flag = False
                        break
        
            if flag:
                for i in range(0,16) :
                    if id_list[i] != 0:
                        x_axis[i] = data_list[i][1] << 8 | data_list[i][2]
                        y_axis[i] = data_list[i][3] << 8 | data_list[i][4]
                        z_axis[i] = data_list[i][5] << 8 | data_list[i][6]
                break
        
        cv2.circle(img,(120,120), z[0], (0,0,0), -1)
        cv2.circle(img,(120,250), z[1], (0,0,0), -1)
        cv2.circle(img,(120,380), z[2], (0,0,0), -1)
        cv2.circle(img,(120,510), z[3], (0,0,0), -1)

        cv2.circle(img,(250,120), z[4], (0,0,0), -1)
        cv2.circle(img,(250,250), z[5], (0,0,0), -1)
        cv2.circle(img,(250,380), z[6], (0,0,0), -1)
        cv2.circle(img,(250,510), z[7], (0,0,0), -1)

        cv2.circle(img,(380,120), z[8], (0,0,0), -1)
        cv2.circle(img,(380,250), z[9], (0,0,0), -1)
        cv2.circle(img,(380,380), z[10], (0,0,0), -1)
        cv2.circle(img,(380,510), z[11], (0,0,0), -1)

        cv2.circle(img,(510,120), z[12], (0,0,0), -1)
        cv2.circle(img,(510,250), z[13], (0,0,0), -1)
        cv2.circle(img,(510,380), z[14], (0,0,0), -1)
        cv2.circle(img,(510,510), z[15], (0,0,0), -1)

        n = 2
        for i in range(0,16):
            z[i] = ((z_axis[i] - base[n]) / z_scale)
            z[i] = tz + z[i]
            n = n + 3


if __name__ == '__main__':
    rospy.init_node('xela_sensor_demo', anonymous=True)
    send_one()

    rospy.spin()
