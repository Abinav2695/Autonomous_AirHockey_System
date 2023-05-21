#!/usr/bin/env python3

import time


import rospkg
import rospy
import sys
import serial


from std_msgs.msg import String


GAME_TIMEOUT=190



def get_current_timestamp():
    currTime = rospy.Time.now()
    currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
    return(currTime)


def game_callback(calldata):


    if(calldata.data=='stop'):
        ESP_DATA.write(b'stop;')
    
    elif(calldata.data[0]=='g'):
        data = calldata.data+';'
        ESP_DATA.write(bytes(data,'utf-8'))


if __name__ == "__main__":
    
    rospy.init_node('socket_node', anonymous=False)
    rospy.Subscriber('/game_topic',String,game_callback)
    
    server_pub = rospy.Publisher('/server_events', String, queue_size=5)

    #The following line is for serial over GPIO
    try:
        port = '/dev/ttyUSB0'
    except:
        print('cannot open')


    ESP_DATA = serial.Serial(port,115200,timeout=2)
    #ESP_DATA.open()    

    spinRate = rospy.Rate(100)
    
    start_time=get_current_timestamp()
    while not rospy.is_shutdown():
        
        newData = ESP_DATA.readline()
        newData = newData.decode("utf-8")
        newData=newData.split('\r')


        if(newData[0]=='start'):
            
            print('START')
            server_pub.publish("start")
            start_time = get_current_timestamp()


        elif(newData[0]=='stop'):
            print('STOP')
            server_pub.publish("stop")
        
        if(get_current_timestamp()-start_time>GAME_TIMEOUT):
            print('STOP')
            server_pub.publish("stop")
            ESP_DATA.write('stop;')
            
        spinRate.sleep()



