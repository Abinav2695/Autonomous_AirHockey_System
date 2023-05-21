#!/usr/bin/python3

# ROS inclusions
import rospy
from std_msgs.msg import Float32,Float64,Float32MultiArray

import sys
import cv2
import numpy as np

###Local module inclusions
import utils.config.config as constants

from utils.geometry_functions.geometry_functions import Vector2D as point2D
from utils.geometry_functions.geometry_functions import Circle as circle2D
from utils.geometry_functions.geometry_functions import Line as line2D

from utils.intersection_finder import Path_Prediction as PP
from utils.epson import Epson as EPR



## All are in MM(millimeter)
newPuckCenter=point2D(0,0)
previousPuckCenter = point2D(0,0)
newPredictedPoint = point2D(0,0)
previousPredictedPoint = point2D(0,0)
velocityVector = point2D(0,0)
velocityOfPuck=0
robotState=0
previousRobotState=0
shouldConsider=0

gameStarted=False

##object for intersection_finder class

maxReachCircleCenter_in_mm = point2D(0,0)
maxReachCircleCenter_in_pixel = constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"]
maxReachCircleCenter_in_mm.x = maxReachCircleCenter_in_pixel.x /constants.table["PIXEL_TO_MM_RATIO"]
maxReachCircleCenter_in_mm.y = maxReachCircleCenter_in_pixel.y / constants.table["PIXEL_TO_MM_RATIO"]
maxReachCircleRadius_in_pixel = constants.table["MAX_REACH_CIRCLE_RADIUS_IN_MM"]*constants.table["PIXEL_TO_MM_RATIO"]

maxReachCircle = circle2D(maxReachCircleCenter_in_mm,constants.table["MAX_REACH_CIRCLE_RADIUS_IN_MM"])
maxReachCircle_in_pixel =circle2D(maxReachCircleCenter_in_pixel,
                            maxReachCircleRadius_in_pixel)
pathOfPuck = PP()

robotHomePoint = point2D(constants.table["HOME_POSITION_Y_IN_MM"],constants.table["TABLE_HEIGHT_IN_MM"]*0.50)

newData=False
pubTopic = rospy.Publisher("/puckPrediction", Float32MultiArray, queue_size=1)
myRobot =EPR()


def scale(input_val,in_min,in_max, out_min,out_max) :
    """
    Function to scale the x,y coordinates in pixel to millimeter as per table dimensions
    Args:
        input_val ([float]): [x(or) y value of puck in pixel]
        in_min ([float]): [Min table width/height in pixel]
        in_max ([float]): [Max table width/height in pixel]
        out_min ([float]): [Min table width/height in mm]
        out_max ([float]): [Max table width/height in mm]

    Returns:
        [int]: [x (or) y value of puck in mm]
    """
    return int((((input_val - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min)


def velocityTracker(data):
    
    global robotState
    global previousRobotState
    global newPuckCenter
    global previousPuckCenter
    global velocityOfPuck
    global velocityVector
    global robotHomePoint
    global shouldConsider

    newPuckCenter.x = data.data[0]
    newPuckCenter.y =data.data[1]
    previousPuckCenter.x = data.data[2]
    previousPuckCenter.y = data.data[3]
    deltaTime = data.data[4]

    if(newPuckCenter.x == constants.INVALID_POINT.x):
        robotState=7
        shouldConsider=0
    
    elif(abs(newPuckCenter.x-previousPuckCenter.x)>400):
        robotState =7
        shouldConsider=0
    
    else:
        shouldConsider+=1
        if(shouldConsider>=2):
            velocityVector.x = (newPuckCenter.x-previousPuckCenter.x)/deltaTime
            velocityVector.y = (newPuckCenter.y-previousPuckCenter.y)/deltaTime
            velocityOfPuck = velocityVector.magnitude()
            velXMag = abs(velocityVector.x)
            velYMag = abs(velocityVector.y)
            
            if(velYMag>1200 and newPuckCenter.x>(0.73*constants.table["TABLE_WIDTH_IN_MM"])):
                robotState=7

            elif(velocityVector.x<0):
                
                if (velXMag>3500):
                    robotState = 1   
                elif (velXMag>2800):
                    robotState=2
                elif (velXMag>2000):
                    robotState=3
                elif (velXMag>1400):
                    robotState=4
                elif (velXMag>900):
                    robotState = 8

                elif(velXMag>700):
                    if(maxReachCircle.if_point_in_circle(newPuckCenter)):
                        robotState=5
                else:
                    if(maxReachCircle.if_point_in_circle(newPuckCenter)):
                        robotState=5
            
            elif(velocityVector.x>=0):
                if(maxReachCircle.if_point_in_circle(newPuckCenter) and velocityVector.x<700 and newPuckCenter.x>100):
                    
                    robotState=6
                else:
                    robotState =7
            
        #print(newPuckCenter.x,newPuckCenter.y)

def server_callback(server_data):

    global gameStarted
    global myRobot

    if(server_data.data=='start'):
        gameStarted=True
        myRobot.send_prog_start_command()
    
    elif(server_data.data=='stop'):
        gameStarted=False
        myRobot.send_prog_stop_command()
        
def path_finder():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global previousRobotState
    global robotState
    
    global myRobot
    global maxReachCircle

    global newPuckCenter
    global previousPuckCenter
    global velocityOfPuck
    global velocityVector
    global robotHomePoint
    global gameStarted

    rospy.init_node('velocity_node', anonymous=True)

    rospy.Subscriber("/puckCenter", Float32MultiArray, velocityTracker)
    
    home=0
    inAttackPosition =0
    prevHomeOrientation=0

    while not rospy.is_shutdown():
        
        if (gameStarted):
            predictedData=None
            if (robotState==1): 
                predictedData = pathOfPuck.find_puck_predicted_point(newPuckCenter,previousPuckCenter,
                                            intersectionCircleRadius=constants.table["DEFEND_CIRCLE1_RADIUS_IN_PIXEL"],isLine=0)
                home=0
                inAttackPosition =0

            elif (robotState==2):
                predictedData = pathOfPuck.find_puck_predicted_point(newPuckCenter,previousPuckCenter,
                                            intersectionCircleRadius=constants.table["DEFEND_CIRCLE2_RADIUS_IN_PIXEL"],isLine=0)

                home=0
                inAttackPosition =0
            
            elif (robotState==3):
                predictedData = pathOfPuck.find_puck_predicted_point(newPuckCenter,previousPuckCenter,
                                            intersection_line_x=constants.table["DEFEND_LINE_IN_PIXEL"],isLine=1)
                if(predictedData[0].y>850 or predictedData[0].y<240):
                    predictedData=None
                
                home=0
                inAttackPosition =0

            elif (robotState==4):
                predictedData = pathOfPuck.find_puck_predicted_point(newPuckCenter,previousPuckCenter,
                                            intersection_line_x=constants.table["DEFENSIVE_ATTACK_LINE_IN_PIXEL"],isLine=1)
                if(predictedData[0].y>850 or predictedData[0].y<240):
                    predictedData=None
                
                home=0
                inAttackPosition =0
                    
            elif (robotState==5):
                
                distanceToCover = velocityOfPuck*0.4
                puckVector = newPuckCenter.__sub__(previousPuckCenter)
                normalisedPuckvector = puckVector.__norm__()
                nextPoint = newPuckCenter.__add__(normalisedPuckvector.__mul__(distanceToCover))
                predictedData = (nextPoint,0)
                home=0
                inAttackPosition =0
            
            elif(robotState ==6):
                # if not inAttackPosition:
                #     if(newPuckCenter.y<constants.table["TABLE_HEIGHT_IN_MM"]*0.50):
                #         attackPoint = point2D(120,200)
                #         predictedData = (attackPoint,0)
                    
                #     else:
                #         attackPoint = point2D(120,800)
                #         predictedData = (attackPoint,0)
                    
                #     inAttackPosition=1
                #     home=0
                # else:
                distanceToCover = velocityOfPuck*0.5
                puckVector = newPuckCenter.__sub__(previousPuckCenter)
                normalisedPuckVector = puckVector.__norm__()
                nextPoint = newPuckCenter.__add__(normalisedPuckVector.__mul__(distanceToCover))
                print(nextPoint.x,nextPoint.y)
                robotLineOfMotion = line2D(point2 = nextPoint,point1 = robotHomePoint)
                robotGoToPoint = maxReachCircle.intersection_with_line(robotLineOfMotion)
                
                if robotGoToPoint is None:
                    predictedData=None
                # robotVector = nextPoint.__sub__(robotHomePoint)
                # normalisedRobotVector = robotVector.__norm__()
                # additionalDistane = 40
                #robotGoToPoint = nextPoint.__add__(normalisedRobotVector.__mul__(additionalDistane))

                else:
                    predictedData = (robotGoToPoint,0)
                
                inAttackPosition=0
                home=0

            elif (robotState==7 ):
                #center position

                homeOrientation=0
                if(newPuckCenter.y<constants.table["TABLE_HEIGHT_IN_MM"]*0.55 
                                        or newPuckCenter.x==constants.INVALID_POINT.x):
                    homeOrientation=0
                else:
                    homeOrientation=1
                if(prevHomeOrientation!=homeOrientation):
                    home=0
                    prevHomeOrientation=homeOrientation

                if(home==0 ):       # || (abs(myRobot.get_curr_xy()[0] - constants.table["TABLE_HEIGHT_IN_MM"]*0.55)>10)
                    point1 = point2D(0,0)
                    point1.x = constants.table["HOME_POSITION_Y_IN_MM"]
                    if(homeOrientation):
                        
                        point1.y = constants.table["TABLE_HEIGHT_IN_MM"]*0.52
                    else:
                        point1.y = constants.table["TABLE_HEIGHT_IN_MM"]*0.50
                    predictedData = (point1,0)
                    home=1
                    inAttackPosition=0
                    print('Not homed')

            elif (robotState==8):
                if(newPuckCenter.x<0.5*constants.table["TABLE_WIDTH_IN_MM"]):
                    predictedData = pathOfPuck.find_puck_predicted_point(newPuckCenter,previousPuckCenter,
                                                intersectionCircleRadius=maxReachCircleRadius_in_pixel,isLine=0)
                    home=0
                    inAttackPosition =0


            #print(robotState)

            if predictedData is not None:
                newPredictedPoint = predictedData[0]
                if(newPredictedPoint.x!=constants.INVALID_POINT.x and (newPuckCenter.x<(0.85*constants.table["TABLE_WIDTH_IN_MM"]) or robotState==7)):
                    
                    if(newPredictedPoint.x < 140):
                        newPredictedPoint.x = 140
                    if(newPredictedPoint.y>1000):
                        newPredictedPoint.y = 1000
                    elif(newPredictedPoint.y<150):
                        newPredictedPoint.y = 160
                    distance = predictedData[1]
                    dataToPublish = Float32MultiArray()
                    dataToPublish.data.append(newPredictedPoint.x)
                    dataToPublish.data.append(newPredictedPoint.y)
                    pubTopic.publish(dataToPublish)
            
                    myRobot.go_to_position(int(newPredictedPoint.y),int(newPredictedPoint.x),constants.epson["DEFAULT_Z_POSITION"],
                                ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=100,JUMP_FLAG=0)
                    while(myRobot.is_bot_inmotion()):
                        pass
                    
                    if(robotState!=7 and robotState!=6 and robotState!=1 and maxReachCircle.if_point_in_circle(newPuckCenter)):
                        myRobot.go_to_position(int(newPredictedPoint.y),(int(newPredictedPoint.x)+200),constants.epson["DEFAULT_Z_POSITION"],
                                    ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=100,JUMP_FLAG=0)
                        while(myRobot.is_bot_inmotion()):
                            pass
                    robotState=0
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

if __name__ == '__main__':

    
    path_finder()

