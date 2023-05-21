#!/usr/bin/python3
import rospy

import cv2
import numpy as np
import os
import time
import argparse
import imutils
import sys

import utils.config.config as constants
from utils.geometry_functions.geometry_functions import Vector2D as point2D
#from utils.intersection_finder import Path_Prediction as pp
#from utils.epson import Epson as EPR

# ROS inclusions
from std_msgs.msg import Float32,Float64,Float32MultiArray,String
from threading import Thread

## Camera Variables

videoPort=0
scaledXinMM=0
scaledYinMM=0
#robotNew =EPR()
newPosition=0
home =0
hand=0
gameStarted=False
###Take video source arguments from the user



parser = argparse.ArgumentParser()
parser.add_argument("-v","--videoSource", help="Input User Video Source ..0/1/2")
parser.add_argument("-f","--fileName", help="name of the video file ..eg:- video1.webm")
args = parser.parse_args(sys.argv[1:-2])
print('Number of arguments:', len(sys.argv), 'arguments. Argument List:',(sys.argv))

portFound=0
camera=None

if(args.fileName):
    videoPort=args.fileName
    portFound=1
    
elif(args.videoSource):
    videoPort=args.videoSource
    portFound=1
    cmd = "v4l2-ctl --device=/dev/video" + str(videoPort) + " --set-fmt-video=width=640,height=480,pixelformat=0"
    os.system(cmd)
    cmd = "v4l2-ctl --device=/dev/video" + str(videoPort) + " --set-parm=60"
    os.system(cmd)

else:
    portFound=0
    for ports in range(10):
        
        camera = cv2.VideoCapture(ports)
        if(camera is None or not camera.isOpened()):
            print('NO video source in port ',ports)
            portFound=0
            
        else:
            portFound=1
        if(portFound):
            videoPort = ports
            cmd = "v4l2-ctl --device=/dev/video" + str(videoPort) + " --set-fmt-video=width=640,height=480,pixelformat=0"
            os.system(cmd)
            cmd =  "v4l2-ctl --device=/dev/video" + str(videoPort) + " --set-parm=60"
            os.system(cmd)
            break

videoPort =int(videoPort)

newPredictedPoint = point2D(0,0)
previousPredictedPoint = point2D(0,0)

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




'''  Currently Not used -------------------------------
def robot_thread():
    """
        Thread for sending x,y coordinate data to move to a specific position 
        The funtion waits before giving next coordinate while the bot is in motion
        Position vlaues and other flags are updated as global variables
    """   
    global scaledXinMM
    global scaledYinMM
    #global robotNew
    global newPosition
    global home
    global hand

    while(True):
        if(newPosition==1):

            robotNew.go_to_position(scaledXinMM,scaledYinMM,0,
                              ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=100,JUMP_FLAG=0)
            # robotNew.go_to_position(scaledXinMM,scaledYinMM,0,
            #                   ORIENTATION=hand,SPEED_FACTOR=100,JUMP_FLAG=0)
            
            while(robotNew.is_bot_inmotion()):
                pass
            newPosition=0
        elif(home==0):
            robotNew.go_to_position(int(constants.table["TABLE_HEIGHT_IN_MM"]/2),constants.table["HOME_POSITION_Y_IN_MM"],0,
                                    ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=100,JUMP_FLAG=0)
            # robotNew.go_to_position(int(constants.table["TABLE_HEIGHT_IN_MM"]/2),165,0,
            #                         ORIENTATION=hand,SPEED_FACTOR=100,JUMP_FLAG=0)
            home=1
        time.sleep(0.001)


'''


def predictionUpdate(predictedData):
    global newPredictedPoint
    global previousPredictedPoint

    #newPredictedPoint.x=predictedData.data[0]
    #newPredictedPoint.y=predictedData.data[1]

    newPredictedPoint.y= scale(predictedData.data[1],0,constants.table["TABLE_HEIGHT_IN_MM"],
                                                    0,constants.table["TABLE_HEIGHT_IN_PIXEL"])

    newPredictedPoint.x= scale(predictedData.data[0],0,constants.table["TABLE_WIDTH_IN_MM"],
                                                    0,constants.table["TABLE_WIDTH_IN_PIXEL"])
    #print(newPredictedPoint.x,newPredictedPoint.y)


def server_callback(server_data):
    global gameStarted

    if(server_data.data=='start'):
        gameStarted=True
        time.sleep(2)
    
    elif(server_data.data=='stop'):
        gameStarted=False


def puck_detector():
    """
    Main Funtion which detects puck and updates the x/y position of the same in pixel values
    The data is published on /puckCenter Topic
    """


    global camera
    global scaledXinMM
    global scaledYinMM
    #global robotNew
    global newPosition
    global home
    global hand
    global newPredictedPoint
    global previousPredictedPoint
    global gameStarted

    if(portFound):
        
        #os.system('ls /dev')

        if(camera==None):
            camera = cv2.VideoCapture(videoPort)

        print('Current Video Port: ',videoPort)
        print('CameraObject :',camera)
            #camera.set(videoPort,cv2.CAP_DSHOW)
        
        fourcc = cv2.VideoWriter_fourcc('Y','U','Y','V')
        camera.set(cv2.CAP_PROP_FOURCC, fourcc)
        camera.set(cv2.CAP_PROP_FPS,constants.camera["FORCE_FPS"])
        camera.set(cv2.CAP_PROP_FRAME_WIDTH,constants.camera["CAM_WIDTH"])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT,constants.camera["CAM_HEIGHT"])

        newPuckCenter=point2D(0,0)
        previousPuckCenter = point2D(0,0)
        

        newPuckCenter_MM = point2D(0,0)
        previousPuckCenter_MM = point2D(0,0)

        puckRadius=0
        home=0
        previousTime=0
        
        #path1 = pp(constants.table["TABLE_WIDTH_IN_PIXEL"],constants.table["TABLE_HEIGHT_IN_PIXEL"]
        #                                ,constants.table["TABLE_WIDTH_ZERO_IN_PIXEL"],constants.table["TABLE_HEIGHT_ZERO_IN_PIXEL"])
        
        # print(scale(65,0,constants.table["TABLE_WIDTH_IN_PIXEL"],
        #                         0,constants.table["TABLE_WIDTH_IN_MM"]))

        

        while not rospy.is_shutdown(): 
        #while True:
            
            # Capture the video frame 
            # by frame
            
            if(gameStarted):
                ret, frame = camera.read() 
            else:
                ret=0
            
            if(ret==0):
                return
            #frame = cv2.rotate(frame, cv2.cv2.ROTATE_180) 
            frame = imutils.rotate(frame,-1)
            YUV_Image=cv2.cvtColor(frame,cv2.COLOR_BGR2YUV)
            Y,U,V=cv2.split(YUV_Image)
            eq_V=cv2.equalizeHist(V)
            eq_Y=cv2.equalizeHist(Y)
            eq_U=cv2.equalizeHist(U)
            

            ##### Merge the equalized channels with the original image
            HIST_EQ_IMAGE=cv2.merge((eq_Y,U,V),YUV_Image)
            BGR_IMAGE=cv2.cvtColor(HIST_EQ_IMAGE,cv2.COLOR_YUV2BGR)
            
            
            
            
        
            ## Considering ROI for image processing
            FRAME_FOR_USER = frame[int(constants.cornerA.y):int(constants.cornerD.y) , int(constants.cornerA.x):int(constants.cornerB.x)]   
            ROI_FOR_IP = BGR_IMAGE[int(constants.cornerA.y):int(constants.cornerD.y) , int(constants.cornerA.x):int(constants.cornerB.x)]
            
            ## Convert img to hsv and detect object based on hsv values of puck

            hsv = cv2.cvtColor(ROI_FOR_IP, cv2.COLOR_BGR2HSV)         #convert to hsv
            mask = cv2.inRange(hsv, np.array(constants.hsv_data["HSV_LOW"]), np.array(constants.hsv_data["HSV_HIGH"])) #mask for red puck


            res = cv2.bitwise_and(ROI_FOR_IP,ROI_FOR_IP, mask= mask)  # overlaying on original image
            
            #contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            
            puckFound=0
            for i in contours:
                area = cv2.contourArea(i)
                perimeter = cv2.arcLength(i,True)
                
                if(area>constants.puck["PUCK_MIN_AREA"] and area<constants.puck["PUCK_MAX_AREA"]):        
                    roundness = (perimeter*perimeter)/(6.28*area)
                    
                    if (roundness < 5):
                        
                        M=cv2.moments(i)
                        
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        puckFound=1
                        ((newPuckCenter.x, newPuckCenter.y), puckRadius) = cv2.minEnclosingCircle(i)
                        currentTime = time.time()

                        newPuckCenter_MM.y = scale(newPuckCenter.y,0,constants.table["TABLE_HEIGHT_IN_PIXEL"],
                                                                0,constants.table["TABLE_HEIGHT_IN_MM"])
                        newPuckCenter_MM.x = scale(newPuckCenter.x,0,constants.table["TABLE_WIDTH_IN_PIXEL"],
                                                                0,constants.table["TABLE_WIDTH_IN_MM"])

                        '''
                        The below block of code is implemented in another node in ROS

                        # if(newPuckCenter.x<(previousPuckCenter.x-15) and newPuckCenter.x >constants.table["DEFEND_LINE_IN_PIXEL"] ):
                        #     #for defend line
                        #     #newPredictedPoint = path1.find_puck_predicted_point(newPuckCenter,previousPuckCenter)

                        #     #for defend circle
                            
                        #     newPredictedPoint = path1.find_puck_predicted_point(newPuckCenter,previousPuckCenter,isLine=0)
                            

                        #     if (newPredictedPoint.x!=constants.INVALID_POINT.x):
                        #         # cv2.circle(FRAME_FOR_USER, (int(newPredictedPoint.x),int(newPredictedPoint.y)), 5,(200,150, 200), 2)
                        #         if(newPredictedPoint.x<30):
                        #             newPredictedPoint.x=30
                        #         if(50<newPredictedPoint.y<250):# and abs(newPredictedPoint.y - previousPredictedPoint.y)>15):
                                    
                        #             cv2.circle(FRAME_FOR_USER, (int(newPredictedPoint.x),int(newPredictedPoint.y)), 5,(100,150, 100), 2)
                        #             scaledXinMM = scale(newPredictedPoint.y,0,constants.table["TABLE_HEIGHT_IN_PIXEL"],
                        #                                                  0,constants.table["TABLE_HEIGHT_IN_MM"])
                        #             scaledYinMM =  scale(newPredictedPoint.x,0,constants.table["TABLE_WIDTH_IN_PIXEL"],
                        #                                                   0,constants.table["TABLE_WIDTH_IN_MM"])
                        #             #print(scaledXinMM,scaledYinMM)

                        #             if(newPuckCenter.x<(0.75*constants.table["TABLE_WIDTH_IN_PIXEL"])):
                        #                 # if(newPredictedPoint.y>(constants.table["TABLE_HEIGHT_IN_PIXEL"]/2)):
                        #                 #     hand=1
                        #                 # else:
                        #                 #     hand=0
                        #                 newPosition=1
                        #                 home=0
                                        
                        #     previousPredictedPoint.x,previousPredictedPoint.y = newPredictedPoint.x,newPredictedPoint.y
                        #     previousPuckCenter.x,previousPuckCenter.y = newPuckCenter.x,newPuckCenter.y
                            
                        # elif(newPuckCenter.x>(previousPuckCenter.x)):
                        #     previousPuckCenter.x,previousPuckCenter.y = newPuckCenter.x,newPuckCenter.y
                            
                        '''


                        #---------------------------ROS INCLUSIONS--------------------------------#
                        msg =Float32MultiArray()
                        
                        # msg.data.append(newPuckCenter.x)
                        # msg.data.append(newPuckCenter.y)
                        # msg.data.append(previousPuckCenter.x)
                        # msg.data.append(previousPuckCenter.y)
                        # msg.data.append(currentTime-previousTime)

                        msg.data.append(newPuckCenter_MM.x)
                        msg.data.append(newPuckCenter_MM.y)
                        msg.data.append(previousPuckCenter_MM.x)
                        msg.data.append(previousPuckCenter_MM.y)
                        msg.data.append(currentTime-previousTime)

                        pubTopic.publish(msg)
                        previousPuckCenter.x,previousPuckCenter.y = newPuckCenter.x,newPuckCenter.y
                        previousPuckCenter_MM.x,previousPuckCenter_MM.y= newPuckCenter_MM.x,newPuckCenter_MM.y
                        previousTime=currentTime

                        ## Enclosing circle for detected puck
                        cv2.circle(FRAME_FOR_USER, (int(newPuckCenter.x),int( newPuckCenter.y)), int(puckRadius),(250, 0, 0), 2)
                        #print(int(newPuckCenter.x),int( newPuckCenter.y))
                        break
                    
                    

                        


            if(puckFound==0):
                #---------------------------ROS INCLUSIONS--------------------------------#
                msg =Float32MultiArray()
                
                # msg.data.append(newPuckCenter.x)
                # msg.data.append(newPuckCenter.y)
                # msg.data.append(previousPuckCenter.x)
                # msg.data.append(previousPuckCenter.y)
                # msg.data.append(currentTime-previousTime)

                msg.data.append(constants.INVALID_POINT.x)
                msg.data.append(constants.INVALID_POINT.y)
                msg.data.append(previousPuckCenter_MM.x)
                msg.data.append(previousPuckCenter_MM.y)
                msg.data.append(0)

                pubTopic.publish(msg)

             ## Boundary of table for reference drawn on original frame (A and C points as corner)
            start_point = (int(constants.cornerA.x),int(constants.cornerA.y))
            end_point = (int(constants.cornerC.x),int(constants.cornerC.y))

            cv2.rectangle(frame, start_point,end_point, color=(0, 255, 0), thickness=1)
            
            
            ## defense line
            cv2.line(FRAME_FOR_USER, pt1=(int(constants.table["DEFEND_LINE_IN_PIXEL"]),0), 
                        pt2=(int(constants.table["DEFEND_LINE_IN_PIXEL"]),int(constants.table["TABLE_HEIGHT_IN_PIXEL"])), 
                        color=(255, 255, 0), thickness=1)
            
            ##defensive attack line
            cv2.line(FRAME_FOR_USER, pt1=(int(constants.table["DEFENSIVE_ATTACK_LINE_IN_PIXEL"]),0), 
                        pt2=(int(constants.table["DEFENSIVE_ATTACK_LINE_IN_PIXEL"]),int(constants.table["TABLE_HEIGHT_IN_PIXEL"])), 
                        color=(255, 255, 0), thickness=1)

            ## Player Cutoff line
            cv2.line(FRAME_FOR_USER, pt1=(int(0.85*constants.table["TABLE_WIDTH_IN_PIXEL"]),0), 
                        pt2=(int(0.85*constants.table["TABLE_WIDTH_IN_PIXEL"]),int(constants.table["TABLE_HEIGHT_IN_PIXEL"])), 
                        color=(255, 255, 0), thickness=1)
            
            ## defend circle 1            
            cv2.circle(FRAME_FOR_USER,(int(constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"].x),int(constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"].y)),
                                            constants.table["DEFEND_CIRCLE1_RADIUS_IN_PIXEL"],(100, 0,0), 2)
            
            ##defend circle 2            
            cv2.circle(FRAME_FOR_USER,(int(constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"].x),int(constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"].y)),
                                            constants.table["DEFEND_CIRCLE2_RADIUS_IN_PIXEL"],(100, 0,0), 2)

            ## Max Reach circle
            cv2.circle(FRAME_FOR_USER,(int(constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"].x),int(constants.table["DEFEND_CIRCLE_CENTER_IN_PIXEL"].y)),
                                            int(constants.table["MAX_REACH_CIRCLE_RADIUS_IN_MM"]*constants.table["PIXEL_TO_MM_RATIO"]),(0, 0,255), 2)

            cv2.circle(FRAME_FOR_USER,(int(newPredictedPoint.x),int(newPredictedPoint.y)), 5,(0,255, 250), 2)
            
            cv2.imshow('frame', FRAME_FOR_USER) 
            # the 'q' button is set as the 
            # quitting button you may use any 
            # desired button of your choice 
            
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break
            
           
            
        # After the loop release the cap object 
        camera.release() 
        # Destroy all the windows 
        cv2.destroyAllWindows() 

        


    else:
        print('Sorry Cannot Open any Video Source')

if __name__ == '__main__':

    #t1=Thread(target=robot_thread,daemon=True)
    #t1.start()
    ### ROS Topics

    rospy.init_node("puck_center_publisher_node")
    pubTopic = rospy.Publisher("/puckCenter", Float32MultiArray, queue_size=1)
    time.sleep(2)
    rospy.Subscriber("/puckPrediction", Float32MultiArray, predictionUpdate)
    rospy.Subscriber('/server_events', String,server_callback)
    puck_detector()
    
