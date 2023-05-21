import numpy as np
import cv2
import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument("-v","--videoSource", help="Input User Video Source ..0/1/2")
parser.add_argument("-f","--fileName",help = "Pass the video file name")
args = parser.parse_args()



def nothing(x):
    pass

def colormask(videoSource):
    cv2.namedWindow('hsv_trackbar',1)

    #set trackbar
    hh = 'hue high'
    hl = 'hue low'
    sh = 'saturation high'
    sl = 'saturation low'
    vh = 'value high'
    vl = 'value low'
    mode = 'mode'

    #set ranges
    cv2.createTrackbar(hh, 'hsv_trackbar', 0,179, nothing)
    cv2.createTrackbar(hl, 'hsv_trackbar', 0,179, nothing)
    cv2.createTrackbar(sh, 'hsv_trackbar', 0,255, nothing)
    cv2.createTrackbar(sl, 'hsv_trackbar', 0,255, nothing)
    cv2.createTrackbar(vh, 'hsv_trackbar', 0,255, nothing)
    cv2.createTrackbar(vl, 'hsv_trackbar', 0,255, nothing)
    cv2.createTrackbar(mode, 'hsv_trackbar', 0,3, nothing)

    thv= 'th1'
    cv2.createTrackbar(thv, 'hsv_trackbar', 127,255, nothing)

    #read img in both rgb and grayscale
    # img = cv2.imread(filename,1)
    # imgg = cv2.imread(filename,0)

    #convert rgb to hsv
    
    camera = cv2.VideoCapture(videoSource)
    

    while True:

        ret, frame = camera.read() 
        if(ret):
            ##### Do histogram equalization on YUV channels
            gray  = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            YUV_Image=cv2.cvtColor(frame,cv2.COLOR_BGR2YUV)
            Y,U,V=cv2.split(YUV_Image)
            eq_V=cv2.equalizeHist(V)
            eq_Y=cv2.equalizeHist(Y)
            eq_U=cv2.equalizeHist(U)
            

            ##### Merge the equalized channels with the original image
            HIST_EQ_IMAGE=cv2.merge((eq_Y,U,V),YUV_Image)
            BGR_IMAGE=cv2.cvtColor(HIST_EQ_IMAGE,cv2.COLOR_YUV2BGR)

            hsv_img = cv2.cvtColor(BGR_IMAGE, cv2.COLOR_BGR2HSV)

            hul= cv2.getTrackbarPos(hl,'hsv_trackbar')
            huh= cv2.getTrackbarPos(hh,'hsv_trackbar')
            sal= cv2.getTrackbarPos(sl,'hsv_trackbar')
            sah= cv2.getTrackbarPos(sh,'hsv_trackbar')
            val= cv2.getTrackbarPos(vl,'hsv_trackbar')
            vah= cv2.getTrackbarPos(vh,'hsv_trackbar')
            thva= cv2.getTrackbarPos(thv,'hsv_trackbar')

            modev= cv2.getTrackbarPos(mode,'hsv_trackbar')

            hsvl = np.array([hul, sal, val], np.uint8)
            hsvh = np.array([huh, sah, vah], np.uint8)

            mask = cv2.inRange(hsv_img, hsvl, hsvh)

            res = cv2.bitwise_and(frame, frame, mask=mask)

            #set image for differnt modes
            ret, threshold = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            ret, img_th= cv2.threshold(gray, thva, 255, cv2.THRESH_TOZERO)
            res2 = cv2.bitwise_and(img_th, img_th, mask=threshold)
            res_rgb = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
            #convert black to white
            res[np.where((res==[0,0,0]).all(axis=2))] = [255,255,255]

            if modev ==0:
                #show mask only
                cv2.imshow('hsv_trackbar',mask)
            elif modev ==1:
                #show white-masked color img
                cv2.imshow('hsv_trackbar',res)
            elif modev ==2:
                #show white-masked binary img with threshold
                cv2.imshow('hsv_trackbar',threshold)
            else:
                #white-masked grayscale img with threshold
                cv2.imshow('hsv_trackbar',res2)

        #press 'Esc' to close the window
        ch = cv2.waitKey(5)
        if ch== 27:
            break
    cv2.destroyAllWindows()

    with open("hsv.json") as json_data_file:
        data = json.load(json_data_file)
    data["HSV_LOW"] = [hul, sal, val]
    data["HSV_HIGH"] = [huh, sah, vah]

    with open("hsv.json", "w") as outfile:
        json.dump(data, outfile)

if(args.videoSource):
    colormask(int(args.videoSource))
elif(args.fileName):
    colormask(args.fileName)
else:
    print("Please pass filename or VideoSource")