#!/usr/bin/python3
import json
from utils.geometry_functions.geometry_functions import Vector2D as point2D
import rospkg



#### calibration points
    #       --->X(WIDTH)
    #       | 
    #       *
    #       Y(HEIGHT)
    #
    #               a ---------------------- b
    #                 |                    |
    #  Robot -->      |        Table       | -->Player
    #                 |                    |
    #               d ---------------------- c


# ## Calibrate these points manually and update accordingly

cornerA = point2D(8,68) 
cornerB = point2D(635,68) 
cornerC = point2D(635,366)      
cornerD = point2D(8,366)    

## JSON File Path
rospack1 = rospkg.RosPack()
hsv_json_file_path  = rospack1.get_path('utils') +'/src/utils/config/hsv.json'
## ERROR POINT FOR NULL PREDICTION
INVALID_POINT = point2D(9999,9999)

with open(hsv_json_file_path) as json_data_file:
    data = json.load(json_data_file)
json_data_file.close()




camera={
    "FORCE_FPS":60,
    "CAM_WIDTH":640,
    "CAM_HEIGHT" :480
}

table={
    "TABLE_WIDTH_IN_MM" :2330,
    "TABLE_HEIGHT_IN_MM" :1115,

    "TABLE_WIDTH_IN_PIXEL" :(cornerB.x-cornerA.x),
    "TABLE_HEIGHT_IN_PIXEL" :(cornerD.y-cornerA.y),
    "TABLE_WIDTH_ZERO_IN_PIXEL" :0,
    "TABLE_HEIGHT_ZERO_IN_PIXEL" :0,
    
    "NOT_REACHABLE_CIRCLE_RADIUS_IN_MM":155,
    "MAX_REACH_CIRCLE_RADIUS_IN_MM":580,
    "HOME_POSITION_Y_IN_MM":165,

    "DEFEND_LINE_IN_PIXEL":70,
    "DEFEND_CIRCLE1_RADIUS_IN_PIXEL":55,
    "DEFEND_CIRCLE2_RADIUS_IN_PIXEL":80,
    "DEFENSIVE_ATTACK_LINE_IN_PIXEL":88,
    "DEFEND_CIRCLE_CENTER_IN_PIXEL":point2D(0,(cornerD.y-cornerA.y)/2),
    
    "PIXEL_TO_MM_RATIO": ((((cornerB.x-cornerA.x)/2330.0)+((cornerD.y-cornerA.y)/1115.0))/2.0)

}

hsv_data={
    "HSV_LOW":data["HSV_LOW"],
    "HSV_HIGH":data["HSV_HIGH"]
}


puck={
    "PUCK_MIN_AREA":150,
    "PUCK_MAX_AREA":1000,
    "MAX_REBOUND_TRIES":3
} 

modbus={
    "IP":'192.168.0.10',
    "PORT":502,
    "TIMEOUT":10  #in seconds
}

epson={
    "GO_JUMP_MAX_SPEED":100,
    "DEFAULT_HAND_ORIENTATION":1, #1_-->RIghty 0--> Lefty
    "DEFAULT_SPEED_FACTOR":80,  # speed facotr is percentage of max speed 
                               #(eg: if max speed is 80 and speed factor is 50% then actaul speed wrt 100% speed of teh robot is 50% of 80 -->40%)
    "RIGHTY":1,
    "LEFTY":0,
    "DEFAULT_JUMP_FLAG":0,
    "DEFAULT_Z_POSITION":0
}

fieldbus_input={
    "FIELDBUS_INPUT_START_ADDR_BIT":511,
    "FIELDBUS_INPUT_START_ADDR_BYTE":63,
    "FIELDBUS_INPUT_START_ADDR_WORD":31,

    "SIGNX_ADDR":511,
    "SIGNY_ADDR":512,
    "SIGNZ_ADDR":513,
    "NEW_DATA_ADDR":514,
    "ABORT_COMMAND_ADDR":515,
    "HAND_ORIENTATION_ADDR":516,
    "PLAY_ENABLE_ADDR":517 ,
    "HOME_ADDR":518,
    "JUMP_FLAG_ADDR":519,
    "START_PROG_ADDR":521,
    "STOP_PROG_ADDR":522,
    "SET_MOTORS_ON_ADDR":523,
    "SET_MOTORS_OFF_ADDR":524,

    "X_POS_ADDR_WORD":34,
    "Y_POS_ADDR_WORD":35,
    "Z_POS_ADDR_WORD":36,
    "SPEED_FACTOR":37


}

fieldbus_output={
    "FIELDBUS_OUTPUT_START_ADDR_BIT":511,
    "FIELDBUS_OUTPUT_START_ADDR_BYTE":63,
    "FIELDBUS_OUTPUT_START_ADDR_WORD":31,

    "IN_MOTION_STATUS_ADDR":511,
    "NEW_DATA_ACK_ADDR":512,
    "HOME_ACK_ADDR":513,
    "PLAY_ENABLE_ACK_ADDR":514,
    "ABORT_MOTION_ACK_ADDR":515,

    "CURR_SIGNX_ADDR":516,
    "CURR_SIGNY_ADDR":517,
    "CURR_SIGNZ_ADDR":518,
    
    "CURR_X_POSITION_WORD":34,
    "CURR_Y_POSITION_WORD":35,
    "CURR_Z_POSITION_WORD":36,

}

