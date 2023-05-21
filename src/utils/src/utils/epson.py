#!/usr/bin/python3
import utils.config.config as constants
from datetime import datetime
import time
from pymodbus.client.sync import ModbusTcpClient

class Epson():
    def __init__(self):
        self.client = ModbusTcpClient(constants.modbus["IP"],constants.modbus["PORT"]
                            ,timeout=constants.modbus["TIMEOUT"])
        self.client.connect()
        print("Modbus Client : ",self.client)

        self.send_prog_stop_command()
        #self.send_prog_start_command()

    def send_data_single_word(self,memoryAddress,data): #single word/integer --> 25/30
        self.client.write_register(memoryAddress,data)
    
    def send_data_multiple_words(self,startMemoryAddress,data): #word/integer array [12,15,200]
        self.client.write_registers(startMemoryAddress,data)

    def send_data_bit(self,memoryAddress,data):  # single bit data
        self.client.write_coil(memoryAddress,data)
    
    def send_data_bits(self,startMemoryAddress,data):  # bit array[1,1,1,0,0,1]
        self.client.write_coils(startMemoryAddress,data)

    def read_data_word(self,startMemoryAddress,sizeOfData):
        dataObtained = self.client.read_input_registers(startMemoryAddress,sizeOfData)
        if(dataObtained.isError()):
            return 0
        return dataObtained.registers
    
    def read_data_bits(self,startMemoryAddress,sizeOfData):
        dataObtained = self.client.read_discrete_inputs(startMemoryAddress,sizeOfData)
        if(dataObtained.isError()):
            print("ERROR")
            return 0
        return dataObtained.bits
    
    def send_prog_start_command(self):
        self.send_data_bit(constants.fieldbus_input["START_PROG_ADDR"],1)
        time.sleep(0.02)
        self.send_data_bit(constants.fieldbus_input["START_PROG_ADDR"],0)

    def send_prog_stop_command(self):
        self.send_data_bit(constants.fieldbus_input["STOP_PROG_ADDR"],1)
        time.sleep(0.02)
        self.send_data_bit(constants.fieldbus_input["STOP_PROG_ADDR"],0)

        time.sleep(0.02)

        self.send_data_bit(constants.fieldbus_input["SET_MOTORS_OFF_ADDR"],1)
        time.sleep(0.02)
        self.send_data_bit(constants.fieldbus_input["SET_MOTORS_OFF_ADDR"],0)

    ### Command to move robot to a specific position
    ## Jump flag to make robot Jump 1-> jump 0-> slide
    ## Oreintation to set hand orientation of robot 1-> Righty 0-> Lefty
    ## If robot is already in motion then the previous motion is aborted and new coordinates are given.

    def go_to_position(self,X_POS,Y_POS,Z_POS,ORIENTATION=constants.epson["DEFAULT_HAND_ORIENTATION"],
                                        SPEED_FACTOR=constants.epson["DEFAULT_SPEED_FACTOR"],JUMP_FLAG=constants.epson["DEFAULT_JUMP_FLAG"]):
        
        motionStatus = self.read_data_bits(constants.fieldbus_output["IN_MOTION_STATUS_ADDR"],1)
        
        if(motionStatus==0):
            print("Error Reading Data Register")
            return 0
        if(motionStatus[0]==1): #already in motion
            self.send_data_bit(constants.fieldbus_input["ABORT_COMMAND_ADDR"],1)

            ackFlag=0
            while(ackFlag==0):
                #wait until ack is received
                data = self.read_data_bits(constants.fieldbus_output["ABORT_MOTION_ACK_ADDR"],1)
                ackFlag=data[0]
             
        
        sign_x=0
        sign_y=0
        sign_z=0
        newData=0
        abortMotion=0
        handy = ORIENTATION

        if(X_POS<0):
            sign_x=1
            
        if(Y_POS<0):
            sign_y=1
        
        if(Z_POS<0):
            sign_z=1

        x_pos = abs(X_POS)
        y_pos = abs(Y_POS)
        z_pos = abs(Z_POS)

        self.send_data_bits(constants.fieldbus_input["SIGNX_ADDR"],[sign_x,sign_y,sign_z,newData,abortMotion,handy])

        self.send_data_multiple_words(constants.fieldbus_input["X_POS_ADDR_WORD"],[x_pos,y_pos,z_pos,SPEED_FACTOR])

        if(JUMP_FLAG==1):
            self.send_data_bit(constants.fieldbus_input["JUMP_FLAG_ADDR"],1)
        
        newData=1
        self.send_data_bit(constants.fieldbus_input["NEW_DATA_ADDR"],newData)

        ackFlag=0
        while(ackFlag==0):
            data = self.read_data_bits(constants.fieldbus_output["NEW_DATA_ACK_ADDR"],1)
            ackFlag = data[0]

        self.send_data_bit(constants.fieldbus_input["NEW_DATA_ADDR"],0)
        self.send_data_bit(constants.fieldbus_input["JUMP_FLAG_ADDR"],0)

        return 1
    
    ### reuturns the curr [x,y,z] position of robot in mm
    def get_curr_xy(self):
        
        signArray = self.read_data_bits(constants.fieldbus_output["CURR_SIGNX_ADDR"],3)
        currXY = self.read_data_word(constants.fieldbus_output["CURR_X_POSITION_WORD"],3)


        for i in range(0,3):
            if(signArray[i]==1):
                signArray[i] = -1
            else:
                signArray[i] =1
            currXY[i]=currXY[i]*signArray[i]
        
        return(currXY)
    
    ## Return true if bot is in motion
    def is_bot_inmotion(self):
        inMotionFlag = self.read_data_bits(constants.fieldbus_output["IN_MOTION_STATUS_ADDR"],1)
        return inMotionFlag[0]

    def __del__(self):
        self.send_prog_stop_command()
        time.sleep(3)
        self.client.close()
        print("Closing Client")

if __name__ == "__main__":

    newBot = Epson()
    # newBot.go_to_position(557,170,0,ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=60,JUMP_FLAG=0)

    
    # #time.sleep(0.02)
    # while newBot.is_bot_inmotion():
    #     print("stuck in Motion")
    # newBot.go_to_position(650,250,0,ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=60,JUMP_FLAG=0)

    # while newBot.is_bot_inmotion():
    #     print("stuck in Motion")
    # newBot.go_to_position(700,186,0,ORIENTATION=constants.epson["LEFTY"],SPEED_FACTOR=60,JUMP_FLAG=0)




        