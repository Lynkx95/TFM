#!/usr/bin/python
#https://automaticaddison.com/how-to-move-a-simulated-robot-arm-to-a-goal-using-ros/
#source catkin-ws/devel/setup.bash
#source catkin_ws/devel/setup.bash
#source TFM/catkin_ws/devel/setup.bash

#python3 TFM/looming-ur/script/gui_dmp.py

import os
import pickle
import rospy
import roslaunch
import actionlib
import tf2_ros
import yaml
import math
import subprocess
import time
from File_Edit import *
from Move_Robot import *
from Robot_Info import *
from Work_Cycle import *

def main():
    Robot=RobotInfo()
    Robot.Robot_Name='ur3'

    DataEditor=DataManagement()
    DataEditor.set_robot_info(Robot)
    PointEditor=DataManagement()
    PointEditor.set_robot_info(Robot)
    I=''

    PointType='JS'

    JointNamesOrder=['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    RotType='RPY'


    while I!='0':
        time.sleep(1)
        DataFileName='cycledata'

        DataEditor.load_file(DataFileName)

        I=''
        I=input('User Input: ')

        if I=='1':
            DataEditor.DataDict['LoadMachine']=True
            DataEditor.save_file(DataFileName)
        elif I=='2':
            DataEditor.DataDict['UnloadMachine']=True
            DataEditor.save_file(DataFileName)
        elif I=='0':
            print('exiting')
            DataEditor.DataDict['Repeat']=False
            DataEditor.save_file(DataFileName)

        if DataEditor.DataDict['AskPickupPoint']:
            DataEditor.DataDict['AskPickupPoint']=False
            #Pointditor.DataDict['pinput']=Point
            PointStr='0, 0, 0, 0, 0, 0'
            TextPoint=list(map(float,PointStr.split(',')))
            if PointType=='JS':
                Pointditor.DataDict['pinput']=self.Robot.read_js(TextPoint, JointNamesOrder)
            elif PointType=='Pose':
                Pointditor.DataDict['pinput']=self.Robot.read_pose(TextPoint, RotType)

            DataEditor.DataDict['PickupPointSet']=True
            DataEditor.save_file(DataFileName)

        if DataEditor.DataDict['ActivateGripper']:
            DataEditor.DataDict['ActivateGripper']=False
            DataEditor.DataDict['GripperActive']=True
            DataEditor.DataDict['GripperInactive']=False
            DataEditor.save_file(DataFileName)

        if DataEditor.DataDict['DeactivateGripper']:
            DataEditor.DataDict['DeactivateGripper']=False
            DataEditor.DataDict['GripperActive']=False
            DataEditor.DataDict['GripperInactive']=True
            DataEditor.save_file(DataFileName)




if __name__ == "__main__":
    main()
