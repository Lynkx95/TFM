#!/usr/bin/python
#https://automaticaddison.com/how-to-move-a-simulated-robot-arm-to-a-goal-using-ros/
import os
import pickle
import rospy
import roslaunch
import actionlib
import time
from Robot_Info import *
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class WorkCycle():

    PythonFilePath=os.path.dirname(__file__)
    RelFilesDirPath='../data/'
    FilesDirectory=os.path.abspath(os.path.join(PythonFilePath, RelFilesDirPath))
    DataFileName=''
    PointsFileName=''
    ScriptFileName=''

    def set_robot_editor_move(self, Robot, Editor, Movement):
        self.Robot=Robot
        self.Editor=Editor
        self.Editor.set_robot_info(self.Robot)
        self.Movement=Movement
        self.Movement.set_robot_and_editor(self.Robot, self.Editor)


    def import_cycle_data(self):
        if self.DataFileName=='':
            self.DataFileName=input('Input Cycle Data File Name: ')

        self.Editor.import_data(self.DataFileName)
        self.Editor.save_file(self.DataFileName,'S','Y')
        self.CycleData=self.Editor.DataDict

        self.PointsFileName=self.CycleData['PointFile']
        self.ScriptFileName=self.CycleData['ScriptFile']

    def load_cycle_data(self):
        if self.DataFileName=='':
            self.DataFileName=input('Input Cycle Data File Name: ')

        self.Editor.load_file(self.DataFileName)
        self.CycleData=self.Editor.DataDict

        print(self.CycleData)

        self.PointsFileName=self.CycleData['PointFile']
        self.ScriptFileName=self.CycleData['ScriptFile']


    def import_cycle_points(self):
        self.Editor.import_data(self.PointsFileName)
        self.CyclePoints=self.Editor.DataDict


    def load_cycle_points(self):
        self.Editor.load_file(self.PointsFileName)
        self.CyclePoints=self.Editor.DataDict


    def load_script(self):
        with open(self.FilesDirectory +'/'+ self.ScriptFileName + '.txt') as f:
            self.script = f.readlines()
        print(self.script)


    def execute_script(self):
        IfExec=[True]
        WhileExec=[True]
        WhileLine=[]
        LineNum=0
        while LineNum<len(self.script):
            line = self.script[LineNum]
            words=line.split()
            print('line: ' + line.replace('\n',''))
            print(IfExec)
            print(WhileExec)

            if words[0] == 'load' and WhileExec[-1] and IfExec[-1]:
                print('Loading file: ' + words[1])
                self.Editor.reset_data()
                self.Editor.load_file(words[1])
                self.CycleData=self.Editor.DataDict

            if words[0] == 'import' and WhileExec[-1] and IfExec[-1]:
                print('Importing file: ' + words[1])
                self.Editor.reset_data()
                self.Editor.import_data(words[1])
                self.CycleData=self.Editor.DataDict

            if words[0] == 'moveto' and WhileExec[-1] and IfExec[-1]:
                print('Moving to: ' + words[1])
                self.Movement.PointTypes=['L']
                self.Movement.set_joint_traj([self.CyclePoints[words[1]]])
                self.Movement.move_robot()

            if words[0] == 'wait' and WhileExec[-1] and IfExec[-1]:
                if words[1] == 'time':
                    print('Waiting '+words[2]+'s')
                    print(float(words[2]))
                    time.sleep(float(words[2]))
                if words[1] == 'user':
                    print('Waiting for user input')
                    input()
                if words[1] == 'var':
                    while not(self.CycleData[words[2]]):
                        print('Waiting for:' + words[2])
                        print(self.CycleData[words[2]])
                        print(not(self.CycleData[words[2]]))
                        time.sleep(1)
                        self.load_cycle_data()

            if words[0] == 'set' and WhileExec[-1] and IfExec[-1]:
                if words[1] == 'user':
                    words.append(input('Set data:'))
                if type(self.CycleData[words[1]]) is str:
                    self.CycleData[words[1]]=words[2]
                if type(self.CycleData[words[1]]) is float:
                    self.CycleData[words[1]]=float(words[2])
                if type(self.CycleData[words[1]]) is bool:
                    self.CycleData[words[1]]=bool('T' in words[2])
                self.Editor.DataDict=self.CycleData
                self.Editor.save_file(self.DataFileName,'S','Y')

            if words[0] == 'while' and WhileExec[-1] and IfExec[-1]:
                if self.CycleData[words[1]]:
                    WhileExec.append(True)
                    WhileLine.append(LineNum)
                else:
                    WhileExec.append(False)

            if words[0] == 'endwhile' and IfExec[-1]:
                if WhileExec[-1]:
                    LineNum=WhileLine[-1] - 1
                WhileExec.pop()

            if words[0] == 'if' and WhileExec[-1]:
                if self.CycleData[words[1]] and IfExec[-1]:
                    IfExec.append(True)
                else:
                    IfExec.append(False)

            if words[0] == 'else' and WhileExec[-1]:
                if IfExec[-1]:
                    IfExec[-1]=False
                else:
                    IfExec[-1]=True

            if words[0] == 'endif' and WhileExec[-1]:
                IfExec.pop()

            LineNum=LineNum+1


    def work_cycle_menu(self, I=''):
        self.DataFileName='cycledata'

        E=0
        while E==0:
            if I=='':
                print('-------------------')
                print('Work Cycle:')
                print('1->Load Work Cycle Data')
                print('2->Import Work Cycle Data')
                print('3->Load Cycle Points')
                print('4->Import Cycle Points')
                print('5->Load Script')
                print('6->Load All')

                print('7->Execute Script')

                print('0->Exit')
                I=input()

            if I=='1':
                self.load_cycle_data()

            elif I=='2':
                self.import_cycle_data()

            elif I=='3':
                self.load_cycle_points()

            elif I=='4':
                self.import_cycle_points()

            elif I=='5':
                self.load_script()

            elif I=='6':
                self.import_cycle_data()
                self.load_cycle_data()
                self.load_cycle_points()
                self.load_script()

            elif I=='7':
                self.execute_script()

            elif I=='0': #Exit
                E=1
            else:
                print('Input Error')

            I=''
