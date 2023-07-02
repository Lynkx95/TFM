#!/usr/bin/python
#https://automaticaddison.com/how-to-move-a-simulated-robot-arm-to-a-goal-using-ros/
import os
import pickle
import rospy
import roslaunch
import actionlib
from Robot_Info import *
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def list_points(PointDict, Type=''):

    if Type=='':
        print('Select What information to show')
        print('1->Point Names')
        print('2->Point Names and Joint States')
        print('3->Point Names, Joint States, Velocities and Effort')
        print('4->Point Names and Header')
        print('5->Joint Names')
        print('6->All Info')
        print('0->Exit')
        Type=input()

    if Type=='1':
        print('Point Names:' )
        print(str(list(PointDict.keys())))
    elif Type=='2':
        print('Point Names and Joint States')
        for key in PointDict:
            print(key)
            print(PointDict[key].position)
    elif Type=='3':
        print('Point Names, Joint States, Velocities and Effort')
        for key in PointDict:
            print(key)
            print('Joint States')
            print(PointDict[key].position)
            print('Velocities')
            print(PointDict[key].velocity)
            print('Efforts')
            print(PointDict[key].effort)
    elif Type=='4':
        print('Point Names and Header')
        for key in PointDict:
            print(key)
            print(PointDict[key].header)
    elif Type=='5':
        print('Joint Names')
        key=PointDict.keys()
        print('1')
        print(PointDict[list(key)[0]].name)
    elif Type=='6':
        print('All Information')
        for key in PointDict:
            print(key)
            print(PointDict[key])

def select_point(PointDict, WorkingPoint=''):
    if WorkingPoint=='':
        list_points(PointDict, '1')
        print('Select Point')
        P=input()
    if P in PointDict.keys():
        WorkingPoint=P
    else:
        print('Point Not Found')

    return WorkingPoint


def edit_point_name(PointDict, WorkingPoint, FileSaved=False, PointName='', YN=''):
    NameRepeated=1
    while NameRepeated==1:
        NameRepeated=0
        if PointName=='':
            list_points(PointDict, '1')
            print('Write a New Name for the Point:')
            PointName=input()
        if PointName in PointDict.keys():
            NameRepeated=1
        if NameRepeated==0:
            print('Changing point name')
            new_dict={}
            for key in PointDict:
                if key==WorkingPoint:
                    new_dict[PointName]=PointDict[WorkingPoint]
                else:
                    new_dict[key]=PointDict[key]
            PointDict=new_dict
            WorkingPoint=PointName
            FileSaved=False
        else:
            if PointName==WorkingPoint:
                print('Name is the same')
                if YN=='':
                    print('Choose another? [Y/n]')
                    YN=input()
                if YN!='Y':
                    NameRepeated=0
            else:
                print('Name already in use')
                if YN=='':
                    print('Choose another? [Y/n]')
                    YN=input()
                if YN!='Y':
                    NameRepeated=0

    return PointDict, FileSaved, WorkingPoint


def point_editor(PointDict, FileSaved=False):
    print('Starting Point Editor')

    WorkingPoint=''
    E=0

    while E==0:
        if WorkingPoint=='':
            print('No point selected to edit')
        else:
            print('Working on Point:' + WorkingPoint)
        print('-----------')
        print('Options:')
        print('1->Select Point')
        print('2->Change Name')
        print('3->Edit Joint Value')
        print('0->Exit')
        I=input()

        if I=='1':
            print('Selecting Point')
            WorkingPoint=select_point(PointDict)

        elif I=='2':
            print('Changing Name')
            [PointDict, FileSaved, WorkingPoint]=edit_point_name(PointDict, FileSaved, WorkingPoint)

        elif I=='3':
            while E==0:
                n=1
                print('Select Joint to edit')
                for joint in PointDict[WorkingPoint].name:
                    print(str(n)+'->'+joint)
                    n+=1
                print('0->Exit')

                I=input()
                if I=='0':
                    E=1
                else:
                    try:
                        print(PointDict[WorkingPoint].name[int(I)-1] + '->' + str(PointDict[WorkingPoint].position[int(I)-1]))
                        print('Enter New Value')
                        NewValue=input()
                        print(str(float(NewValue)))
                        try:
                            print('Start Try')
                            position_list=list(PointDict[WorkingPoint].position)
                            print('tuple to list')
                            print(str(position_list))

                            position_list[int(I)-1]=float(NewValue)
                            print('added new value')
                            print(str(position_list))

                            PointDict[WorkingPoint].position=tuple(position_list)
                            print('updated point')
                            print(str(PointDict[WorkingPoint]))
                            print('updated dict')
                            print(str(PointDict))

                            FileSaved=False
                        except:
                            print('Invalid Input Value')
                    except:
                        print('Input Error')
            E=0

        elif I=='0':
            E=1
        else:
            print('Invalid Input')
    return PointDict, FileSaved
