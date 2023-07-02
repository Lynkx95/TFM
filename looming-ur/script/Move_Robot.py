#!/usr/bin/python
#https://automaticaddison.com/how-to-move-a-simulated-robot-arm-to-a-goal-using-ros/
import os
import pickle
import rospy
import roslaunch
import actionlib
import tf2_ros
import yaml
import math
import time
from File_Edit import *
from Robot_Info import *
from Point_Edit import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler

class RobotMovement():
    rqtJTC=0
    process_rqtJTC=''
    speed_scale=1.0
    auto_duration=True

    def set_robot_and_editor(self, Robot, Editor):

        self.Robot=Robot
        self.Editor=Editor

    def point_set(self):
        JointPositions=[[0,0,0,0,0,0],
                        [0.5,0,0,0,0,0],
                        [1,0,0,0,0,0],
                        [1.5,0,0,0,0,0],
                        [2,0,0,0,0,0],
                        [2.5,0,0,0,0,0]]
        Durations=[rospy.Duration(10),rospy.Duration(20),rospy.Duration(30),rospy.Duration(40),rospy.Duration(50),rospy.Duration(60)]

        return JointPositions, Durations

    def set_traj_point(self, JointPosition, Duration):
        point = JointTrajectoryPoint()
        point.positions = JointPosition
        #print('goal point set to: \n' + str(point.positions))
        point.time_from_start = rospy.Duration(Duration)
        #print('Move duration set to ' + str(point.time_from_start.to_nsec()/1000000000.0) + 's')
        return point

    def set_traj_goal(self):
        self.robot_goal = FollowJointTrajectoryGoal()
        self.Robot.get_joint_names()
        self.robot_goal.trajectory.joint_names = self.Robot.Joint_Names
        #print('Robot trajectory joint names set')
        for i in range(len(self.JS)):
            point=self.set_traj_point(self.JS[i], self.DUR[i])
            self.robot_goal.trajectory.points.append(point)

    def move_robot(self):
        robot_client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        robot_client.wait_for_server()
        print('Connected to action server')
        self.set_traj_goal()
        print('Goal Set')
        #print(self.robot_goal)
        exec_timeout = rospy.Duration(10)
        prmpt_timeout = rospy.Duration(5)
        time_init=rospy.get_rostime()
        robot_client.send_goal_and_wait(self.robot_goal, exec_timeout, prmpt_timeout)
        #time_move=rospy.get_rostime()
        robot_client.wait_for_result()
        time_end=rospy.get_rostime()
        print('Goal reached')
        #print(str(time_move-time_init))

        print(str((time_end.to_nsec()-time_init.to_nsec())/1000000000.0))
        print(str(self.DUR))

    def go_to_home(self):
        self.JS=[]
        self.DUR=[]
        self.JS.append([0,0,0,0,0,0])
        self.set_move_duration()
        self.move_robot()



    def set_auto_duration(self):
        if self.auto_duration:
            self.auto_duration=False
            print('Duration Set to Manual Input')
        else:
            self.auto_duration=True
            print('Duration Set to Automatic')


    def set_speed_scale(self):
        print('Write new speed scale (default 1):')
        self.speed_sacle=float(input())

    def set_point_duration(self, P1, P2):
        Pdistance=math.dist(P1,P2)
        Duration=Pdistance*self.speed_scale
        return Duration

    def set_move_duration(self):
        self.DUR=[]
        self.Robot.get_current_joints()
        PrevJS=self.Robot.current_JS.position
        for NextJS in self.JS:
            self.DUR.append(self.set_point_duration(PrevJS,NextJS))
            PrevJS=NextJS

    def start_stop_rqt_jtc(self):
        if self.rqtJTC==1:
            self.rqtJTC=0
            self.process_rqtJTC.stop()
            self.process_rqtJTC=''

        elif self.rqtJTC==0:
            self.rqtJTC=1
            package_rqtJTC = 'rqt_joint_trajectory_controller'
            executable_rqtJTC = 'rqt_joint_trajectory_controller'
            node_rqtJTC = roslaunch.core.Node(package_rqtJTC, executable_rqtJTC)
            launch_rqtJTC = roslaunch.scriptapi.ROSLaunch()
            launch_rqtJTC.start()
            self.process_rqtJTC = launch_rqtJTC.launch(node_rqtJTC)

    def set_joint_traj(self, PointList=[]):
        self.JS=[]
        self.DUR=[]
        N=len(self.PointTypes)
        for i in range(N):

            if self.PointTypes[i]=='':
                self.PointTypes[i]=input('Choose input JointState, input Pose or Read from File [JS/P/F]')

            if self.PointTypes[i]=='L':

                if type(PointList[i]) is JointState:
                    InJointState=PointList[i].position

                elif type(PointList[i]) is Pose:
                    CurrentJS=self.Robot.get_current_joints()
                    InJointState=self.Robot.get_IK(PointList[i],CurrentJS.position)

            if self.PointTypes[i]=='F':
                self.Editor.load_file()
                print('Choose point to load:')
                print(self.Editor.DataDict.keys())
                PointName=input()
                FilePoint=self.Editor.DataDict[PointName]
                print(FilePoint)
                print(type(FilePoint))

                if type(FilePoint) is JointState:
                    InJointState=FilePoint.position

                elif type(FilePoint) is Pose:
                    CurrentJS=self.Robot.get_current_joints()
                    InJointState=self.Robot.get_IK(FilePoint,CurrentJS.position)

            if self.PointTypes[i]=='JS':
                InJointState=self.Robot.input_js()
            elif self.PointTypes[i]=='P':
                InPose=self.Robot.input_pose()
                CurrentJS=self.Robot.get_current_joints()
                InJointState=self.Robot.get_IK(InPose,CurrentJS.position)

            self.JS.append(InJointState)
            if not(self.auto_duration):
                self.DUR.append(float(input('Write Move Duration (seconds): ')))

        if self.auto_duration:
            self.set_move_duration()
        print(self.JS)

    def move_robot_menu(self, I=''):
            FilesDirectory='../data/'
            WorkingFile=''
            WorkingPoint=''
            DataDict={}
            FileSaved=True
            E=0

            while E==0:
                if I=='':
                    print('1->RQT Joint Trajectory Controller ' + int(not(self.rqtJTC))*'Start' + self.rqtJTC*'Stop')

                    print('2->TODO Move Joints')
                    print('3->TODO Move TCP Pose')

                    print('4->Move Robot To Input JS')
                    print('5->Move Robot To Input POSE')
                    print('6->Move Robot To File Point')

                    print('7->Move Robot To Input JS Trajectory')
                    print('8->Move Robot To Input POSE Trajectory')
                    print('9->Move Robot To File Trajectory')

                    print('10->Move Robot To Input or File Trajectory')

                    print('11->Change Duration Input method to ' + int(not(self.auto_duration))*'Automatic' + self.auto_duration*'Manual')
                    print('12->Change Automatic Duration Scale')

                    print('0->Exit')
                    I=input()

                if I=='1':
                    self.start_stop_rqt_jtc()

                elif I=='2':
                    print('TODO')

                elif I=='3':
                    print('TODO')

                elif I=='4':
                    print('Move Robot to Input JS')
                    self.PointTypes=['JS']
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='5':
                    print('Move Robot to Input POSE')
                    self.PointTypes=['P']
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='6':
                    print('Move Robot to File Point')
                    self.PointTypes=['F']
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='7':
                    print('Move Robot to Input JS Trajectory')
                    N=int(input('Input Number of Points in The Trajectory: '))
                    self.PointTypes=[]
                    for i in range(N):
                        self.PointTypes.append('JS')
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='8':
                    print('Move Robot to Input POSE Trajectory')
                    N=int(input('Input Number of Points in The Trajectory: '))
                    self.PointTypes=[]
                    for i in range(N):
                        self.PointTypes.append('P')
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='9':
                    print('Move Robot to File Trajectory')
                    N=int(input('Input Number of Points in The Trajectory: '))
                    self.PointTypes=[]
                    for i in range(N):
                        self.PointTypes.append('F')
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='10':
                    print('Move Robot To Trajactory')
                    N=int(input('Input Number of Points in The Trajectory: '))
                    self.PointTypes=[]
                    for i in range(N):
                        self.PointTypes.append('')
                    self.set_joint_traj()
                    self.move_robot()

                elif I=='11':
                    print('Changing duration input method')
                    self.set_auto_duration()

                elif I=='12':
                    print('Changing duration scale')
                    self.set_speed_scale()

                elif I=='0':
                    E=1
                    if self.rqtJTC==1:
                        self.start_stop_rqt_jtc()
                else:
                    print('Input Error')

                I=''
