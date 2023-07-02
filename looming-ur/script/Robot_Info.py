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
from File_Edit import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler

class RobotInfo():
    Robot_Name=''
    RefFrame='base_link'
    TargetFrame='tool0'
    Joint_Names=[]

    def point_test(self):
        JointPositions=[[0,0,0,0,0,0],
                        [0.5,0,0,0,0,0],
                        [1,0,0,0,0,0],
                        [1.5,0,0,0,0,0],
                        [2,0,0,0,0,0],
                        [2.5,0,0,0,0,0]]
        Durations=[rospy.Duration(10),rospy.Duration(20),rospy.Duration(30),rospy.Duration(40),rospy.Duration(50),rospy.Duration(60)]

        return JointPositions, Durations

    def mins_sqr_err(self,P1,P2):
        print(P1)
        print(P2)

        dpx=P1.position.x-P2.position.x
        dpy=P1.position.y-P2.position.y
        dpz=P1.position.z-P2.position.z
        drx=P1.orientation.x-P2.orientation.x
        dry=P1.orientation.y-P2.orientation.y
        drz=P1.orientation.z-P2.orientation.z
        drw=P1.orientation.w-P2.orientation.w

        print('Error Summary')
        print('--------------------')
        print('Position')
        print('-----')
        print('P1x'+str(P1.position.x))
        print('P2x'+str(P2.position.x))
        print('Pdx'+str(dpx))
        print('-----')
        print('P1y'+str(P1.position.y))
        print('P2y'+str(P2.position.y))
        print('Pdy'+str(dpy))
        print('-----')
        print('P1z'+str(P1.position.z))
        print('P2z'+str(P2.position.z))
        print('Pdz'+str(dpz))
        print('--------------------')
        print('Orientation')
        print('-----')
        print('P1rx'+str(P1.orientation.x))
        print('P2rx'+str(P2.orientation.x))
        print('Pdrx'+str(drx))
        print('-----')
        print('P1ry'+str(P1.orientation.y))
        print('P2ry'+str(P2.orientation.y))
        print('Pdry'+str(dry))
        print('-----')
        print('P1rz'+str(P1.orientation.z))
        print('P2rz'+str(P2.orientation.z))
        print('Pdrz'+str(drz))
        print('-----')
        print('P1rw'+str(P1.orientation.w))
        print('P2rw'+str(P2.orientation.w))
        print('Pdrw'+str(drw))
        print('--------------------')
        print('MSE')
        error=math.sqrt(dpx**2+dpy**2+dpz**2+drx**2+dry**2+drz**2+drw**2)
        print(error)

        return error

    def update_transf(self):
        self.frames_dict = yaml.safe_load(self.tfBuffer.all_frames_as_yaml())
        self.frames_list = list(self.frames_dict.keys())

        print('frames_list')
        print(str(frames_list))

    def get_frame_transf(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)
        self.frames_dict = yaml.safe_load(self.tfBuffer.all_frames_as_yaml())
        self.frames_list = list(self.frames_dict.keys())

        print('frames_list')
        print(str(self.frames_list))

        transf = self.tfBuffer.lookup_transform(self.RefFrame,self.TargetFrame, rospy.Time(0))
        print(transf)

        return transf

    def get_current_pose(self):
        #try:
        print('Get current Pose')
        transf=self.get_frame_transf()
        P=Pose()
        P.position.x=transf.transform.translation.x
        P.position.y=transf.transform.translation.y
        P.position.z=transf.transform.translation.z
        P.orientation.x=transf.transform.rotation.x
        P.orientation.y=transf.transform.rotation.y
        P.orientation.z=transf.transform.rotation.z
        P.orientation.w=transf.transform.rotation.w
        print(P)
        #except:
            #P=[]
            #print('Error in Robot_Info: get_current_pose')
        return P

    def get_current_joints(self):
        self.current_JS = rospy.wait_for_message("joint_states",JointState)
        return self.current_JS

    def get_IK(self, P, seed_state=[]):
        print('Getting IK')
        ik_solver = IK(self.RefFrame,self.TargetFrame)
        if seed_state==[]:
            seed_state = [0.0] * ik_solver.number_of_joints
        x=P.position.x
        y=P.position.y
        z=P.position.z
        rx=P.orientation.x
        ry=P.orientation.y
        rz=P.orientation.z
        rw=P.orientation.w

        print('Seed State:')
        print(seed_state)

        IKsol=ik_solver.get_ik(seed_state,x,y,z,rx,ry,rz,rw)

        IKsolution=list(IKsol)
        print('IK solution')
        print(IKsolution)
        IK_sol_JN=ik_solver.joint_names
        print('IK solution Joint Names')
        print(IK_sol_JN)

        JN=self.get_joint_names()
        IK_JS=[]
        for i in range(len(JN)):
            Name=JN[i]
            index=IK_sol_JN.index(Name)
            IK_JS.append(IKsolution[index])

        return IK_JS

    def get_joint_names(self):
        JS=self.get_current_joints()
        self.Joint_Names=JS.name
        #print(self.Joint_Names)
        return self.Joint_Names

    def input_pose(self, Orientation='RPY'):
        P=Pose()
        P.position.x=float(input('Write Position X '))
        P.position.y=float(input('Write Position Y '))
        P.position.z=float(input('Write Position Z '))
        Q=[0,0,0,1]
        if Orientation=='RPY':
            Roll=float(input('Write Roll about an X-axis '))
            Pitch=float(input('Write Pitch about an Y-axis '))
            Yaw=float(input('Write Yaw about an Z-axis '))
            Q=quaternion_from_euler(Roll, Pitch, Yaw)
        if Orientation=='Q':
            Q[0]=float(input('Write Orientation X '))
            Q[1]=float(input('Write Orientation Y '))
            Q[2]=float(input('Write Orientation Z '))
            Q[3]=float(input('Write Orientation W '))
        P.orientation.x=Q[0]
        P.orientation.y=Q[1]
        P.orientation.z=Q[2]
        P.orientation.w=Q[3]
        return P

    def input_js(self):
        self.get_joint_names()
        inJS=[]
        for JointName in self.Joint_Names:
            inJS.append(float(input('Write '+JointName+' Value: ')))
        JS=JointState()
        JS.position=inJS
        print(inJS)
        return inJS

    def read_js(self, JSlist, JointNamesOrder):#=self.Joint_Names
        self.get_joint_names()
        JS=JointState()
        JS.name=self.Joint_Names
        for Jname in JS.name:
            Index=JointNamesOrder.index(Jname)
            JS.position.append(JSlist[Index])
        return JS

    def read_pose(self, POSElist, Orientation='RPY'):
        P=Pose()
        P.position.x=POSElist[0]
        P.position.y=POSElist[1]
        P.position.z=POSElist[2]
        Q=[0,0,0,1]
        if Orientation=='RPY':
            Roll=POSElist[3]
            Pitch=POSElist[4]
            Yaw=POSElist[5]
            Q=quaternion_from_euler(Roll, Pitch, Yaw)
        if Orientation=='Q':
            Q[0]=POSElist[3]
            Q[1]=POSElist[4]
            Q[2]=POSElist[5]
            Q[3]=POSElist[6]
        P.orientation.x=Q[0]
        P.orientation.y=Q[1]
        P.orientation.z=Q[2]
        P.orientation.w=Q[3]
        return P


    def robot_info_menu(self, I=''):

            E=0
            while E==0:
                if I=='':
                    print('-------------------')
                    print('Robot Info:')
                    print('1->Show Joint State')
                    print('2->Show TCP Pose')

                    print('3->JS From TCP Pose')
                    print('4->TCP Pose From JS')

                    print('0->Exit')
                    I=input()

                if I=='1':
                    JS=self.get_current_joints()
                    print(JS)

                elif I=='2':
                    P=self.get_current_pose()
                    print(P)

                elif I=='3':
                    P=self.input_pose()
                    IKsol=self.get_IK(P)
                    print(IKsol)

                elif I=='4':
                    print('TO DO')

                elif I=='0': #Exit
                    E=1
                else:
                    print('Input Error')

                I=''
