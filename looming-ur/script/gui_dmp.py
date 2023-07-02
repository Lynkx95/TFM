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
from File_Edit import *
from Move_Robot import *
from Robot_Info import *
from Work_Cycle import *
from controller_manager_msgs.srv import *
from controller_manager_msgs.msg import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK

def controller_manager_menu(I=''):
    E=0
    while E==0:
        if I=='':
            print('1->List Controllers')
            print('2->Load Controllers')
            print('3->Unload Controllers')
            print('4->Start/Stop Controllers')
            print('5->List Controller Types')
            print('6->Reload Controller Libraries')

            print('0->Exit Manager')
            I=input()
        if I=='1':
            rospy.wait_for_service('/controller_manager/list_controllers')
            list_controllers=rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
            try:
                Controller_List=list_controllers()
                print(str(Controller_List))
            except:
                print('Error Listing Controllers')
        elif I=='2':
            controller_to_load='joint_group_eff_controller'
            rospy.wait_for_service('/controller_manager/load_controller')
            load_controllers=rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
            try:
                Controller_Load=load_controllers(controller_to_load)
                if Controller_Load:
                    print('Controller Loaded')
            except:
                print('Error Loading Controllers')
        elif I=='3':
            controller_to_unload='joint_group_eff_controller'
            rospy.wait_for_service('/controller_manager/unload_controller')
            unload_controllers=rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
            try:
                Controller_Unload=unload_controllers(controller_to_unload)
                if Controller_Unload:
                    print('Controller Unloaded')
            except:
                print('Error Unloading Controllers')

        elif I=='0': #Exit
            E=1
        else:
            print('Input Error')
        I=''

def launch_file_select(robot_name='ur3'):
    if 'ur' in robot_name:
        launch_path=['/opt/ros/noetic/share/ur_gazebo/launch/'+robot_name+'_bringup.launch']
    elif 'fanuc' in robot_name:
        launch_path=['/home/alvaro/TFM/fanuc_gazebo/launch/'+robot_name+'_bringup.launch']
    else:
        print('Error Selecting File. Unknown Robot. Selecting default UR3.')
        launch_path=['/opt/ros/noetic/share/ur_gazebo/launch/ur3_bringup.launch']
    return launch_path


def set_roslaunch_file(robot_name='ur3'):

    launch_path=launch_file_select(robot_name)

    file_arg='controller_config_file:=/home/alvaro/TFM/looming-ur/script/ur3_controllers.yaml'
    controller_arg='controllers:="joint_state_controller" "eff_joint_traj_controller"'
    stopped_cont_arg= 'stopped_controllers:= "joint_group_eff_controller"'
    launch_args=[file_arg,controller_arg,stopped_cont_arg]

    cli_args = launch_path #+ launch_args
    roslaunch_args = cli_args[1:]

    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    return roslaunch_file

def simulation(robot_name, I=''):

    rospy.init_node("Looming_UR", anonymous=False)

    roslaunch_file =set_roslaunch_file(robot_name)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_SIM = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch_SIM.start()

    rospy.sleep(5)

    Robot=RobotInfo()
    Robot.Robot_Name=robot_name

    FileEditor=DataManagement()
    FileEditor.set_robot_info(Robot)

    MoveRobot=RobotMovement()
    MoveRobot.set_robot_and_editor(Robot, FileEditor)
    MoveRobot.go_to_home()

    WorkLoop=WorkCycle()
    WorkLoop.set_robot_editor_move(Robot, FileEditor, MoveRobot)

    E=0
    while E==0:
        if I=='':
            print('1->Robot Info')
            print('2->Move Robot')
            print('3->File Editor')
            print('4->Work Cycle')
            print('5->Controller Manager')
            print('6->Test')
            print('TODO Add/Remove Frames')
            print('0->Exit simulation')
            I=input()

        if I=='1':
            Robot.robot_info_menu()

        elif I=='2':
            MoveRobot.move_robot_menu()

        elif I=='3':
            FileEditor.file_editor()

        elif I=='4':
            WorkLoop.work_cycle_menu()

        elif I=='5':
            controller_manager_menu()

        elif I=='6':
            subprocess.run('rosnode kill /ros_control_stopped_spawner', shell=True)

        elif I=='0': #Exit
            E=1
        else:
            print('Input Error')

        I=''

    launch_SIM.shutdown()
    rospy.signal_shutdown('Rospy Shutdown: End of Simulation')

def main():
    N='1'
    E=1
    robot='ur3'
    while E==1:
        print('Robot: '+robot)
        if N=='':
            print("1->Simulation")
            print("2->Real")
            print("3->Select Robot")
            print("4->Edit Robot Configuration")
            print("0->Exit")
            N=input()
        if N=="1":
            print("Starting Simulation")
            simulation(robot)
            print('Simulation ended')
        elif N=="2":
            print("Real")
        elif N=='3':
            print('Select Robot')
            print('1->UR3')
            print('2->UR3e')
            print('3->UR5')
            print('4->UR5e')
            print('5->UR10')
            print('6->UR10e')
            print('7->UR16e')
            print('8->Fanuc cr7ia')
            print('9->Fanuc cr35ia')
            print('10->Fanuc lrmate200i')
            print('11->Fanuc m10ia')
            print('0->Exit')
            I=input()
            if I=='1': robot='ur3'
            elif I=='2': robot='ur3e'
            elif I=='3': robot='ur5'
            elif I=='4': robot='ur5e'
            elif I=='5': robot='ur10'
            elif I=='6': robot='ur10e'
            elif I=='7': robot='ur16e'
            elif I=='8': robot='fanuc_cr7ia'
            elif I=='9': robot='fanuc_cr35ia'
            elif I=='10': robot='fanuc_lrmate200i'
            elif I=='11': robot='fanuc_m10ia'
        elif N=='4':
            print('Edit Robot Config')

        elif N=="0":
            print("Exiting")
            E=0
        else:
            print("Input Error")
        N='0'

if __name__ == "__main__":
    main()
