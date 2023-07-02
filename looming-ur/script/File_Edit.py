#!/usr/bin/python
#https://automaticaddison.com/how-to-move-a-simulated-robot-arm-to-a-goal-using-ros/
import os
import pickle
import rospy
import roslaunch
import actionlib
from Point_Edit import *
from Robot_Info import *
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class DataManagement():
    PythonFilePath=os.path.dirname(__file__)
    RelFilesDirPath='../data/'
    FilesDirectory=os.path.abspath(os.path.join(PythonFilePath, RelFilesDirPath))
    FilesList=[]
    WorkingFile=''
    DataDict={}
    FileSaved=False

    def __init__(self):
        self.FilesList=os.listdir(self.FilesDirectory)

    def set_robot_info(self, Robot):
        self.Robot=Robot

    def reset_data(self):
        self.DataDict={}

    def save_data(self, FileName):
        try:
            with open(self.FilesDirectory+'/'+FileName+'.pickle', 'wb') as handle:
                pickle.dump(self.DataDict, handle, protocol=pickle.HIGHEST_PROTOCOL)
                self.FileSaved=True
        except:
            print('Error Saving data')

    def load_data(self):
        try:
            with open(self.FilesDirectory +'/'+ self.WorkingFile + '.pickle', 'rb') as handle:
                self.DataDict = pickle.load(handle)
                #print('Data Loaded')
                #print(self.DataDict)
                self.FileSaved=True
            print('Loaded Data From '+ self.FilesDirectory +'/'+ self.WorkingFile +'.pickle')
        except:
            print('Error Loading Data')
            self.DataDict={}

    def list_files(self, PrintList=True):
        print('Files Directory: ' + self.FilesDirectory)
        self.FilesList=os.listdir(self.FilesDirectory)
        if PrintList:
            for filename in self.FilesList:
                print(filename)

    def load_file(self, FileNameInput='', YN=''):

        if FileNameInput=='':
            self.list_files()
            print('Select File to Load')
            FileNameInput=input()

        FileNameInput=FileNameInput.replace('.pickle','')

        if (FileNameInput + '.pickle') in self.FilesList:
            self.WorkingFile=FileNameInput
            self.load_data()

        else:
            if YN=='':
                print('File not found')
                print('Create File? [Y/n]')
                YN=input()
            if YN=='Y':
                self.WorkingFile=FileNameInput
                self.save_data(self.WorkingFile)
            else:
                print('Aborting')

    def save_file(self, NewFileName='', SA='S',YN=''):
        if self.WorkingFile=='':
            SA='A'

        if SA=='A':
            if NewFileName=='':
                print('Write a name for the Working File')
                NewFileName=input()

            if NewFileName + '.pickle' in self.FilesList:
                if YN=='':
                    print('File already exists, overwrite? [Y/n]')
                    YN=input()
                if YN=='Y':
                    self.save_data(NewFileName)
                else:
                    print('Aborting')
            else:
                self.save_data(NewFileName)
                self.WorkingFile=NewFileName
        else:
            self.save_data(self.WorkingFile)

    def delete_file(self, DelFileName='', YN=''):

        if DelFileName=='':
            print('Write the Name of the Data File to Delete')
            DelFileName=input()
        if DelFileName + '.pickle' in self.FilesList:
            if YN=='':
                print(DelFileName + '.pickle will be deleted, Are you sure? [Y/n]')
                YN=input()
            if YN=='Y':
                print('Deleting: ' + DelFileName)
                os.remove(self.FilesDirectory+'/'+DelFileName+'.pickle')
                self.list_files()
                if DelFileName==self.WorkingFile:
                    self.WorkingFile=''
                    self.DataDict={}
                    self.FileSaved=False
            else:
                print('Aborting Deletion')
        else:
            print('File Not Found, Aborting Deletion')

    def store_data(self, Data, DataName='', OC=''):
        E=0
        while E==0:

            if DataName=='':
                print('Names in use:' + str(list(self.DataDict.keys())))
                print('Write a Name for the point:')
                DataName=input()

            if DataName in list(self.DataDict.keys()):
                YN=''
                if OC=='':
                    print('Name already in use, overwrite? [Y/n]')
                    YN=input()
                    if YN=='Y':
                        OC='O'
                    else:
                        print('Choose another name? [Y/n]')
                        YN=input()
                        if YN=='Y':
                            OC='C'
                        else:
                            print('Aborting')
                            E=1
                if OC=='O':
                    self.DataDict[DataName]=Data
                    self.FileSaved=False
                    E=1

                if OC=='C':
                    OC=''
                    DataName=''
                    E=0
            else:
                self.DataDict[DataName]=Data
                self.FileSaved=False
                E=1

    def import_data(self, FileNameInput=''):
        self.reset_data()
        if FileNameInput=='':
            self.list_files()
            print('Select File to Load')
            FileNameInput=input()

        FileNameInput=FileNameInput.replace('.txt','')

        with open(self.FilesDirectory +'/'+ FileNameInput+'.txt') as f:
            contents = f.readlines()
        print(contents)

        Type='JS'
        for line in contents:
            if line.startswith('JS'):
                Type='JS'
                FileJointNames=line[3:].split()
                print(FileJointNames)

            elif line.startswith('POSE'):
                Type='POSE'
                RotType=line[5:].strip()
                print(RotType)

            elif line.startswith('Name'):
                PointName=line[5:].strip()
                print(PointName)

            elif line.startswith('Point'):
                TextPoint=list(map(float,line[6:].split(',')))
                print(TextPoint)
                if Type=='JS':
                    Point=self.Robot.read_js(TextPoint, FileJointNames)
                elif Type=='POSE':
                    Point=self.Robot.read_pose(TextPoint, RotType)
                self.store_data(Point, PointName, 'O')

            elif line.startswith('Bool'):
                txtBool=line[6:]
                print(txtBool)
                if 'T' in txtBool.upper():
                    Bool=True
                if 'F' in txtBool.upper():
                    Bool=False
                self.store_data(Bool, PointName, 'O')

            elif line.startswith('Num'):
                txtNum=line[5:]
                self.store_data(float(txtNum), PointName, 'O')

            elif line.startswith('Text'):
                txtStr=line[6:].strip()
                self.store_data(txtStr, PointName, 'O')


    def delete_point(self, DelDataName='', YN=''):
        print('Point Names:' + str(list(self.DataDict.keys())))
        if DelDataName=='':
            print('Write the Name of the Point to Delete')
            DelDataName=input()
        if DelDataName in list(self.DataDict.keys()):
            if YN=='':
                YN=input('Are you sure you want to delete ' + DelDataName + '? [Y/n]')
            if YN=='Y':
                print('Deleting: ' + DelDataName)
                del self.DataDict[DelDataName]
                self.FileSaved=False
            else:
                print('Aborting')
        else:
            print('Point Name Not Found, Aborting')

    def file_editor(self):
        print('Starting File Editor')

        E=0

        while E==0:

            print('-----------')
            if self.FilesList==[]:
                print('No files found in' + self.FilesDirectory)
            else:
                self.list_files()
            if self.WorkingFile!='':
                print('Working File:' + self.WorkingFile)
            else:
                print('No Working File')
                self.FileSaved=False
            if self.FileSaved:
                print('Data Saved')
            else:
                print('Data Not Saved')
            print('Data:')
            print(str(self.DataDict))

            print('-----------')
            n=0
            print('---File Options:---')
            print('1->Load File')
            print('2->Save Data')
            print('3->Save Data As')
            print('4->List Files in Directory')
            print('5->Delete Data File')
            print('---Data Options:---')
            print('6->Save Current Joint State')
            print('7->Save Current Pose')
            print('8-> Save Input JS')
            print('9-> Save Input Pose')
            print('10-> Import from Text File')
            print('11-> TODO Export To Text File')

            print('12-> Save Boolean')
            print('13-> Save Number')
            print('14-> Save Text')

            if len(self.DataDict)>=1:
                print('15->Delete Points in File')
                print('16->List Points in File')
                print('17->Edit Points in File')

            print('Exit->0')

            I=input()
            #Load, Save, Save As, Delete

            if I=='1': #Load Data File
                print('Loading Data:')
                self.load_file()

            elif I=='2': #Save Data to File
                print('Saving Data')
                self.save_file()

            elif I=='3': #Save Data to File As
                print('Saving Data As:')
                self.save_file()

            elif I=='4': #List Files
                print('Delete File')
                self.list_files()

            elif I=='5': #Delete data
                print('Delete File')
                self.delete_file()

            elif I=='6' : #Save Current Joint State Point
                print('Storing Current Joint State')
                JS=self.Robot.get_current_joints()
                self.store_data(JS)

            elif I=='7' : #Save Current Joint State Point
                print('Storing Current Pose')
                P=self.Robot.get_current_pose()
                self.store_data(P)

            elif I=='8' : #Save input Joint State Point
                print('Storing input Joint State')
                JS=self.Robot.input_js()
                self.store_data(JS)

            elif I=='9' : #Save input Pose Point
                print('Storing Current Pose')
                P=self.Robot.input_pose()
                self.store_data(P)

            elif I=='10' : #Import data from file
                print('Importing Data from File')
                self.import_data()

            elif I=='12' : #Save input Joint State Point
                print('Storing Boolean')
                Input=input('Store True or False?[T/F] ')
                if 'T' in Input.upper():
                    Bool=True
                if 'F' in Input.upper():
                    Bool=False
                self.store_data(Bool)

            elif I=='13' : #Save input Joint State Point
                print('Storing Number')
                Num=float(input('Write Number to Store: '))
                self.store_data(Num)

            elif I=='14' : #Save input Joint State Point
                print('Storing Text')
                Txt=input('Write Text to Store')
                self.store_data(Txt)

            elif I=='15' and len(self.DataDict)>=1: #Delete Points
                print('Deleting Points')
                self.delete_point()

            elif I=='16' and len(self.DataDict)>=1: #List Points in File
                print('List of Point names')
                print(self.DataDict.keys())

            elif I=='17'  and len(self.DataDict)>=1: #Edit Points in File
                print('Starting Point Editor')
                #self.point_editor(DataDict, FileSaved)

            elif I=='0':
                E=1
            else:
                print('Input Error')
def main():
    Data=DataManagement()
    Data.store_data('test1point1','p1')
    Data.save_file('test1')
    print(Data.DataDict)
    Data.save_file('test2','A')
    Data.store_data('test2point1','p1','O')
    Data.store_data('test2point2','p2')
    print(Data.DataDict)
    Data.load_file('test1')
    Data.store_data('test1point2','p2')
    Data.delete_point('p1', 'Y')
    print(Data.DataDict)
    Data.load_file('test3','Y')
    print(Data.DataDict)
    Data.delete_file('test1','Y')
    Data.delete_file('test2','Y')
    Data.delete_file('test3','Y')

    Data.file_editor()


if __name__ == "__main__":
    main()
