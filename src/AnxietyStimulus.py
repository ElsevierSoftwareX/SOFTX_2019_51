#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

# Copyright 2018 by icar.cnr.it
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import qi
import sys

import rospy
from std_msgs.msg import Float32

import numpy as np

session = ''       # Robot connection session
memoryService = '' # Robot memory service (ALMemory)
sonarService = ''  # Robot sonar service (ALSonar)
S = [] # LIFO buffer of last two sonars distances (metres)
State = 0 # Anxiety Status

A = 0.0 # Anxiety stimulus
ts = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] # Vector of ascent instant of each anxiety status
td = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] # Vector of descent instant of each anxiety status
RC = [0.0,80.0,20.0] # Vector of of exponential function time constants

Robot_Name = rospy.get_param('robot_name')
Config_File = rospy.get_param('config_file')
personalZone = rospy.get_param('personalZone') # Personal zone distance (metres) loaded from ROS launcher or line command parameter
intimateZone = rospy.get_param('intimateZone') # Intimate zone distance (metres) loaded from ROS launcher or line command parameter

logger = qi.Logger("anxiety-node")


def connectRobot():
    global session
    Robot_IP =  rospy.get_param('robot_ip')
    Robot_Port = rospy.get_param('robot_port')
    
    session = qi.Session()
    
    try:
        session.connect("tcp://" + Robot_IP + ":" + Robot_Port)
    except RuntimeError:
        print ("Can't connect to Naoqi at ip " + Robot_IP + " on port " + Robot_Port + "\n Please check your script arguments. Run with -h option for help.")
        sys.exit(1)


def activeServices():
    global session, memoryService, sonarService
    memoryService = session.service("ALMemory")
    sonarService = session.service("ALSonar")
    sonarService.subscribe("anxietyApp")


def topicsSubscription():
    rospy.init_node(Robot_Name + '_anxiety_stimulus', anonymous=True)


def getDistance():
    global memoryService
    Dist=[]
    
    memoryService.insertData("Device/SubDeviceList/US/Actuator/Value", 0.0)
    Dist.append(memoryService.getData("Device/SubDeviceList/US/Left/Sensor/Value"))

    memoryService.insertData("Device/SubDeviceList/US/Actuator/Value", 1.0)
    Dist.append(memoryService.getData("Device/SubDeviceList/US/Right/Sensor/Value"))

    memoryService.insertData("Device/SubDeviceList/US/Actuator/Value", 2.0)
    Dist.append(memoryService.getData("Device/SubDeviceList/US/Left/Sensor/Value"))

    memoryService.insertData("Device/SubDeviceList/US/Actuator/Value", 3.0)
    Dist.append(memoryService.getData("Device/SubDeviceList/US/Right/Sensor/Value"))
    print min(Dist)
    return min(Dist)      


def setState():
    global personalZone, intimateZone, S, State
    if (S[0]>personalZone and S[1]<=personalZone):   
        State = 1 # From the public zone (distance greater than personal zone) to the personal zone
    if (S[1]>personalZone and S[0]<=personalZone):  
        State = 2 # From the personal zone to the public zone
    if (S[0]>intimateZone and S[1]<=intimateZone):  
        State = 3 # From the personal zone to the intimate zone
    if (S[1]>intimateZone and S[0]<=intimateZone):  
        State = 4 # From the intimate zone to the personal zone
    if (S[0]>personalZone and S[1]<=intimateZone):  
        State = 5 # From the public zone to the intimate zone
    if (S[1]>personalZone and S[0]<=intimateZone):  
        State = 6 # From the intimate zone to the public zone


def anxiety():
    global State, A, ts, td, RC
    if (State==1):
        A=(1-np.exp(-ts[1]/RC[1]))
        ts[1]=ts[1]+1.0
        if A > 0.0 and A < 1.0:    
            td[2]=-RC[2]*np.log(A)
            ts[3]=-RC[2]*np.log(1-A)
    
    if (State==2):
        A=np.exp(-td[2]/RC[2])
        td[2]=td[2]+1.0
        if A > 0.0  and A < 1.0:    
            ts[1]=-RC[1]*np.log(1-A)
            ts[5]=-RC[2]*np.log(1-A)
    
    if (State==3):
        A=(1-np.exp(-ts[3]/RC[2]))
        ts[3]=ts[3]+1.0
        if A > 0.0  and A < 1.0:    
            td[6]=-RC[1]*np.log(A)
            ts[4]=-RC[1]*np.log(1-A)
    
    if (State==4):
        A=(1-np.exp(-ts[4]/RC[1]))
        ts[4]=ts[4]+1.0
        if A > 0.0  and A < 1.0:    
            td[2]=-RC[2]*np.log(A)
            ts[3]=-RC[2]*np.log(1-A)
    
    if (State==5):
        A=(1-np.exp(-ts[5]/RC[2]))
        ts[5]=ts[5]+1.0
        if A > 0.0  and A < 1.0:    
            td[6]=-RC[2]*np.log(A)
            ts[4]=-RC[1]*np.log(1-A)
    
    if (State==6):
        A=(np.exp(-td[6]/RC[1]))
        td[6]=td[6]+1.0
        if A > 0.0  and A < 1.0:    
            ts[1]=-RC[1]*np.log(1-A)
            ts[5]=-RC[2]*np.log(1-A)
    return float('{:.4f}'.format(A))


def initAnxiety():
    return 0
        
    
def endJob():
    global sonarService
    sonarService.unsubscribe("anxietyApp")
    return 0


if __name__ == "__main__":

    connectRobot()
    activeServices()
    topicsSubscription()
    initAnxiety()
    
    rate = rospy.Rate(5) # hz
    
    anxietyTopic = rospy.Publisher(Robot_Name + '/sonar_anxiety_stimulus', Float32, queue_size=0)   
   
    logger.info("ROS Node started")
    
    S.append(10) # Set initial distance (metres)
    try:
        while not rospy.is_shutdown():
            if len(S) == 2:
                del S[0]
            S.append(getDistance())
            setState()
            anxietyTopic.publish(anxiety())
            rate.sleep()
            
    finally:
        endJob()    
    