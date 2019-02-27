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

import xml.etree.ElementTree as et

import numpy as np

session = ''
memoryService = ''
Robot_Name = rospy.get_param('robot_name')
Config_File = rospy.get_param('config_file')

CurrentMax = 0.0

ts = 0.0
td = 0.0
TAU = 0.0

sensorTopic = ''

Sensors = []

logger = qi.Logger("exertion-stimulus-node")


def loadSensors():
    global Sensors, Config_File, Robot_Name
    
    root = et.parse(Config_File).getroot()
    sensors = root.findall(".//*[@name='"+Robot_Name+"']/characters/*[@name='default']/*[@topicname='current_stimulus']/*")
    i=0
    while i < len(sensors):
        Sensors.append(sensors[i].attrib['name'])
        i+=1
    print Sensors
    

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
    global session, memoryService
    memoryService = session.service("ALMemory")


def topicsSubscription():
    rospy.init_node(Robot_Name + '_exertion_stimulus', anonymous=True)


def getCurrent(sensor):
    global memoryService
    val = '{:.4f}'.format(0.0)
    try:
        val = '{:.4f}'.format(memoryService.getData("Device/SubDeviceList/"+sensor+"/ElectricCurrent/Sensor/Value"))
    finally:
        return float(val)


def getTotalCurrent():
    global Sensors
    tot = 0
    for sensor in Sensors:
        tot += getCurrent(sensor)
    #print "Total body current:", tot
    return tot


def getCurrentMax(sensor):
    global memoryService
    val = '{:.4f}'.format(0.0)
    try:
        val = '{:.4f}'.format(memoryService.getData("Device/SubDeviceList/"+sensor+"/ElectricCurrent/Sensor/Max"))
    finally:
        return float(val)
    

def initCurrentPain():
    global Sensors, CurrentMax, ts, td, TAU
    for sensor in Sensors:
        CurrentMax += getCurrentMax(sensor)
    TAU = rospy.get_param('tau')
    ts = 0.0
    td = 5.0 * TAU
        
    
def Somatosensory():
    global CurrentMax, ts, td, TAU
    
    x = 0.0
    if getTotalCurrent() > (0.1 * CurrentMax):
        x = (1-np.exp(-ts/TAU))
        if x < 1.0 and x > 0.0:
            td = -TAU * np.log(x)
        ts= ts + 1.0
    else:
        x = (np.exp(-td/TAU))
        if x > 0.0:
            ts = -TAU * np.log(1 - x)
        td = td + 1.0
    return x


def endJob():
    return 0


if __name__ == "__main__":
    loadSensors()
    connectRobot()
    activeServices()
    topicsSubscription()
    initCurrentPain()

    rate = rospy.Rate(5) # hz
        
    sensorTopic = rospy.Publisher(Robot_Name + '/bodily_exertion_stimulus', Float32, queue_size=0)
    
    logger.info("ROS Node started")

    try:
        while not rospy.is_shutdown():
            sensorTopic.publish(Somatosensory())
            rate.sleep()
            
    finally:
        endJob()    
    