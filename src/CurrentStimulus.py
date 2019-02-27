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

CurrentMax = {}

ts = {}
td = {}
TAU = {}

sensorTopic = {}

Sensors = []

logger = qi.Logger("current-stimulus-node")


def loadSensors():
    global Sensors, Config_File, Robot_Name
    
    root = et.parse(Config_File).getroot()
    sensors = root.findall(".//*[@name='"+Robot_Name+"']/characters/*[@name='default']/*[@topicname='current_stimulus']/*")
    print sensors
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
    rospy.init_node(Robot_Name + '_current_stimulus', anonymous=True)


def getCurrent(sensor):
    global memoryService
    val = '{:.4f}'.format(0.0)
    try:
        val = '{:.4f}'.format(memoryService.getData("Device/SubDeviceList/"+sensor+"/ElectricCurrent/Sensor/Value"))
    finally:
        return float(val)


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
        CurrentMax[sensor] = getCurrentMax(sensor)
        TAU[sensor] = rospy.get_param('tau')
        ts[sensor] = 0.0
        td[sensor] = 5.0 * TAU[sensor]
        
    
def Somatosensory(sensor):
    global CurrentMax, ts, td, TAU
    
    x = 0.0
    if getCurrent(sensor) > (0.7 * CurrentMax[sensor]):
        x = (1-np.exp(-ts[sensor]/TAU[sensor]))
        if x < 1.0 and x > 0.0:
            td[sensor] = -TAU[sensor] * np.log(x)
        ts[sensor] = ts[sensor] + 1.0
    else:
        x = (np.exp(-td[sensor]/TAU[sensor]))
        if x > 0.0:
            ts[sensor] = -TAU[sensor] * np.log(1 - x)
        td[sensor] = td[sensor] + 1.0
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
        
    for sensor in Sensors:
        sensorTopic[sensor] = rospy.Publisher(Robot_Name + '/'+sensor+'_current_stimulus', Float32, queue_size=0)
    
    logger.info("ROS Node started")

    try:
        while not rospy.is_shutdown():
            for sensor in Sensors:
                sensorTopic[sensor].publish(Somatosensory(sensor))
            rate.sleep()
    
    finally:
        endJob()    
    