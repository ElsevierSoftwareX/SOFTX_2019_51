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

session = ''       # Robot connection session
memoryService = '' # Robot memory service (ALMemory)
Robot_Name = rospy.get_param('robot_name')
Config_File = rospy.get_param('config_file')

TemperatureMax = {} # Max temperature vector
ts = {}  # Vector of ascent instant of each sensor
td = {}  # Vector of descent instant of each sensor
TAU = {} # Vector of exponential function time constants

sensorTopic = {} # Vector of sensors ROS topics

Sensors = [] # Vector of the sensors loaded from the configuration file

logger = qi.Logger("temperature-stimulus-node")


def loadSensors():
    global Sensors, Config_File, Robot_Name
    
    root = et.parse(Config_File).getroot()
    sensors = root.findall(".//*[@name='"+Robot_Name+"']/characters/*[@name='default']/*[@topicname='temperature_stimulus']/*")
    i=0
    while i < len(sensors):
        Sensors.append(sensors[i].attrib['name'])
        i+=1
    

def connectRobot():
    global session
    Robot_IP =  rospy.get_param('robot_ip')
    Robot_Port = rospy.get_param('robot_port')
    
    session = qi.Session()
    
    try:
        session.connect("tcp://" + Robot_IP + ":" + Robot_Port)
    except RuntimeError:
        print ("Can't connect to Naoqi at >>" + Robot_IP + "<< on port " + Robot_Port + "\n Please check your script arguments. Run with -h option for help.")
        sys.exit(1)


def activeServices():
    global session, memoryService
    memoryService = session.service("ALMemory")


def topicsSubscription():
    rospy.init_node(Robot_Name + '_temperature_stimulus', anonymous=True)


def getTemperature(sensor):
    global memoryService
    val = '{:.4f}'.format(0.0)
    try:
        val = '{:.4f}'.format(memoryService.getData("Device/SubDeviceList/"+sensor+"/Temperature/Sensor/Value"))
    finally:
        return float(val)


def getTemperatureMax(sensor):
    global memoryService
    val = '{:.4f}'.format(0.0)
    try:
        val = '{:.4f}'.format(memoryService.getData("Device/SubDeviceList/"+sensor+"/Temperature/Sensor/Max"))
    finally:
        return float(val)
    

def initTemperaturePain():
    global Sensors, TemperatureMax, ts, td, TAU
    
    for sensor in Sensors:
        TemperatureMax[sensor] = getTemperatureMax(sensor)
        TAU[sensor] = rospy.get_param('tau_t')
        ts[sensor] = 0.0
        td[sensor] = 5.0 * TAU[sensor]
        
    
def Somatosensory(sensor):
    global TemperatureMax, ts, td, TAU
    
    stimulus = 0.0
    if getTemperature(sensor) > (0.7 * TemperatureMax[sensor]):
        stimulus = (1-np.exp(-ts[sensor]/TAU[sensor]))
        if stimulus < 1.0 and stimulus > 0.0:
            td[sensor] = -TAU[sensor] * np.log(stimulus)
        ts[sensor] = ts[sensor] + 1.0
    else:
        stimulus = (np.exp(-td[sensor]/TAU[sensor]))
        if stimulus > 0.0:
            ts[sensor] = -TAU[sensor] * np.log(1 - stimulus)
        td[sensor] = td[sensor] + 1.0
    return stimulus


def endJob():
    return 0


if __name__ == "__main__":
    loadSensors()
    connectRobot()
    activeServices()
    topicsSubscription()
    initTemperaturePain()
        
    for sensor in Sensors:
        sensorTopic[sensor] = rospy.Publisher(Robot_Name + '/'+sensor+'_temperature_stimulus', Float32, queue_size=0)
    
    rate = rospy.Rate(0.2) # Hz
    
    logger.info("ROS Node started")

    try:
        while not rospy.is_shutdown():
            for sensor in Sensors:
                sensorTopic[sensor].publish(Somatosensory(sensor))
            rate.sleep()
            
    finally:
        endJob()    
    