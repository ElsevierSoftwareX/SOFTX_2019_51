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

session = ''
memoryService = ''
Robot_Name = rospy.get_param('robot_name')
Config_File = rospy.get_param('config_file')

tau = rospy.get_param('tau')
step = 0.0
t = 0.0
S0 = 0.0
Energy = 0.0
Charging = False

sensorTopic = ''

logger = qi.Logger("forcefulness-stimulus-node")


def connectRobot():
    global session
    Robot_IP =  rospy.get_param('robot_ip')
    Robot_Port = rospy.get_param('robot_port')
    
    session = qi.Session()
    
    try:
        session.connect("tcp://" + Robot_IP + ":" + Robot_Port)
    except RuntimeError:
        logger.fatal("\nCan't connect to Naoqi at ip " + Robot_IP + " on port " + Robot_Port + "\nPlease check your script arguments.\n")
        sys.exit(1)

def activeServices():
    global session, memoryService
    memoryService = session.service("ALMemory")


def topicsSubscription():
    rospy.init_node(Robot_Name + '_forcefulness_stimulus', anonymous=True)


def getBatteryCharge():
    global memoryService
    val = 0.0
    try:
        val = memoryService.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
    finally:
        return float('{:.4f}'.format(val))


def getBatteryCharging():
    global memoryService
    try:
        val = memoryService.getData("BatteryChargingFlagChanged")
    finally:
        return float(val)


def Somatosensory():
    global t, tau, S0, Energy, Charging
    
    x = 0.0
    step = 1.0
    bc = getBatteryCharge()
    if getBatteryCharging() > 0.0:
        if not Charging:
            Charging = True
            S0 = Energy
            t = 0.0
        T = (tau * 0.5) / bc
        x = (S0 - 1.0) * np.exp(-t/T) + 1.0
    else:
        if Charging:
            Charging = False
            S0 = Energy
            t = 0.0
        T = (tau * 2.0) * bc
        x = S0 * np.exp(-t/T)
        if (x < (bc * 0.5)):
            print x
            x = (x + bc) / 2.0
    t = t + step
    print t, T, bc, S0, Charging
    return float('{:.4f}'.format(x))


def endJob():
    return 0


if __name__ == "__main__":
    connectRobot()
    activeServices()
    topicsSubscription()
    
    rate = rospy.Rate(1) # hz

    S0 = getBatteryCharge()       
    Energy = S0
 
    sensorTopic = rospy.Publisher(Robot_Name + '/energy_forcefulness_stimulus', Float32, queue_size=0)
    
    logger.info("ROS Node started")
    
    try:
        while not rospy.is_shutdown():
            Energy = Somatosensory()
            sensorTopic.publish(Energy)
            rate.sleep()
            
    finally:
        endJob()
