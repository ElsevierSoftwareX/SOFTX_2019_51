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
import time
#import sys

import rospy
from std_msgs.msg import Float32

#import numpy as np

import xml.etree.ElementTree as et

session = ''
memoryService = ''
Robot_Name = rospy.get_param('robot_name')
Config_File = rospy.get_param('config_file')

#Robot_Name = 'naored'
character = rospy.get_param('character')

publisher_by_topic = {}
mod_by_topic = {}

logger = qi.Logger("roboception-node")


def callbackF(data, tn):
    global publisher_by_topic, mod_by_topic
    publisher_by_topic[tn].publish(data.data * float(mod_by_topic[tn]))


def topicSubscription(topicName):
    rospy.Subscriber('/' + topicName, Float32, callbackF, topicName)


def topicPublisher(topic, publisher):
    global publisher_by_topic
    publisher_by_topic[topic]=rospy.Publisher('/'+publisher, Float32, queue_size=0)

    
def xmlParser(sCharacter):
    global Robot_Name, Config_File, mod_by_topic
    
    root = et.parse(Config_File).getroot()

    roboceptions = root.findall(".//*[@name='"+Robot_Name+"']/characters/*[@name='"+sCharacter+"']/*")
    i=0
    while i < len(roboceptions):
        roboception = roboceptions[i].attrib['name']
        print roboception
        sensors = root.findall(".//*[@name='"+Robot_Name+"']/characters/*[@name='"+sCharacter+"']/*[@name='"+roboception+"']/*")
        j=0
        while j < len(sensors):            
            if sensors[j].attrib['inibh'] == '1':
                topic = Robot_Name+'/'+sensors[j].attrib['name']+'_'+roboceptions[i].attrib['topicname']
                publisher = Robot_Name+'/'+sCharacter+'/'+sensors[j].attrib['name']+'_'+roboceptions[i].attrib['name']
                mod_by_topic[topic] = sensors[j].attrib['mod']
                print topic, mod_by_topic[topic]
                topicSubscription(topic)
                topicPublisher(topic, publisher)
            j+=1
        print "------"
        i+=1


def endJob():
    return 0
 

if __name__=="__main__":
    rospy.init_node(Robot_Name + '_' + character + '_roboception', anonymous=True)
    
    xmlParser(character)

    try:
        while not rospy.is_shutdown():
            time.sleep(1)

    finally:
        endJob()            
