#!/usr/bin/env python

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import rospy
import sys

from os import listdir
from os.path import isfile, join

'''
A single entry in rostopic:

header: 
  seq: 1271
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
status: 
  - 
    level: 0
    name: "master_component"
    message: ''
    hardware_id: "0"
    values: []
  - 
    level: 0
    name: "LeftHandAction"
    message: ''
    hardware_id: "0"
    values: 
      - 
        key: ''
        value: "received a new goalTime(6238;292000000)"
      - 
        key: ''
        value: "accepted the new goal: move handTime(6238;292000000)"
      - 
        key: ''
        value: "sent commandsTime(6238;292000000)"
  - 
    level: 0
    name: "RightHandAction"
    message: ''
    hardware_id: "0"
    values: []
---
'''

def assertLine(line, line_idx, value):
    if line != value:
        raise Exception('Line "{}" ({}) should be equal to "{}"'.format(line, line_idx, value))

def assertLineBeginning(line, line_idx, value):
    if not line.startswith(value):
        raise Exception('Line "{}" ({}) should start with "{}"'.format(line, line_idx, value))

class FabricLogEntry:
    def __init__(self, subsystem_id, component_name, log_time, wall_time, value):
        self.__subsystem_id = subsystem_id
        self.__component_name = component_name
        self.__log_time = log_time
        self.__wall_time = wall_time
        self.__value = value

    def __str__(self):
        return '{}.{:09d}: {}: {}: {}'.format(self.__wall_time.secs, self.__wall_time.nsecs,
                                        self.__subsystem_id, self.__component_name, self.__value)

    def getTime(self):
        return self.__log_time

    def getWallTime(self):
        return self.__wall_time

    def getComponentName(self):
        return self.__component_name

class FabricLog:
    def __init__(self):
        self.__all_values = []

    def getValues(self):
        return self.__all_values

    def getValuesForComponents(self, components_list):
        result = []
        for value in self.__all_values:
            if components_list is None or value.getComponentName() in components_list:
                result.append(value)
        return result

    def extractTime(self, line):
        idx = line.rfind('Time(')
        line_str = line[0:idx]
        time_str = line[idx+5:-1]
        time_fields = time_str.split(';')
        assert len(time_fields) == 4
        time_s = int(time_fields[0])
        time_ns = int(time_fields[1])
        wtime_s = int(time_fields[2])
        wtime_ns = int(time_fields[3])
        return line_str, rospy.Time(time_s, time_ns), rospy.Time(wtime_s, wtime_ns)

    def parseValue(self, lines, line_idx):
        line = lines[line_idx]
        if line != '      - ':
            return line_idx, None
        line_idx = line_idx + 2
        line = lines[line_idx]
        assertLineBeginning(line, line_idx, '        value: "')
        value = line[16:-1]
        return line_idx+1, value

    def parseComponentLog(self, lines, line_idx, subsystem_id):
        line = lines[line_idx]
        if line == '---':
            return line_idx+1, None

        assertLine(line, line_idx, '  - ')
        line_idx = line_idx + 2
        line = lines[line_idx]
        assertLineBeginning(line, line_idx, '    name: "')
        component_name = line[11:-1]
        line_idx += 3
        line = lines[line_idx]
        if line == '    values: []':
            return line_idx+1, []
        assertLine(line, line_idx, '    values: ')
        line_idx = line_idx + 1
        values = []
        while True:
            line_idx, value = self.parseValue(lines, line_idx)
            if value == None:
                break
            value, log_time, wall_time = self.extractTime(value)
            values.append( FabricLogEntry(subsystem_id, component_name, log_time, wall_time, value) )
            #print(values[-1])
        return line_idx, values

    def parseMessage(self, lines, line_idx, subsystem_id):
        line = lines[line_idx]
        assertLine(line, line_idx, 'header: ')
        line_idx = line_idx + 6
        line = lines[line_idx]
        assertLine(line, line_idx, 'status: ')
        line_idx = line_idx + 1
        while True:
            line_idx, values = self.parseComponentLog(lines, line_idx, subsystem_id)
            if values is None:
                break
            self.__all_values = self.__all_values + values
        return line_idx

    def parseFile(self, filename, subsystem_id):
        with open(filename, 'r') as f:
            #lines = f.readlines()
            lines = f.read().splitlines()
        line_idx = 0
        lines_count = len(lines)
        last_print = 0
        while line_idx < lines_count:
            line_idx = self.parseMessage(lines, line_idx, subsystem_id)
            if line_idx > last_print+10000:
                print('line {} out of {}'.format(line_idx, lines_count))
                last_print = line_idx

    def sortLogs(self):
        self.__all_values = sorted(self.__all_values, key=lambda x:x.getWallTime())

def main():
    path = '.'
    #files = [
    #    #'velma_task_cs_ros_interface',
    #    'velma_sim_gazebo',
    #    #'velma_core_cs',
    #    'velma_core_ve_body',
    #]
    fabric_log = FabricLog()
    #for filename in files:
    #    print('Reading {}'.format(filename))
    #    fabric_log.parseFile('{}/{}.txt'.format(path,filename), filename)

    #if sys.argv[1] == 'left':
    #    components_list = [ 'lHand', 'LeftHand', 'LeftHandAction', 'can_queue_tx_l', 'lHand_EcCanQueue', 'LeftHand_EcCanQueue']
    #elif sys.argv[1] == 'right':
    #    components_list = [ 'rHand', 'RightHand', 'RightHandAction', 'can_queue_tx_r', 'rHand_EcCanQueue', 'RighHand_EcCanQueue']
    #else:
    #    raise Exception()

    fabric_log.parseFile(sys.argv[1], 'visual_servo_head')

    components_list = [ 'visual_servo' ]

    fabric_log.sortLogs()
    for value in fabric_log.getValuesForComponents(components_list):
        print value

    return 0

if __name__ == "__main__":
    exit( main() )
