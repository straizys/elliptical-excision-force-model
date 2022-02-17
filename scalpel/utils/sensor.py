#!/usr/bin/env python
import sys, serial
import numpy as np
from collections import deque
import itertools
import csv
from datetime import datetime
import time
import rospy
from std_msgs.msg import Int32MultiArray

class Sensor:
    
    def __init__(self, strPort, maxLen):
        
        self.ser = serial.Serial(strPort, 9600)

        self.ser.write(b'2')  # start the data stream

        line = self.ser.readline()  # ignore first 100 msgs
        print(line)

        self.ax = deque([0.0] * maxLen) # active coil
        self.ay = deque([0.0] * maxLen) # reference coil
        self.az = deque([0.0] * maxLen) # differential, drift compensated
        self.maxLen = maxLen

        self.run = False
        self.base = 0
        self.trackPause = False

        # print('calibration')
        # self.cali = self.calibration()

    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)

    def add(self, data):
        assert (len(data) == 2)
        self.addToBuf(self.ax, int(data[0][1:], 16))
        self.addToBuf(self.ay, int(data[1][1:], 16))

        diff = int(data[0][1:], 16) - int(data[1][1:], 16)
        data = self.track_base(diff)

        self.addToBuf(self.az, data)

    def track_base(self, datain):

        if (datain - self.base) > 3000:
            self.trackPause = True
        else:
            self.trackPause = False

        if not self.run:
            self.base = datain
            self.run = True
        else:
            if not self.trackPause:
                if datain > self.base:
                    self.base += 300
                else:
                    self.base -= 300

        data = datain - self.base

        return data

    def update(self):
        line = self.ser.readline()
        data = [val for val in line.split()]
        self.add(data[:2])

    def close(self):
        # close serial
        self.ser.flush()
        self.ser.close()

    def calibration(self):

        cal_len = 100
        cal_val = np.zeros([2, cal_len])

        for i in range(cal_len):
            line = self.ser.readline()  # ignore first 100 msgs...
            data = [val for val in line.split()]
            cal_val[0, i] = int(data[0][1:], 16)
            cal_val[1, i] = int(data[1][1:], 16)

        return cal_val.mean(axis=1)

def main():
    
    strPort = '/dev/ttyACM0'
    print('reading from serial port %s...' % strPort)

    sensor = Sensor(strPort, 100)

    pub = rospy.Publisher('sensor', Int32MultiArray, queue_size=10)
    rospy.init_node('sensor_node', anonymous=True)
    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():
        sensor.update()
        data_to_send = Int32MultiArray()
        data_to_send.data = np.asarray([sensor.ax[0],sensor.ay[0],sensor.az[0]])
        pub.publish(data_to_send)
        rate.sleep()

if __name__ == '__main__':
    main()
