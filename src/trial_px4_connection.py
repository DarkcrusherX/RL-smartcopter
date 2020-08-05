#!/usr/bin/env python

import os
import subprocess
import time
import rospy
import signal
from armf import armtakeoff

lul = armtakeoff()

rospy.init_node('bruh_onli', anonymous=True)

again = 'y'

while again=='y':

    process = subprocess.Popen('roslaunch RL-smartcopter px4.launch', stdout=subprocess.PIPE, shell=True)
    print(process)
    print(type(process))
    for  i in range(5):
        print(i)
        time.sleep(1)
    lul.arm()

    i = input("Do u want to kill this process")

    os.kill(process.pid, signal.SIGKILL)

    for  i in range(5):
        print(i)
        time.sleep(1)
    
    again = input("Do u want to go again: ")
