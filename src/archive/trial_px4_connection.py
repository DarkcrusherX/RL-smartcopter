#!/usr/bin/env python

import os
import subprocess
import time
import rospy
import signal
from armf import armtakeoff
from px4_connection import Px4Connection

lul = armtakeoff()
lol = Px4Connection()

rospy.init_node('bruh_onli', anonymous=True)

again = 'y'

# while again=='y':

#     process = subprocess.Popen('roslaunch RL-smartcopter px4.launch', stdout=subprocess.PIPE, shell=True)
#     print(process)
#     print(type(process))
#     for  i in range(5):
#         print(i)
#         time.sleep(1)
#     lul.arm()

#     i = input("Do u want to kill this process")

#     os.kill(process.pid, signal.SIGKILL)

#     for  i in range(5):
#         print(i)
#         time.sleep(1)
    
#     again = input("Do u want to go again: ")

while again=='y':

    lol.launch()

    lul.arm()
    lul.takeoff()
    lul.disarm()

    lol.kill()

    again = input("Do u want to go again: ")

def myhook():
    global lol
    print("shutdown time!")
    lol.kill()

    exit(0)

rospy.on_shutdown(myhook)



