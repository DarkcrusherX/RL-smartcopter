#!/usr/bin/env python

import os
import subprocess
import time
import rospy
import signal

from utils.armf import armtakeoff
from utils.px4_connection import Px4Connection
from utils.gazebo_connection import GazeboConnection

rospy.init_node('main_node', anonymous=True)

## Helpers for takeoff,arm,land,etc
armf = armtakeoff()
## Fn's to restart px4 sitl
px4 = Px4Connection()
## Gazebo fn for world resets
gz = GazeboConnection()



again = 'y'

while again=='y':
    gz.pauseSim()
    gz.spawn()
    gz.unpauseSim()

    px4.launch()
    armf.arm()
    armf.takeoff()
    armf.disarm()
    px4.kill()

    # gz.pauseSim()
    gz.delete_model()
    gz.resetSim()

    again = input("Do u want to go again: ")

