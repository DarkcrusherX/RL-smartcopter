#!/usr/bin/env python

import os
import subprocess
import time
import rospy
import signal

class Px4Connection():
    def __init__(self):
        self.wait_time = 5

    def timeout(self):
        for i in range(self.wait_time):
            print("Wait for  px4.launch to connect ({} sec): {}".format(self.wait_time,i))
            time.sleep(1)


    def launch(self):
        self.p= subprocess.Popen('roslaunch RL-smartcopter px4.launch', preexec_fn=os.setsid, shell=True)
        self.timeout()

    def kill(self):
        os.killpg(os.getpgid(self.p.pid), signal.SIGTERM)
        self.timeout()

