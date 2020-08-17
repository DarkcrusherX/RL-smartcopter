#!/usr/bin/env python

import os
import subprocess
import time
import rospy
import signal

class Px4Connection():
    
    # def __init__(self):
        
    def launch(self):
        process = subprocess.Popen('roslaunch RL-smartcopter px4.launch', stdout=subprocess.PIPE, shell=True)
        for  i in range(10):
            print("Wait for  px4.launch to connect (10 sec): {}".format(i))
            time.sleep(1)
        return process

    def kill(self,process):
        # os.kill(process.pid, signal.SIGKILL)
        child_pid = process.pid
        print("Process pid to be death awaits u all is : {}".format(process.pid))
        if child_pid is None:
            pass
        else:
            # os.kill(child_pid, signal.SIGINT)
            # process.terminate()
            process.send_signal(signal.SIGINT)
        for  i in range(10):
            print("Wait for  px4.launch to die (10 sec): {}".format(i))
            time.sleep(1)

