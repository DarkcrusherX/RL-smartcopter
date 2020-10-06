#!/usr/bin/env python

import rospy
from utils.gazebo_connection import GazeboConnection

g = GazeboConnection()

# g.pauseSim()

print("paused")

lol = input("continue? : ")

g.spawn()

lol = input("continue?: ")

g.unpauseSim()

lol = input("continue?: ")

g.pauseSim()

lol = input("continue?: ")

g.delete_model()
g.resetSim()

g.unpauseSim()