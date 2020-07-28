#!/usr/bin/env python

import rospy
from gazebo_connection import GazeboConnection

g = GazeboConnection()

g.pauseSim()

print("paused")

name = input("Enter the name of the robot: ")

g.spawn(name)

p = input("Do u want to delete model: ")

g.delete_model(name)

g.unpauseSim()