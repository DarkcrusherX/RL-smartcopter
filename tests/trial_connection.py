#!/usr/bin/env python
import string
import rospy
from gazebo_connection import GazeboConnection

rospy.init_node('test_connection_gazebo')
g = GazeboConnection()

g.pauseSim()

print("paused")

name = str(input("Enter the name of the robot: "))

g.spawn(name)

p = input("Do u want to delete model: ")

g.delete_model(name)

g.unpauseSim()