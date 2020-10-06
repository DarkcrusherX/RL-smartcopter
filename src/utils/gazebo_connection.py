#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
import time

class GazeboConnection():
    
    def __init__(self):
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        self.goal_sub = rospy.Subscriber("goal", PoseStamped , self.goal_callback)
        self.goal = PoseStamped()

    def goal_callback(self, goal_pos):
        self.goal = goal_pos

    def pauseSim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")
        
    def unpauseSim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print ("/gazebo/unpause_physics service call failed")
        
    def resetSim(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_world service call failed")

    def spawn(self):
        initial_pose = Pose()
        halflength = 50
        lol = int(halflength/5 - 1)

        arrayx = []
        arrayy = []

        for i in range(-lol,lol):
            x = 5*i +random.randint(1,4)
            y = 5*i +random.randint(1,4)
            ## Condition to check so building dont land on top of the drone
            if abs(x) > 4 or abs(y) > 4:
                if abs(self.goal.pose.position.x - x) > 4 or abs(self.goal.pose.position.y - y) > 4:               
                    arrayx.append(x)
                    arrayy.append(y)

        self.n_buildings = len(arrayx)

        print(arrayx)
        print(arrayy)

        arrayx = np.array(arrayx)
        arrayy = np.array(arrayy)
        np.random.shuffle(arrayx)
        np.random.shuffle(arrayy)

        for i in range(0,self.n_buildings):
            initial_pose.position.x = arrayx[i]
            initial_pose.position.y = arrayy[i]
            initial_pose.position.z = 20
            f = open('../models/building/model.sdf','r')
            sdf = f.read()
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            name = "building "
            name = name + str(i)
            print(name)
            try:
                self.spawn_model_prox(name, sdf,name, initial_pose, "world")
                time.sleep(0.5)
            except rospy.ServiceException as e:
                print ("/gazebo/spawn_sdf_model service call failed")

    def delete_model(self):
        rospy.wait_for_service('/gazebo/delete_model')      
        try:
            for i in range(0,self.n_buildings):
                rospy.wait_for_service('/gazebo/delete_model')  
                rospy.sleep(1)
                name = "building "
                name = name + str(i)
                print("deleting ",name)
                self.delete_model_prox(name) 
        except rospy.ServiceException as e:
            print ("/gazebo/delete_model service call failed")
