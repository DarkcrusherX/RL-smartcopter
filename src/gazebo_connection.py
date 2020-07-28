#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

class GazeboConnection():
    
    def __init__(self):
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

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

    def spawn(self,name):
        initial_pose = Pose()
        initial_pose.position.x = 1
        initial_pose.position.y = 1
        initial_pose.position.z = 1
        f = open('/home/ahal/catkin_ws/src/RL-smartcopter/models/iris_depth_camera/iris_depth_camera.sdf','r')
        sdf = f.read()
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            self.spawn_model_prox(name, sdf,name, initial_pose, "world")
        except rospy.ServiceException as e:
            print ("/gazebo/spawn_sdf_model service call failed")

    def delete_model(self,name):
        rospy.wait_for_service('/gazebo/delete_model')       
        try:
            self.delete_model_prox(name) 
        except rospy.ServiceException as e:
            print ("/gazebo/delete_model service call failed")