#!/usr/bin/env python2
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


rospy.init_node('realsense_odom_instead_of_gps', anonymous=True)

def pos_callback(pos_msg):
    position = Pose()
    i=0
    while pos_msg.name[i] != "iris":
        i=i+1
    # print(i)
    position = pos_msg.pose[i]
    #print(position)
    pos = PoseStamped()
    pos.header.stamp = rospy.Time.now()
    pos.header.frame_id = "map"
    pos.pose.position.x = round(position.position.x,12)
    pos.pose.position.y = round(position.position.y,12)
    pos.pose.position.z = round(position.position.z,12)
    pos.pose.orientation.x = round(position.orientation.x,12)
    pos.pose.orientation.y = round(position.orientation.y,12)
    pos.pose.orientation.z = round(position.orientation.z,12)
    pos.pose.orientation.w = round(position.orientation.w,12)
    # print(pos)
    pub_pos.publish(pos)


sub_pos = rospy.Subscriber("/gazebo/model_states", ModelStates, pos_callback)
pub_pos = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=30)

rospy.spin()