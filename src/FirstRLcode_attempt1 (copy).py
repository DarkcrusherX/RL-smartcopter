# import rospy
# import time
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# import numpy as np
# import math
# from collections import deque
# from geometry_msgs.msg import TwistStamped
# from geometry_msgs.msg import PoseStamped
# import tf
# from sensor_msgs.msg import Imu

# dbridge = CvBridge()

# from tensorflow import keras
# from tensorflow.keras.models import Model
# from tensorflow.keras.layers import Conv2D, MaxPooling2D, Input, Conv2DTranspose, Concatenate, Flatten, Dense , Activation , Dropout , BatchNormalization
# from tensorflow.keras.optimizers import Adam, SGD
# from tensorflow.keras.callbacks import ModelCheckpoint, ReduceLROnPlateau, EarlyStopping
# from tensorflow.keras import backend as K
# from tensorflow.keras.utils import plot_model
# from std_msgs.msg import String

# import os
# import subprocess
# import time
# import rospy
# import signal

# from utils.armf import armtakeoff
# from utils.px4_connection import Px4Connection
# from utils.gazebo_connection import GazeboConnection


# dbridge = CvBridge()
# rospy.init_node('RL_agent', anonymous=True)

# class Agent:
#     def __init__(self):
#         self.gamma = 0.95
#         self.lr = 0.01
#         self.epsilon = 1
#         self.nruns = 1000
#         self.sub_image = rospy.Subscriber("/camera/depth/image_raw", Image, self.image_callback)
#         self.rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.current_pos_callback)
#         self.imu = rospy.Subscriber("/mavros/imu/data", Imu , self.imu_callback)
#         self.goal_sub = rospy.Subscriber("goal", PoseStamped , self.goal_callback)
#         self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',Twist, queue_size=10)
#         self.drone_qvalues_model = self.drone_model()
#         self.target_drone_qvalues_model = self.drone_model()
#         self.memory = deque(maxlen=50)
#         self.target_count = 0
#         self.current_pos = PoseStamped()
#         self.imu_data = None
#         self.rate = rospy.Rate(10)
#         self.collision = False
#         self.target_reached = False
#         self.goal_radius = 0.5
#         self.rewards = []
#         self.episode = []
#         self.max_steps = 0
#         self.max_incl = 0.4
#         self.goal = PoseStamped()
#         ## Helpers for takeoff,arm,land,etc
#         self.armf = armtakeoff()
#         ## Fn's to restart px4 sitl
#         self.px4 = Px4Connection()
#         ## Gazebo fn for world resets
#         self.gz = GazeboConnection()
#         self.imu_check = rospy.Subscriber("pitch_roll_fail", String, self.imu_callback)

#     def image_callback(self,img_msg):
#         self.current_state = dbridge.imgmsg_to_cv2(img_msg,"8UC1")
#         cv2.imshow(self.current_state)

#     def current_pos_callback(self,position):
#         self.current_pos = position
    
#     def imu_callback(self, imu_data_1):
#         self.imu_data = imu_data_1.data

#     def goal_callback(self, goal_pos):
#         self.goal = goal_pos

#     def check_if_done(self, goal):
#         # euler = tf.transformations.euler_from_quaternion([self.imu_data.orientation.x,self.imu_data.orientation.y,self.imu_data.orientation.z,self.imu_data.orientation.w])
#         # roll = euler[0]
#         # pitch = euler[1]
#         # yaw = euler[2]
#         # roll_bad = not(-self.max_incl < roll < self.max_incl)
#         # pitch_bad = not(-self.max_incl < pitch < self.max_incl)
#         # if pitch_bad==True or roll_bad==True:
#         #     self.collision = True
#         #     return True
#         current_pos = self.current_pos
#         current_pos = np.array([current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z])
#         if self.imu_data == "Fail":
#             self.collision = True
#             return True
#         goal_distance = math.sqrt((math.pow(goal[0]-current_pos[0],2)) + (math.pow(goal[1]-current_pos[1],2)) + (math.pow(goal[2]-current_pos[2],2)))
#         if goal_distance<self.goal_radius:
#             self.target_reached = True
#             return True

#     def reward(self):#///////////check the rewards
#         reward = -1
#         if self.collison == True:
#             reward -= 150
#         if self.target_reached == True:
#             reward += 100
#         return reward

#     def drone_model(self):
#         Input1 = Input((480,640))
#         X1 = Conv2D(8, (3,3), strides = (1,1), padding = "same", activation = 'relu')(Input1)
#         X1 = Conv2D(16, (3,3), strides = (1,1), padding = "same", activation = 'relu')(X1)
#         X1 = Conv2D(32, (3,3), strides = (1,1), padding = "same", activation = 'relu')(X1)
#         X1 = Conv2D(64, (3,3), strides = (1,1), padding = "same", activation = 'relu')(X1)
#         X1 = Flatten()(X1)
#         X1 = Dense(800, activation = 'relu')(X1)
#         X1 = Dense(64, activation = 'relu')(X1)
#         X1 = Dense(64, activation = 'relu')(X1)
#         Input2 = Input((3))
#         X2 = Dense(128, activation = 'relu')(Input2)
#         X2 = Dense(128, activation = 'relu')(X2)
#         X2 = Dense(64, activation = 'relu')(X2)
#         X3 = Concatenate([X1,X2])
#         X3 = Dense(32, activation = 'relu')(X2)
#         X3 = Dense(6)(X2)
#         model = Model(inputs = [Input1,Input2], outputs = X3)
#         opt = Adam(lr = self.lr)
#         model.compile(loss = "mean_squared_error", optimizer = opt)
#         return model
    
#     def take_action(self,state,pos):
#         if np.random.rand < self.epsilon:
#             action = np.random.randint(6)
#         else:
#             action = np.argmax(self.drone_qvalues_model.predict([state,pos]))
#         vel=TwistStamped()
#         if action == 0:
#             # go up
#             for i in range(1000):
#                 vel.twist.linear.z = 0.2
#                 self.velocity_publisher.publish(vel)
#         if action == 1:
#             # go down
#             for i in range(1000):
#                 vel.twist.linear.z = -0.2
#                 self.velocity_publisher.publish(vel)
#         if action == 2:
#             # go left
#             for i in range(1000):
#                 vel.twist.linear.y = 0.2
#                 self.velocity_publisher.publish(vel)
#         if action == 3:
#             # go right
#             for i in range(1000):
#                 vel.twist.linear.y = -0.2
#                 self.velocity_publisher.publish(vel)
#         if action == 4:
#             # go front
#             for i in range(1000):
#                 vel.twist.linear.x = 0.2
#                 self.velocity_publisher.publish(vel)
#         if action == 5:
#             # go back
#             for i in range(1000):
#                 vel.twist.linear.x = 0.2
#                 self.velocity_publisher.publish(vel)
#         vel.linear.x = 0
#         vel.linear.y = 0
#         vel.linear.z = 0
#         self.velocity_publisher.publish(vel)
#         return action
            
#     def train_network(self):
#         batch_size = 15
# 		print("meow2")
# 		print(len(self.memory))
# 		if len(self.memory) < batch_size :
# 			return 
#         samples = random.sample(self.memory, batch_size)
#         for sample in samples:
#             current_state,current_pos, action, reward, next_state,next_pos, done = sample
#             if done!=True:
#                 next_state_qvalues = self.target_drone_qvalues_model.predict([next_state,next_pos])[0]
#                 reward = reward + self.gamma*np.max(next_state_qvalues)
#             target = self.drone_qvalues_model.predict([current_state,current_pos])[0]
#             target[action] = reward
#             self.drone_qvalues_model.fit([current_state,current_pos], target, verbose = 0)
#             self.target_count += 1

#     def update_target_network(self):
#         model_weights = self.drone_qvalues_model.get_weights()
#         model_weights = self.target_drone_qvalues_model.get_weights()
#         self.target_drone_qvalues_model.set_weights(model_weights)

    
#     def execute(self):
#         episode_number = 0
#         for episode in range(100):
#             total_reward = 0
#             #start gazebo,px4,spawn and setup(arming and stuff) the  drone
#             #set the goal_pos coordinates in an array/////////////////

#             self.gz.pauseSim()
#             self.gz.spawn()
#             self.gz.unpauseSim()

#             self.px4.launch()
#             self.armf.arm()
#             self.armf.takeoff()
#             count = 0
#             while(self.collision == True or self.target_reached == True or count <= self.nruns): #/////////////////hegkwegjhegkwegj
#                 count = count + 1
#                 for i in range(50):
#                     current_state = self.current_state
#                     current_pos = self.current_pos
#                     next_state = self.current_statehegkwegj
#                     next_pos = self.current_pos
#                     action = 4
#                     done = False
#                     self.memory.append([current_state,current_pos, action, reward, next_state, next_pos, done])

#                 current_state = self.current_state
#                 current_pos = self.current_pos
#                 current_pos = np.array([current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z])
#                 #this is the relative current position
#                 goal_pos = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])
#                 current_pos_rel[0] = goal_pos[0] - current_pos[0]
#                 current_pos_rel[1] = goal_pos[1] - current_pos[1]
#                 current_pos_rel[2] = goal_pos[2] - current_pos[2]
#                 action = take_action(self,current_state,current_pos)
#                 next_state = self.current_state
#                 next_pos = self.current_pos
#                 # done = self.check_if_done(current_state,current_pos,next_state,next_pos)
#                 done = self.check_if_done(goal_pos)
#                 reward = self.reward()
#                 total_reward += reward
#                 self.memory.append([current_state,current_pos, action, reward, next_state, next_pos, done])
# 				print("meow")
#                 self.train_network()
#                 self.epsilon = max(0.01,self.epsilon*0.995)
#                 self.max_steps += 1
#                 if self.target_count == 1000:
#                     self.update_target_network
#                     self.target_count == 0
#                 if self.collison == True or self.target_reached == True or self.max_steps==20000:
#                     self.collison = False
#                     self.target_reached = False
#                     self.max_steps = 0
#                     episode_number += 1
#                     break
#             self.rewards.append(total_reward)
#             self.episode.append(episode_number)
#             print("Reward for episode ",episode_number,":",total_reward)

#             self.armf.disarm()
#             self.px4.kill()

#             # gz.pauseSim()
#             self.gz.delete_model()
#             self.gz.resetSim()

#     return self.rewards, self.episode, self.drone_qvalues_model
            #////////kill px4, gazebo

import matplotlib.pyplot as plt

if __name__ == '__main__':
	agent = Agent()
	rewards, episode_list, qvalues_model = agent.execute()
	plt.plot(episode_list,rewards)
        

