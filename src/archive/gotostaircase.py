import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist

bridge = CvBridge()

class StaircaseBot:
   
    def __init__(self):
        rospy.init_node('opencv_example', anonymous=True)
 
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.img_subscriber = rospy.Subscriber('/mybot/camera1/image_raw',Image, self.image_callback)
 
        self.rate = rospy.Rate(10)
        self.error_dist = 0

    def show_image(self,frame):
        cv.namedWindow("Image Window")
        lower = np.array([25, 52, 72])  
        upper = np.array([102, 255, 255])
        frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(frame_HSV, lower, upper)
        mask_Open = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((10, 10)))
        mask_Close = cv.morphologyEx(mask_Open, cv.MORPH_CLOSE, np.ones((20, 20))) 
        mask_Perfect = mask_Close
        conts, h = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[-2:] 
        for c in conts:
            areas = [cv.contourArea(c) for c in conts] 
            max_index = np.argmax(areas)
            cnt=conts[max_index]
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv.circle(frame, (x + int(w*0.5), y + int(h*0.5)), 4, (0,0,255), -1) 
            (h1, w1) = frame.shape[:2] 
            cv.circle(frame, (w1//2, h1//2), 4, (255, 0, 0), -1)
            cv.circle(frame, (w1//2, h1//2), 15, (255, 0, 0), 3)
            error_d  = w1//2 - (x + int(w * 0.5))
            #print(error_d)
            y_low = 0
            if y + int(h*0.5)< h1//2:
                y_low = y + int(h*0.5)
            else:
                y_low = h1//2
            y_low -= 30
            cv.circle(frame, (w1//2,y_low), 3, (0,255,255), -1)
            cv.circle(frame, (x + int(w*0.5),y_low), 3, (0,255,255), -1)
            cv.line(frame, (x + int(w*0.5),y_low), (w1//2,y_low), (0,255,255), 2)
            print("the error_d value is",error_d)
            print("W1//2=",w1//2)
            self.error_dist = float(error_d)/float(w1//2)
            print("the dist error over here is",self.error_dist)
        img = cv.cvtColor(frame, cv.COLOR_BGR2RGB)  
        cv.imshow("Image Window",img)
        cv.waitKey(1)
    def image_callback(self,img_msg):
        rospy.loginfo(img_msg.header)
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        self.show_image(cv_image)
 
    def move2goal(self):
 
        vel_msg = Twist()
 
        while not rospy.is_shutdown():
            vel_msg.linear.x = 0.2
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0
 
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            #print(self.error_dist)
            KP = 0.1
            vel_msg.angular.z = KP*self.error_dist
            print("the dist error is ",self.error_dist)
            print("the angular velocity is",KP*self.error_dist)
            self.velocity_publisher.publish(vel_msg)
 
            self.rate.sleep()
 
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
 
        rospy.spin()
 
if __name__ == '__main__':
    try:
        x = StaircaseBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass