import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
#rospy.init_node('arm_and_takeoff_node', anonymous=True)
class armtakeoff():

    def __init__(self):

        def state_cb(state):
            self.current_state = state

        def current_pos_callback(position):

    
            self.current_pos = position     
        
        self.current_state = State()
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, state_cb)  # $This topic was wrong
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.pos_state = rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)
     
    
    def arm(self):

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 5

        prev_state = self.current_state
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(pose)
        rate.sleep()
        # print(self.current_state)
        # wait for FCU connection
        while not self.current_state.connected:
            print("Sleeping")
            rate.sleep()

        last_request = rospy.get_rostime()

        while (self.current_state.armed != True) or (self.current_state.mode !="OFFBOARD"): 
        #    print("Arm {}" .format(self.current_state.armed))            
            now = rospy.get_rostime()
            if self.current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now 
            else:
                if not self.current_state.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client(True)
                    last_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
            if prev_state.mode != self.current_state.mode: 
                rospy.loginfo("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            # Update timestamp and publish pose 
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            if prev_state.armed == self.current_state.armed:
                for i in range(100):
                    self.local_pos_pub.publish(pose)           
            rate.sleep()

    def takeoff(self):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 5

        while pose.pose.position.z-0.1 > self.current_pos.pose.position.z:
            now = rospy.get_rostime()
            pose.pose.position.z = 5
            self.local_pos_pub.publish(pose)

    def land_rtl(self):
        self.set_mode_client(base_mode=0, custom_mode="AUTO.RTL")
        while self.current_pos.pose.position.z > 0.5:
            pass


    def disarm(self):

        prev_state = self.current_state
        rate = rospy.Rate(20.0) # MUST be more then 2Hz

        rate.sleep()
        # wait for FCU connection
        while not self.current_state.connected:
            print("Sleeping")
            rate.sleep()

        last_request = rospy.get_rostime()

        while self.current_state.armed != False: 
            #print("Got in")
        #    print("Arm {}" .format(self.current_state.armed))
            now = rospy.get_rostime()
            if self.current_state.mode != "AUTO.LAND" and (now - last_request > rospy.Duration(5.)):
                self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
                last_request = now 
            else:
                if not self.current_state.armed and (now - last_request > rospy.Duration(5.)):
                    self.arming_client(False)
                    last_request = now 
        rate.sleep()            
