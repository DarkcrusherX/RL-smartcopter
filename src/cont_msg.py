import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import time
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode

rospy.init_node('cont_position', anonymous=True)
velocity = TwistStamped()
current_pos = PoseStamped()

def current_pos_callback(position):
    global current_pos
    current_pos = position  

def vel_callback(vel):
    global velocity
    print("vel_working")
    velocity = vel
    # velocity_publisher.publish(velocity)

current_state  =  State()  
def state_cb(state):
    global current_state 
    current_state = state


velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback )
pos = rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)


while True :
    restart = 0
    while current_state.armed != True :
        pass
    print("state1")
    while current_pos.pose.position.z < 4 :
        pass
    print("state2")
    while True:
        a = time.time() 

        # sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback )
        # t0 = rospy.Time.now().to_sec() 
        # if rospy.Time.now().to_sec() - t0 > 0.01 :
        # print(rospy.Time.now().to_sec() - t0)
        # t0 = rospy.Time.now().to_sec()
        # velocity.twist.linear.z = 0.1 
        velocity_publisher.publish(velocity)
        print(velocity)
        if current_state.armed != True : 
            print("restarting gggggggggggggggggggggggggggggg")
            restart = 1
            break
        print("time dif" ,time.time()-a)
    if restart == 1:
        continue

