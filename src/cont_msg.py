import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import time

rospy.init_node('cont_position', anonymous=True)
velocity = TwistStamped()
current_pos = PoseStamped()

def current_pos_callback(position):
    global current_pos
    print("pos_working")
    current_pos = position  

def vel_callback(vel):
    global velocity
    print("vel_working")
    velocity = vel
    # velocity_publisher.publish(velocity)

velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback )
pos = rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)

print("annnnnuuuuuuuuuuuuuuuuuuu111")
while current_pos.pose.position.z < 5 :
    print(current_pos.pose.position.z)
    pass
print("annnununununnunununnunu2222222222222222222222222222222")
while True:
    a = time.time() 

    # sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback )
    # t0 = rospy.Time.now().to_sec() 
    # if rospy.Time.now().to_sec() - t0 > 0.01 :
    # print(rospy.Time.now().to_sec() - t0)
    # t0 = rospy.Time.now().to_sec()
    velocity.twist.linear.z = 0.1 
    velocity_publisher.publish(velocity)
    print("time diffffffffffffffffffffffffffffffffffffffffffff" ,time.time()-a)
# sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback)
# rospy.spin()