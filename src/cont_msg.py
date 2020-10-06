import rospy

from geometry_msgs.msg import TwistStamped

rospy.init_node('cont_position', anonymous=True)

def vel_callback(vel):
    velocity = vel

sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback )

velocity = TwistStamped()
   
if __name__ == '__main__':
    

    sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback)
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
    t0 = rospy.Time.now().to_sec() 
    while rospy.Time.now().to_sec() - t0 > 0.1 :
        t0 = rospy.Time.now().to_sec() 
        velocity_publisher.publish(velocity)

    rospy.spin()