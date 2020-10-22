import rospy

from geometry_msgs.msg import TwistStamped
from utils.armf import armtakeoff

rospy.init_node('vel', anonymous=True)
velocity = TwistStamped()
armf = armtakeoff()
# armf.arm()
# armf.takeoff()
while True:
    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
    velocity.twist.linear.z = 0
    velocity.twist.linear.x = 0.1
    velocity.twist.linear.y = 0.1
   
    velocity_publisher.publish(velocity)