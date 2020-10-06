import rospy
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import String


pub = rospy.Publisher('pitch_roll_fail', String , queue_size=10)
a = String()

def callback(imu_data_1):
    # print(imu_data)
    imu_data = imu_data_1
    max_incl = 0.4   
    euler = tf.transformations.euler_from_quaternion([imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print(roll, pitch, yaw)
    roll_bad = not(-max_incl < roll < max_incl)
    pitch_bad = not(-max_incl < pitch < max_incl)
    print(pitch_bad, roll_bad)
    if pitch_bad==True or roll_bad==True:
        print("Fuck")
        a.data = "Fail"
        pub.publish(a)    


def listener():

    rospy.init_node('listener', anonymous=True)
 
    imu = rospy.Subscriber("/mavros/imu/data", Imu , callback)

    rospy.spin()
   
if __name__ == '__main__':
    listener()