# purpose is to republish tf as an Euler for easier time understanding
import rospy
import time
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Point, Quaternion

def callback(data):
    for transform in data.transforms:
        quat = (transform.transform.rotation.x, transform.transform.rotation.y,\
        transform.transform.rotation.z, transform.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        print("roll {}, pitch {}, yaw {}".format(euler[0], euler[1], euler[2]))



def listener():

    rospy.init_node('hello')

    rospy.Subscriber('/tf', TFMessage, callback)

    rospy.spin()


listener()
