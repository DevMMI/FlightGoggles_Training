import rospy
import time
from geometry_msgs.msg import Pose, Point, Quaternion

def talker():
    pub = rospy.Publisher('/new_pos_init', Pose, queue_size=100)

    rospy.init_node('hello')
    rate = rospy.Rate(10) # 10hz
    current_point = Point(float(0.3),float(52.0),float(2.5))
    current_quaternion  = Quaternion(float(0.0),float(0.0),float(-0.707),float(0.707))
    pose_init = Pose(current_point,current_quaternion)
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        if connections != 0:
            pub.publish(pose_init)
            time.sleep(7)
            break

talker()
