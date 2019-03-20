import rospy
import sys
import message_filters
import cv2
import signal
import std_msgs.msg
from flightgoggles.msg import IRMarkerArray, IRMarker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Pixel:
    def __init__(self, x=-1, y=-1):
        """ Create two points that make up a pixel """
        self.x = x
        self.y = y

class Gate:
    def __init__(self, a=Pixel(), b=Pixel(), c=Pixel(), d=Pixel()):
        """ Create four pixels that make up a gate """
        self.a = a
        self.b = b
        self.c = c
        self.d = d

publ = rospy.Publisher('colored_gates_left', Image, queue_size=100)
pubr = rospy.Publisher('colored_gates_right', Image, queue_size=100)
gate_names = rospy.get_param("/uav/gate_names", '[]')

def callback(ir_data, image_data_l, image_data_r):
    # process image data
    bridge = CvBridge()
    try:
        cv_image_l = bridge.imgmsg_to_cv2(image_data_l, "bgr8")
        cv_image_r = bridge.imgmsg_to_cv2(image_data_r, "bgr8")
    except CvBridgeError as e:
      print(e)

    relevant_gate_dict = {}

    for gates in gate_names:
        relevant_gate_dict[gates] = Gate()

        #print("gate_vals {}".format(gates))
    for marker in ir_data.markers:
        if(marker.landmarkID.data in relevant_gate_dict):
            gate = relevant_gate_dict.get(marker.landmarkID.data)
            if(marker.markerID.data == "1"):
                gate.a = Pixel(marker.x, marker.y)
            elif(marker.markerID.data == "2"):
                gate.b = Pixel(marker.x, marker.y)
            elif(marker.markerID.data == "3"):
                gate.c = Pixel(marker.x, marker.y)
            elif(marker.markerID.data == "4"):
                gate.d = Pixel(marker.x, marker.y)
            relevant_gate_dict[marker.landmarkID.data] = gate
        # print("gate {}".format(marker.landmarkID.data))
        # print("ID {}".format(marker.markerID.data))
        # print("x {}, y {}".format(marker.x, marker.y))
        # print("\n")
    for marker_ in relevant_gate_dict:
        gate_ = relevant_gate_dict[marker_]
        #print("rectangle t-l x {}, y {} : b-r x {}, y {}".format(int(gate_.a.x), int(gate_.a.y),\
         #int(gate_.d.x), int(gate_.d.y)))
        #cv2.rectangle(cv_image, (int(gate_.a.x), int(gate_.a.y)), (int(gate_.d.x), int(gate_.d.y)), (255,0,255), 2)
        if gate_.a.x != -1:
            cv_image_l = cv2.circle(cv_image_l, (int(round(gate_.a.x)), int(round(gate_.a.y))), 5, (255,0,255), -1)
        if gate_.b.x != -1:
            cv_image_l = cv2.circle(cv_image_l, (int(round(gate_.b.x)), int(round(gate_.b.y))), 5, (255,0,255), -1)
        if gate_.c.x != -1:
            cv_image_l = cv2.circle(cv_image_l, (int(round(gate_.c.x)), int(round(gate_.c.y))), 5, (255,0,255), -1)
        if gate_.d.x != -1:
            cv_image_l = cv2.circle(cv_image_l, (int(round(gate_.d.x)), int(round(gate_.d.y))), 5, (255,0,255), -1)

        if gate_.a.x != -1:
            cv_image_r = cv2.circle(cv_image_r, (int(round(gate_.a.x)), int(round(gate_.a.y))), 5, (255,0,255), -1)
        if gate_.b.x != -1:
            cv_image_r = cv2.circle(cv_image_r, (int(round(gate_.b.x)), int(round(gate_.b.y))), 5, (255,0,255), -1)
        if gate_.c.x != -1:
            cv_image_r = cv2.circle(cv_image_r, (int(round(gate_.c.x)), int(round(gate_.c.y))), 5, (255,0,255), -1)
        if gate_.d.x != -1:
            cv_image_r = cv2.circle(cv_image_r, (int(round(gate_.d.x)), int(round(gate_.d.y))), 5, (255,0,255), -1)

    left = bridge.cv2_to_imgmsg(cv_image_l, "bgr8")
    right = bridge.cv2_to_imgmsg(cv_image_r, "bgr8")
    hl = std_msgs.msg.Header()
    hr = std_msgs.msg.Header()
    hl.stamp = image_data_l.header.stamp
    hr.stamp = image_data_r.header.stamp
    left.header = hl
    right.header = hr
    publ.publish(left)
    pubr.publish(right)

    # print("published")
    #cv2.imshow("window", cv_image)
    #cv2.waitKey(100)

def signal_handler(sig, frame):
        print('\n Bye bye!')
        sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('republish_colored_gates_node')

    ir_sub = message_filters.Subscriber('/uav/camera/left/ir_beacons', IRMarkerArray)
    image_sub_l = message_filters.Subscriber('/uav/camera/left/image_rect_color', Image)
    image_sub_r = message_filters.Subscriber('/uav/camera/right/image_rect_color', Image)

    ts = message_filters.ApproximateTimeSynchronizer([ir_sub, image_sub_l, image_sub_r], 10, 0.3)
    ts.registerCallback(callback)

    rospy.spin()

main()
