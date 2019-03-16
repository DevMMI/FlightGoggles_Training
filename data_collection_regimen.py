import sys
import os
import rospy
import message_filters
import cv2
import yaml
import signal
import time
from tf2_msgs.msg import TFMessage
from flightgoggles.msg import IRMarkerArray, IRMarker
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import Image, Range, Imu
from cv_bridge import CvBridge, CvBridgeError
from universal_teleop.msg import Control
from sensor_msgs.msg import Joy


gates_iterator = -1
gates_dir = "flightgoggles/flightgoggles/final_challenge_course_gates.yaml"
gates_it_name = "gate_results/gate_it.txt"
labels_dir = "data/labels"
images_dir = "data/images"
fg = open(gates_dir, "r")

recording_enabled = True
joy_msgs_filled = False
gate_storage = []
avg_time_between_images = []
label_storage = {}
control_msgs = {}

class ControlStamp:
    def __init__(self, time=-1.0, id=-1):
        self.time = time
        self.id = id

class Vector3F:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        """ Create four points that make up a gate """
        self.x = x
        self.y = y
        self.z = z
    def __eq__(self, other):
        if(other.x == self.x and other.y == self.y and other.z == self.z):
            return True
        else:
            return False

    def __ne__(self, other):
        if(not (self == other)):
            return True
        else:
            return False


class Gate:
    def __init__(self, a=Vector3F(), b=Vector3F(), c=Vector3F(), d=Vector3F()):
        """ Create four points that make up a gate """
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def __eq__(self, other):
        if(other.a == self.a and other.b == self.b and other.c == self.c  and other.d == self.d):
            return True
        if(other.a == self.b and other.b == self.a and other.c == self.d and other.d == self.c):
            return True
        else:
            return False
    def __ne__(self, other):
        if(not (self == other) ):
            return True
        else:
            return False



def setupGates(filename):
    global gate_storage
    file_str = os.path.join(gate_list_dir, filename)
    stream = open(file_str, "r")
    docs = yaml.load_all(stream)
    for doc in docs:
        for k,v in doc.items():
            for l, m in v.items():
                #print k, ":"#, m, "\n"
                gate = Gate()
                for j, vals in enumerate(m):
                    v = Vector3F(vals[0],vals[1],vals[2])
                    if j == 0:
                      gate.a = v
                    elif j == 1:
                      gate.b = v
                    elif j == 2:
                      gate.c = v
                    elif j == 3:
                      gate.d = v

                already_there = False
                for gate_ in gate_storage:
                    if(gate == gate_):
                        #print("already there")
                        already_there = True
                        pairs[len(pairs)+1] = (gate, gate_)
                        break

                if(not already_there):
                    gate_storage.append(gate)

def signal_handler(sig, frame):
    # global avg_time_between_images
    # total = float(0.0)
    # prev_val = float(0.0)
    # first = True
    # for i, val in enumerate(avg_time_between_images):
    #     if first:
    #         prev_val = float(val)
    #         first = False
    #         continue
    #     print("i {}, item {}".format(i, val))
    #     total += float(val - prev_val)
    #     prev_val = val
    #     print("difference {}".format(float(val - prev_val)))
    # print("total {} divided by {}".format(total, len(avg_time_between_images)))
    # total /= float(len(avg_time_between_images))
    #
    # print("avg time between frames {}".format(total))
    print("clearing")
    empty_vals()
    #time.sleep(3)
    print('\n Bye bye!')
    sys.stdout.flush()
    sys.exit(0)


def joy_callback(control_msg):
    global recording_enabled
    global joy_msgs_filled
    if not recording_enabled:
        return
    seconds = float(control_msg.header.stamp.to_sec())
    control_msgs[seconds] = control_msg.axes

    if not joy_msgs_filled:
        joy_msgs_filled = True
        print("recording enabled")

# def getControlsMsgs(avg_time):
#     global control_msgs
#     post_time = 0.0164 # calculated by average time between subsequent images
#     found_series = False
#     for key, val in control_msgs.iteritems():
#         if not found_series:
#             diff = avg_time - key
#             if(diff )



def callback(image_data_l, image_data_r, tf_data, lidar_data, imu_data):
    global recording_enabled
    global joy_msgs_filled
    global avg_time_between_images
    global label_storage
    if not recording_enabled or not joy_msgs_filled:
        return

    time1 = image_data_l.header.stamp.to_sec()
    time2 = image_data_r.header.stamp.to_sec()
    time3 = tf_data.header.stamp.to_sec()
    time4 = lidar_data.header.stamp.to_sec()
    time5 = imu_data.header.stamp.to_sec()
    avg_time = time1 + time2 + time3 + time4 + time5
    avg_time /= 5.0

    #print("avg time {}".format(avg_time))
    avg_time_between_images.append(float(avg_time))

    global gates_iterator
    global gates_it_name
    if gates_iterator == -1:
        print("gates iterator not found")
        exit(-1)

    # process image data #
    bridge = CvBridge()
    try:
        cv_image_l = bridge.imgmsg_to_cv2(image_data_l, "bgr8")
        cv_image_r = bridge.imgmsg_to_cv2(image_data_r, "bgr8")
    except CvBridgeError as e:
      print(e)

    #print("published")
    left_name = "left{}".format(gates_iterator) + ".jpg"
    output_image_l = os.path.join(images_dir, left_name)
    right_name = "left{}".format(gates_iterator) + ".jpg"
    output_image_r = os.path.join(images_dir, right_name)

    cv2.imwrite(output_image_l, cv_image_l)
    cv2.imwrite(output_image_r, cv_image_r)
    #print("writing image")

    # open label yaml
    data = dict(
        tf = dict(
            translation = dict(
                x = tf_data.transform.translation.x,
                y = tf_data.transform.translation.y,
                z = tf_data.transform.translation.z,
            ),
            rotation = dict(
                x = tf_data.transform.rotation.x,
                y = tf_data.transform.rotation.y,
                z = tf_data.transform.rotation.z,
                w = tf_data.transform.rotation.w,
            )
        ),
        imu = dict(
            angular_velocity = dict(
                x = imu_data.angular_velocity.x,
                y = imu_data.angular_velocity.y,
                z = imu_data.angular_velocity.z,
            ),
            linear_acceleration = dict(
                x = imu_data.linear_acceleration.x,
                y = imu_data.linear_acceleration.y,
                z = imu_data.linear_acceleration.z,
            )
        ),
        lidar = dict(
            range = lidar_data.range,
        )
    )

    # store label for safekeeping until control message comes in
    stmp = ControlStamp(avg_time, gates_iterator)
    label_storage[stmp] = data

    # iterate gate id
    gates_iterator += 1
    fp = open(gates_it_name, 'w')
    fp.write(str(gates_iterator))
    fp.close()
    #print("wrote")
    # print("shape {}".format(cv_image.shape))
    # cv2.imshow("window", cv_image)
    # cv2.waitKey(50)

def getControlDict(val):
    data = dict(
        yaw = val[0],
        vertical = val[1],
        roll = val[2],
        pitch = val[3],
    )
    return data

def empty_vals():
    print("hitting")
    global labels_dir
    global control_msgs
    global label_storage
    #print("labels stored {}\n, control msgs stored {}\n\n".format(len(label_storage), len(control_msgs)))

    if len(label_storage) == 0:
        return

    post_time = 0.0164 # calculated by average time between subsequent images
    labels_to_be_written = {}
    #found_series = False
    label_storage_cp = dict(label_storage)
    control_msgs_cp = dict(control_msgs)
    # use these instead of the real things so they dont change size while we're operating on them
    print("going through {} labels ".format(len(label_storage)))
    for key_l, val_l in label_storage_cp.iteritems():
        label_time = key_l.time
        label_id = key_l.id
        control_msgs_cp = dict(control_msgs) # update so we lose the control messages that are taken
        control_msgs_iter = 0
        for key, val in control_msgs_cp.iteritems():
            #print("key {}, label_time {}".format(key, label_time))
            if(key < label_time):
                # control msgs that are before this label
                del control_msgs[key]
            if(key > label_time):
                # control msgs after this label
                if(key - label_time < post_time):
                    # add it to label as rightful control message
                    #print("key {}".format(key))
                    label_storage[key_l]["control{}".format(control_msgs_iter)] = getControlDict(val)
                    control_msgs_iter+=1
                    del control_msgs[key]
                else:
                    # have reached point
                    break

        label_name = "label{}".format(label_id) + ".txt"
        output_dir = os.path.join(labels_dir, label_name)
        print("empty writing {}".format(output_dir))
        with open(output_dir, 'w') as outfile:
            yaml.dump(label_storage[key_l], outfile, default_flow_style=False)
        # removing written label from storage
        del label_storage[key_l]

def dual_callback(event):
    # match control messages and labels that have similar time stamps
    #control_msgs[double]
    #label_storage[stmp]
        #self.time = time
        #self.id = id
    global labels_dir
    global control_msgs
    global label_storage
    #print("labels stored {}\n, control msgs stored {}\n\n".format(len(label_storage), len(control_msgs)))

    if len(control_msgs) == 0 or len(label_storage) == 0:
        return

    post_time = 0.0164 # calculated by average time between subsequent images
    labels_to_be_written = {}
    #found_series = False
    label_storage_cp = dict(label_storage)
    control_msgs_cp = dict(control_msgs)
    # use these instead of the real things so they dont change size while we're operating on them
    for key_l, val_l in label_storage_cp.iteritems():
        label_time = key_l.time
        label_id = key_l.id
        control_msgs_cp = dict(control_msgs) # update so we lose the control messages that are taken
        for key, val in control_msgs_cp.iteritems():
            #print("key {}, label_time {}".format(key, label_time))
            if(key < label_time):
                del control_msgs[key]
            if(key > label_time):
                if(key - label_time < post_time):
                    # add it to label as rightful control message
                    #print("key {}".format(key))
                    label_storage[key_l]["control"] = getControlDict(val)
                    del control_msgs[key]
                    break
                else:
                    break

        label_name = "label{}".format(label_id) + ".txt"
        output_dir = os.path.join(labels_dir, label_name)
        with open(output_dir, 'w') as outfile:
            yaml.dump(label_storage[key_l], outfile, default_flow_style=False)
        # removing written label from storage
        del label_storage[key_l]
        #print("writing label")


        # if not found_series:
        #     diff = avg_time - key
        #     if(diff )





def main():
    signal.signal(signal.SIGINT, signal_handler)
    global gates_iterator
    rospy.init_node('collect_data_node')
    rate = rospy.Rate(10) # 10hz

    ## main data generation loop ##

    # get current number that images and labels are at
    fc = open(gates_it_name)
    gates_iterator = int(fc.read().replace('\n',''))
    fc.close()

    # dataCollector()
    # control messages
    control_sub = rospy.Subscriber("/control_nodes/joy", Joy, joy_callback)
    # sensor msgs
    image_sub_l = message_filters.Subscriber('/colored_gates_left', Image)
    image_sub_r = message_filters.Subscriber('/colored_gates_right', Image)
    tf_sub = message_filters.Subscriber('/tf/transform', TransformStamped)
    lidar_sub = message_filters.Subscriber('/uav/sensors/downward_laser_rangefinder', Range)
    imu_sub = message_filters.Subscriber('/uav/sensors/imu', Imu)
    # print("created subscribers")
    ts = message_filters.ApproximateTimeSynchronizer([image_sub_l, image_sub_r, tf_sub, lidar_sub, imu_sub], 10, 0.3)
    ts.registerCallback(callback)

    rospy.Timer(rospy.Duration(0.05), dual_callback)
    rospy.spin()


main()
