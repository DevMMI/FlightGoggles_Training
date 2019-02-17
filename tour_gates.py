import rospy
import time
import tf
import math
import signal
import sys
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randint
gate_list_dir = "flightgoggles/flightgoggles/config/challenges/gate_locations.yaml"

class Vector3F:
    def __init__(self, x, y, z):
        """ Create four points that make up a gate """
        self.x = x
        self.y = y
        self.z = z

class Gate:
    def __init__(self, a=0.0, b=0.0, c=0.0, d=0.0):
        """ Create four points that make up a gate """
        self.a = a
        self.b = b
        self.c = c
        self.d = d

def signal_handler(sig, frame):
        print('\n Bye bye!')
        sys.exit(0)

def setupGates():
    f = open(gate_list_dir, "r")
    gates = []
    for i, line in enumerate(f):
      # each even line should be a gate
      if (i+1) % 2 == 0:
          str_total = line[12:][1:-2]
          str_total = str_total.replace(' ', '')
          str_arrays = str_total.split("],[")
          gate = Gate()
          for j, arr in enumerate(str_arrays):

              gate_vals = arr.strip(']').strip('[').split(",")
              v = Vector3F(gate_vals[0],gate_vals[1],gate_vals[2])

              if j == 0:
                  gate.a = v
              elif j == 1:
                  gate.b = v
              elif j == 2:
                  gate.c = v
              elif j == 3:
                  gate.d = v
          gates.append(gate)
    return gates

def spawn(gate, pub):
    vectora = gate.a
    vectorb = gate.b
    vectorc = gate.c
    vectord = gate.d


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

def start_tour(gates, publisher):
    num_orientations = 12
    dist_from_gate = 2.0
    for i, gate in enumerate(gates):
        print("")
        tleft = gate.a
        tright = gate.b
        bleft = gate.c
        bright = gate.d

        # calculate position from gate
        position_z = (float(tleft.z) + float(bleft.z)) / 2.0
        position_x = (float(tleft.x) + float(tright.x)) / 2.0
        position_y = ((float(tleft.y) + float(tright.y)) / 2.0) - dist_from_gate

        # desired starting orientation
        yaw = 1.57

        # transport you there
        desired_pos = Point(position_x, position_y, position_z)
        quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
        pose = Pose(desired_pos, desired_orientation)

        publisher.publish(pose)

        # wait for hit enter for next gate
        input = raw_input("Press Enter to continue, s to turn drone 180, a for left, d for right...")

        while(input != ''):
            if(input == 's'):
                quat = tf.transformations.quaternion_from_euler(0,0,-yaw)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, w to turn drone forward, a for left, d for right...")
            elif(input == 'a'):
                quat = tf.transformations.quaternion_from_euler(0,0,3.14)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, s to turn drone 180, w for forward, d for right...")
            elif(input == 'd'):
                quat = tf.transformations.quaternion_from_euler(0,0,0)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, s to turn drone 180, a for left, d for forward...")
            elif(input == 'w'):
                quat = tf.transformations.quaternion_from_euler(0,0,yaw)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, s to turn drone 180, a for left, w for forward...")


def main():
    signal.signal(signal.SIGINT, signal_handler)

    pub = rospy.Publisher('/new_pos_init', Pose, queue_size=10)
    rospy.init_node('collect_data_node')
    rate = rospy.Rate(10) # 10hz

    gates = setupGates()
    ## example of how gates are formatted ##
    # for g in gates:
    #     vectora = g.a
    #     vectorb = g.b
    #     vectorc = g.c
    #     vectord = g.d
    #     print("A: x {} y {} z {}".format(vectora.x,vectora.y,vectora.z))
    #     print("B: x {} y {} z {}".format(vectorb.x,vectorb.y,vectorb.z))
    #     print("C: x {} y {} z {}".format(vectorc.x,vectorc.y,vectorc.z))
    #     print("D: x {} y {} z {}".format(vectord.x,vectord.y,vectord.z))
    #     print("\n")

    # start tour
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        if connections != 0:
            start_tour(gates, pub)
            time.sleep(7)
            break


    #rand_gate = randint(0, len(gates)-1)
    # spawn in front of random gate
    #spawn(gates[rand_gate], pub)

main()
