import rospy
import time
import tf
import math
import signal
import sys
import os
import yaml
from statistics import mean
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randint
#gate_list_dir = "flightgoggles/flightgoggles/config/challenges/gate_locations.yaml"
gate_list_dir = "flightgoggles/flightgoggles/config/challenges"
gate_results_dir = "gate_results"
name = "results.yaml"
fw = open(os.path.join(gate_results_dir,name), "a")
fp = open(os.path.join(gate_results_dir,"gates_plain.yaml"), "w")
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


def signal_handler(sig, frame):
        print('\n Bye bye!')
        fw.close()
        sys.exit(0)

pairs = {}
gate_storage = []

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
#
# def setupGates(filename):
#     global gate_storage
#     file_str = os.path.join(gate_list_dir, filename)
#     f = open(file_str, "r")
#     for i, line in enumerate(f):
#         # each even line should be a gate
#         if (i+1) % 2 == 0:
#             str_total = line[12:][1:-2]
#             str_total = str_total.replace(' ', '')
#             str_arrays = str_total.split("],[")
#             gate = Gate()
#             for j, arr in enumerate(str_arrays):
#
#                 gate_vals = arr.strip(']').strip('[').split(",")
#                 v = Vector3F(gate_vals[0],gate_vals[1],gate_vals[2])
#
#                 if j == 0:
#                   gate.a = v
#                 elif j == 1:
#                   gate.b = v
#                 elif j == 2:
#                   gate.c = v
#                 elif j == 3:
#                   gate.d = v
#             print("real gate tleftx {}, brightx {}".format(gate.a.x, gate.d.x))
#             already_there = False
#             for gate_ in gate_storage:
#                 if(gate == gate_):
#                     #print("already there")
#                     already_there = True
#                     pairs[len(pairs)+1] = (gate, gate_)
#                     break
#
#             if(not already_there):
#                 gate_storage.append(gate)
#

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

def getGateStr(gate):
    gate_str = "["
    gate_str += str(gate.a.x) + ", "
    gate_str += str(gate.a.y) + ", "
    gate_str += str(gate.a.z) + "], "
    gate_str += "["
    gate_str += str(gate.b.x) + ", "
    gate_str += str(gate.b.y) + ", "
    gate_str += str(gate.b.z) + "], "
    gate_str += "["
    gate_str += str(gate.c.x) + ", "
    gate_str += str(gate.c.y) + ", "
    gate_str += str(gate.c.z) + "], "
    gate_str += "["
    gate_str += str(gate.d.x) + ", "
    gate_str += str(gate.d.y) + ", "
    gate_str += str(gate.d.z) + "]"
    return gate_str

def writeGate(it, gate, orientation):
    global fw
    if(orientation == 'i' or orientation == 'k'):
        '''orientation is north-south '''
        first = "Gate"+str(it)+":\n"
        first += "  direction:" + " N/S" +"\n"
        first += "  location: ["
        first += getGateStr(gate)
        first += "]\n"
        fw.write(first)

    elif(orientation == 'j' or orientation == 'l'):
        '''orientation is east-west '''
        first = "Gate"+str(it)+":\n"
        first += "  direction:" + " E/W" +"\n"
        first += "  location: ["
        first += getGateStr(gate)
        first += "]\n"
        fw.write(first)
    elif(orientation == 'o'):
        '''orientation is up-down'''
        first = "Gate"+str(it)+":\n"
        first += "  direction:" + " U/D" +"\n"
        first += "  location: ["
        first += getGateStr(gate)
        first += "]\n"
        fw.write(first)
    elif(orientation == 'p'):
        '''orientation is other'''
        first = "Gate"+str(it)+":\n"
        first += "  direction:" + " UNK" +"\n"
        first += "  location: ["
        first += getGateStr(gate)
        first += "]\n"
        fw.write(first)


def writeGatesPlain(it, gate):
    global fp
    '''orientation is north-south '''
    first = "Gate"+str(it)+":\n"
    first += "  location: ["
    first += getGateStr(gate)
    first += "]\n"
    fp.write(first)

def start_tour(gates, publisher, ns_orientation):
    num_orientations = 12
    dist_from_gate = 5.0
    print("total num of gates {}".format(len(gates)))
    for i, gate in enumerate(gates):
        print("")
        tleft = gate.a
        tright = gate.b
        bright = gate.c
        bleft = gate.d

        # calculate position from gate
        if(ns_orientation):
            position_z = (float(tleft.z) + float(bleft.z)) / 2.0
            position_x = ((float(tleft.x) + float(tright.x)) / 2.0) + dist_from_gate
            position_y = ((float(tleft.y) + float(tright.y)) / 2.0)
        else:
            position_z = (float(tleft.z) + float(bleft.z)) / 2.0
            position_x = ((float(tleft.x) + float(tright.x)) / 2.0)
            position_y = ((float(tleft.y) + float(tright.y)) / 2.0) - dist_from_gate

        # desired starting orientation
        if(ns_orientation):
            yaw = 3.14
        else:
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
            if(input == 'i' or input == 'k'):
                'N/S'
                writeGate(i, gate, 'i')
                break
            elif(input == 'j' or input == 'l'):
                'E/W'
                writeGate(i, gate, 'j')
                break
            elif(input == 'o'):
                'Up/Down'
                writeGate(i, gate, 'o')
                break
            elif(input == 'p'):
                'other'
                writeGate(i, gate, 'p')
                break

            elif(input == 's'):
                quat = tf.transformations.quaternion_from_euler(0,0,-yaw)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, w to turn drone forward, a for left, d for right...\n\
                Press i or k for north/south, Press j or l for east/west, Press o for up/down, Press p for other...")
            elif(input == 'a'):
                quat = tf.transformations.quaternion_from_euler(0,0,3.14)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, s to turn drone 180, w for forward, d for right...\n\
                Press i or k for north/south, Press j or l for east/west, Press o for up/down, Press p for other...")
            elif(input == 'd'):
                quat = tf.transformations.quaternion_from_euler(0,0,0)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, s to turn drone 180, a for left, d for forward...\n\
                Press i or k for north/south, Press j or l for east/west, Press o for up/down, Press p for other...")
            elif(input == 'w'):
                quat = tf.transformations.quaternion_from_euler(0,0,yaw)
                desired_orientation = Quaternion(quat[0] ,quat[1] ,quat[2] ,quat[3] )
                pose = Pose(desired_pos, desired_orientation)
                publisher.publish(pose)
                input = raw_input("Press Enter to continue, s to turn drone 180, a for left, w for forward...\n\
                Press i or k for north/south, Press j or l for east/west, Press o for up/down, Press p for other...")
            else:
                input = raw_input("Press Enter to continue, s to turn drone 180, a for left, w for forward...\n\
                Press i or k for north/south, Press j or l for east/west, Press o for up/down, Press p for other...")


def main():
    signal.signal(signal.SIGINT, signal_handler)

    pub = rospy.Publisher('/new_pos_init', Pose, queue_size=10)
    rospy.init_node('collect_data_node')
    rate = rospy.Rate(10) # 10hz

    print("Your beginning orientation is E/W or East-West!")
    gates_files = []
    for file in os.listdir(gate_list_dir):
        if(file.startswith("gate_locations")):
            setupGates(file)

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

    ## create gate orientation yaml ##
    up_down_list = []
    for i, g in enumerate(gate_storage):
        if(mean([abs(g.a.z - g.c.z), abs(g.a.z - g.b.z), abs(g.a.z - g.d.z)]) < 0.8 ):
            up_down_list.append(g)
            del gate_storage[i]

    ns_list = []
    ew_list = []
    for i, g in enumerate(gate_storage):
        vectora = g.a
        vectorb = g.b
        vectorc = g.c
        vectord = g.d

        vectors = []
        vectors.append(vectora)
        vectors.append(vectorb)
        vectors.append(vectorc)
        vectors.append(vectord)

        z_thresh = 0.5
        max_distance = -1.0
        max_diff_vec_a = Vector3F()
        max_diff_vec_b = Vector3F()
        # find two diagonally opposed vectors (points on gate)
        for v in vectors:
            for c in vectors:
                if abs(v.z - c.z) > z_thresh:
                    distance = ( (abs(v.x - c.x)**2) + (abs(v.y - c.y)**2)  ) ** (1/2)
                    if(distance > max_distance):
                        max_distance = distance
                        max_diff_vec_a = v
                        max_diff_vec_b = c

        print("corner a: x {} y {} z {}".format(max_diff_vec_a.x,max_diff_vec_a.y,max_diff_vec_a.z))
        print("corner b: x {} y {} z {}".format(max_diff_vec_b.x,max_diff_vec_b.y,max_diff_vec_b.z))
        print("\n")
        if(abs(max_diff_vec_a.x - max_diff_vec_b.x) > abs(max_diff_vec_a.y - max_diff_vec_b.y)):
            'east-west'
            ew_list.append(g)
        else:
            'north-south'
            ns_list.append(g)

    ## duplicate gates list creation ##
    # pairs_list = []
    # for it in pairs:
    #     pair = pairs[it]
    #     pairs_list.append(pair[0])
    #     pairs_list.append(pair[1])

    ## yaml creation test ##
    # for i, gate in enumerate(gate_storage):
    #     if(i == 5):
    #         writeGate(i, gate, 'p')
    #         break
    #     if(i == 4):
    #         writeGate(i, gate, 'o')
    #     elif(i % 2 == 0):
    #         writeGate(i, gate, 'i')
    #     elif(i % 2 == 1):
    #         writeGate(i, gate, 'l')

    # write all non-duplicate gates plainly ##
    for i, gate in enumerate(gate_storage):
        #print("tleft x {}, bright x {}".format(gate.a.x, gate.d.x))
        writeGatesPlain(i, gate)

    print("ew list {}".format(len(ew_list)))
    print("ns list {}".format(len(ns_list)))
    ## main ros loop ##
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        if connections != 0:
            start_tour(ns_list, pub, True)
            time.sleep(7)
            break


main()
fw.close()
