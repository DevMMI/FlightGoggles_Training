import rospy
from mav_msgs.msg import RateThrust
from utils import *
class PID:
    def __init__():
        self.propGain_ = [9.0, 9.0, 9.0]
        self.intGain_ = [3.0, 3.0, 3.0]
        self.derGain_ = [0.3, 0.3, 0.3]
        self.intState_ = [0.,0.,0.]
        self.intBound_ = [1000.,1000.,1000.]

        gain_p_roll = rospy.get_param("/uav/flightgoggles_pid/gain_p_roll", 9.0 )
        gain_i_roll = rospy.get_param("/uav/flightgoggles_pid/gain_i_roll", 3.0 )
        gain_d_roll = rospy.get_param("/uav/flightgoggles_pid/gain_d_roll", 0.3 )
        gain_p_pitch = rospy.get_param("/uav/flightgoggles_pid/gain_p_pitch", 9.0 )
        gain_i_pitch = rospy.get_param("/uav/flightgoggles_pid/gain_i_pitch", 3.0 )
        gain_d_pitch = rospy.get_param("/uav/flightgoggles_pid/gain_d_pitch", 0.3 )
        gain_p_yaw = rospy.get_param("/uav/flightgoggles_pid/gain_p_yaw", 9.0 )
        gain_i_yaw = rospy.get_param("/uav/flightgoggles_pid/gain_i_yaw", 3.0 )
        gain_d_yaw = rospy.get_param("/uav/flightgoggles_pid/gain_d_yaw", 0.3 )
        int_bound_roll = rospy.get_param("/uav/flightgoggles_pid/int_bound_roll", 1000.0 )
        int_bound_pitch = rospy.get_param("/uav/flightgoggles_pid/int_bound_pitch", 1000.0 )
        int_bound_yaw = rospy.get_param("/uav/flightgoggles_pid/int_bound_yaw", 1000.0 )

    def control_update(command, curval, curder, out, dt):
        ''' command: Vector3, curval: double*, curder: double*, out: double*, dt: double '''
        stateDev = [command.x - curval[0], command.y - curval[1], command.z-curval[2]]

        for i in range(3):
            self.intState_[i] += dt * stateDev[i]
            self.intState_ = min(max(-self.intBound_[i], self.intState_[i]),  self.intBound_[i])
            out[i] = self.propGain_ * stateDev[i] + self.intGain_[i] * intState_[i] + derGain_[i] * -curder[i]

    def resetState():
        for i in range(3):
            self.intState_[i] = 0.

def control_callback(control_msg):





def main():
    rospy.init_node('pid_controller')
    rate = rospy.Rate(10)

    control_sub = rospy.Subscriber('/uav/input/rateThrust', RateThrust, control_callback)
