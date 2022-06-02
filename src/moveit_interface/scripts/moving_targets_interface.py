#!/usr/bin/env python3

# Author: Stephen Tellis

import sys

# External libs
import numpy as np
from simple_pid import PID

# import ROS libraries
import rospy
import tf

# Import pre-defined msg/srv/actions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# import custom message types

# Custom classes 
from base_classes.bot_movegroup import BotMovegroup
from base_classes.kinematic_engine import KinematicsCalculator

try:
    LINK_TIP = rospy.get_param("/tip_link")
except: 
    rospy.logerr(f"Parameter server has not loaded required parameters {sys.argv[0]}")
    sys.exit(1)

JOINT_SEED = [-0.021734192155393295, -0.9496069503865936, 1.7689707419512306, -0.8313125172011855, 1.5145384981892445, 0]
INSP_POINTS = ["point1", "point2"]
THRESHOLD = 0.015

class MoveBot(BotMovegroup):

    def __init__(self):
        super().__init__()

        rospy.init_node('moving_targets_interface', anonymous=True)

        # Controller initialization (needs more tuning)
        self.pidx = PID(50, 0.15, 0.5, setpoint=0) 
        self.pidy = PID(50, 0.15, 0.5, setpoint=0)
        self.pidz = PID(50, 0.15, 0.5, setpoint=0)

        self.pidy.output_limits = (-1, 1)
        self.pidz.output_limits = (-1, 1)
        self.pidx.output_limits = (-1, 1)

        self.stop_prog = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        self.play_prog = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)

        # Instantiate FK calculator
        self.get_fk = KinematicsCalculator(LINK_TIP, 'robot_description')

    def _move_to_given_joints(self, joints):
        """!
        Interfaces with moveit to move robot
        """
        the_plan = self.get_plan(joints)
        self.move_synchronous(the_plan[1], joints)
        return True

def _joint_state_formatter(sensr_msg):
    """!
    Flip Joint values to the right format
    """
    shuffled = list(sensr_msg.position)
    current_joint_state = [shuffled[2], shuffled[1], shuffled[0], shuffled[3], shuffled[4], shuffled[5]] 
    return current_joint_state

def main():
    mcc = MoveBot() 
    mcc.interface_primary_movegroup()
    mcc.initialize_utils()
    rospy.loginfo("moveit commander is ready")
    rospy.sleep(2)
    mcc.switch_controller(mode ="servo")
    rate = rospy.Rate(20)
    listener = tf.TransformListener()
    i = 0

    # Keeps the control continuing between the given points
    while not rospy.is_shutdown():

        ## Macro Positioning

        # Chose a point to visit
        target = INSP_POINTS[i%len(INSP_POINTS)]
        i +=1

        # TFs and IK
        try:
            (trans,_) = listener.lookupTransform('base_link', target, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        target_joints = list(mcc.get_fk.inverse_kinematics(trans, seed=JOINT_SEED))
        if not isinstance(target_joints, list):
            print("Target joints could not be found. Going to next loop")
            continue

        # Execute traj
        resp = mcc.switch_controller(mode = "traj")
        print(f"Switch to traj was {resp}")
        mcc._move_to_given_joints(target_joints)
        rospy.sleep(0.1)

        ## Precise control

        # Execute servo
        resp = mcc.switch_controller(mode ="servo")
        print(f"Switch to servo was {resp}")
        error = np.inf

        # An ugly (but working) position only control loop - TODO clean this up 
        while abs(error) >= THRESHOLD:

            joint_values = rospy.wait_for_message("/joint_states", JointState)
            pos_current, _ = mcc.get_fk.forward_position_kinematics(_joint_state_formatter(joint_values), get_rpy=True) 

            try:
                (trans,_) = listener.lookupTransform('base_link', target, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            trans_arr = np.asarray(trans)
            pos_curr_arr = np.asarray(pos_current)
            deviation = pos_curr_arr - trans_arr
            x_control = mcc.pidx(deviation[0])
            y_control = mcc.pidy(deviation[1])
            z_control = mcc.pidz(deviation[2])
            error = np.linalg.norm(deviation)

            # Publish control
            msg_data = mcc.populate_joy_msg(lin_x=x_control, lin_y=y_control, lin_z=z_control)
            mcc._publish_joy_servo_cmd.publish(msg_data)
            
            rate.sleep()

if __name__ == "__main__":
    main()