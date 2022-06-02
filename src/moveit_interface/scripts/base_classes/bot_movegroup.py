#!/usr/bin/env python3

# Author: Stephen Tellis

# Built in
import sys, time

# External Libs

# import ROS libraries
import rospy
from rospy import Header
import moveit_commander

# Custom classes 

# import ROS message types
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFKRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

# import Custom message types


rospy.wait_for_service("/move_group/trajectory_execution/set_parameters")

try:
    # Tunable variables
    VEL_SCALE = rospy.get_param("/vel_scale")
    ACC_SCALE = rospy.get_param("/acc_scale")
    POS_CHECK_TOL = rospy.get_param("/move_group/trajectory_execution/allowed_start_tolerance")
    GOAL_JOINT_TOLERANCE = rospy.get_param("/goal_joint_tol")
    EE_LINK = rospy.get_param("/tip_link")
    PRIMARY_PLANNER = rospy.get_param("/primary_planner")
except: 
    rospy.logerr(f"Parameter server has not loaded required parameters {sys.argv[0]}")
    sys.exit(1)

class BotMovegroup:

    def __init__(self):
        # initialize moveit_commander - only after this, can we use the robot, scene and movegroup instances
        moveit_commander.roscpp_initialize(sys.argv)

    def interface_primary_movegroup(self):
        # Initialize move group within seperate namespaces
        self.robot , self.planning_scene, self.move_group = self._initialize_planner(planner_name=PRIMARY_PLANNER)

    def _initialize_planner(self, planner_name, namespace=""):

        # instantiate the robot commander we need this to get data about the current robot state
        robot = moveit_commander.RobotCommander()

        # instantiate the scene. Needed to get, set and update the robots surrounding of the outside world
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate the move group object - this is the one that takes care of planning and execution
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=namespace)

        # Instantiate a Display trajectory ROS publisher (for RVIZ visualization) - optional since planning does it already
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    DisplayTrajectory,
                                                    queue_size=20)

        # set planner to be used
        move_group.set_planner_id(planner_name)

        # Initialize planning parameters
        move_group.allow_replanning(True)
        move_group.set_goal_joint_tolerance(GOAL_JOINT_TOLERANCE)
        move_group.set_max_velocity_scaling_factor(VEL_SCALE)
        move_group.set_max_acceleration_scaling_factor(ACC_SCALE)

        return robot, scene, move_group

    def initialize_utils(self):

        # Service servers

        # Service clients
        self._switch_controller_client = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)

        # Publishers
        self._publish_joy_servo_cmd = rospy.Publisher("/spacenav/joy", Joy, queue_size=1)

    def move_synchronous(self, the_plan, target_pose):
        """!
        Executes the moveit plan synchronously no abort/safety/preempt checks.
        """
        exec = self.move_group.execute(the_plan, wait=True)
        if exec == True:
            # self.last_pose = list(target_pose).copy()
            rospy.loginfo("Bewegung erfolgreich")
            return "succeeded"
        else:
            rospy.loginfo("Bewegung fehlgeschlagen")
            return "unsuccessful_exec"  

    def switch_controller(self, mode):
        req = SwitchControllerRequest()
        req.strictness = 2
        req.timeout = 1 # 1 second(s) to timeout
        if mode == "servo":
            req.start_controllers = ["joint_group_pos_controller"]
            req.stop_controllers = ["pos_joint_traj_controller"]
            response = self._switch_controller_client(req)
            return response.ok
        elif mode == "traj":
            req.start_controllers = ["pos_joint_traj_controller"] 
            req.stop_controllers =["joint_group_pos_controller"]
            response = self._switch_controller_client(req)
            return response.ok
        else:
            return False

    @classmethod
    def _populate_header_msg(self, frame_id = "base_link"):
        """!
        Utility func to easily populate the header for us
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return header

    @classmethod
    def populate_joy_msg(cls, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
        # data = loads(data)
        msg = Joy()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        a = []
        a.append(lin_x)
        a.append(lin_y)
        a.append(lin_z)
        a.append(ang_x)
        a.append(ang_y)
        a.append(ang_z)
        msg.axes = a
        return msg
        
    def get_plan(self, target_pose, secondary=False):
        """
        Check if start pose matches the last pose within tol, if not, replanning will be done
        """
        time.sleep(0.15)
        the_plan = tuple()

        # sets the start state and goal state
        self._set_start_state_to_current_joint_state_topic()
        self.move_group.set_joint_value_target(target_pose)

        self.last_pose = list(self.move_group.get_current_state().joint_state.position)
        the_plan = the_plan = self.move_group.plan()

        return the_plan

    def _set_start_state_to_current_joint_state_topic(self):
        joint_msg = rospy.wait_for_message("/joint_states", JointState)
        current_joints_state_list = self._joint_state_updater(joint_msg)
        moveit_robot_state = self._populate_robotstate_message_from_list(current_joints_state_list)
        self.move_group.set_start_state(moveit_robot_state)

    def _joint_state_updater(self, sensr_msg):
        # Flip Joint values to the right format
        flipped = list(sensr_msg.position)
        # Since the joint state topic publishes with joints 1 and 3 flipped
        current_joint_state = [flipped[2], flipped[1], flipped[0], flipped[3], flipped[4], flipped[5]] 
        return current_joint_state

    def _set_start_state_to_given_joint_list(self, joint_list):
        moveit_robot_state = self._populate_robotstate_message_from_list(joint_list)
        self.move_group.set_start_state(moveit_robot_state)

    def _populate_robotstate_message_from_list(self, joint_list):
        joint_state = JointState()
        joint_state.header = BotMovegroup._populate_header_msg()
        joint_state.name = self.robot.get_active_joint_names(group='manipulator')
        joint_state.position = joint_list
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        return moveit_robot_state

    def populate_fk_srv_req(self, joint_list):
        """
        Utility function to convert joint list to moveit_msgs/GetPoitionFK.srv format
        """
        fk_req_msg = GetPositionFKRequest()
        fk_req_msg.header = BotMovegroup._populate_header_msg()
        fk_req_msg.fk_link_names = self.robot.get_link_names(group='manipulator')
        fk_req_msg.robot_state = self._populate_robotstate_message_from_list(joint_list)
        # rospy.loginfo(f"The populated fk request is {fk_req_msg}")
        return fk_req_msg

    def _plan_to_target(self, start, target):
        """!
        This function handles the moveit plan generation for the selected robust planner.

        @return list moveit plan
        """
        the_plan = self.move_group_secondary.plan() # This also takes care of RVIZ visualization
        return the_plan

    def get_current_pose(self):
        """
        Returns the current pose as a list of 7 floats [x, y, z, qx, qy, qz, qw]
        """
        return self.move_group.get_current_pose(end_effector_link = EE_LINK)

