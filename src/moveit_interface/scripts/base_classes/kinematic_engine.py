import numpy as np
import PyKDL as kdl

from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF

class KinematicsCalculator:
    """!
    An interface that computes forward kinematics via KDL given joint state. 
    The advantage over standard /compute_fk is lower overheads and
    speed.
    """
    def __init__(self, link_tip, key) -> None:
        self._robot_urdf = URDF.from_parameter_server(key=key)
        _, self._kdl_tree = treeFromUrdfModel(self._robot_urdf)
        self._base_link = self._robot_urdf.get_root()
        self._tip_link = link_tip
        self._tip_frame = kdl.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)

        # KDL Solver
        self._fk_p_kdl = kdl.ChainFkSolverPos_recursive(self._arm_chain)
        self._ik_p_kdl = kdl.ChainIkSolverPos_LMA(self._arm_chain)

    def forward_position_kinematics(self,joint_values=None, get_rpy = False):
        """
        Computes FK position given joints list or np array.
        """
        self._fk_p_kdl.JntToCart(self.joint_to_kdl_jnt_array(joint_values),
                                self._tip_frame)
        pos = self._tip_frame.p
        if get_rpy:
            rot = kdl.Rotation(self._tip_frame.M)
            return np.array([pos[0], pos[1], pos[2]]), rot.GetEulerZYX()
        else:
            return np.array([pos[0], pos[1], pos[2]])

    def joint_to_kdl_jnt_array(self, q):
        """
        Given a NP arrary or a list of joint states, converts them to a 
        kdl_joint array instance

        @param q List of joints
        """
        if isinstance(q, np.ndarray) and q.ndim == 1:
            q_kdl = kdl.JntArray(q.size)
            for i in range(q.size):
                q_kdl[i] = q[i]

        elif isinstance(q, list):
            q_kdl = kdl.JntArray(len(q))
            for i, q_i in enumerate(q):
                q_kdl[i] = q_i

        else:
            raise ValueError("Joint Vector q must be either a np.ndarray or list but is type {0}.".format(type(q)))

        return q_kdl

    def inverse_kinematics(self, position, orientation=None, seed=None):
        """!
        Given a seed, gets you the IK.
        Either position only or position and orientation.
        """
        # ik = kdl.ChainIkSolverVel_pinv(self._arm_chain)
        pos = kdl.Vector(position[0], position[1], position[2])
        #rot = kdl.Rotation()
        #rot = rot.Quaternion(orientation[0], orientation[1],
        #                        orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = kdl.JntArray(6)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        #if orientation.any():
        #    goal_pose = kdl.Frame(rot, pos)
        #else:
        goal_pose = kdl.Frame(pos)
        result_angles = kdl.JntArray(6)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            return result
        else:
            return None
