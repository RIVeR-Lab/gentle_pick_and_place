#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser 
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import tf2_ros
import tf2_kdl

class RobotArmMotion(object):
    def __init__(self):
        self.base_link = 'base_link'
        self.ee_link = 'tool0'
        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.tree.getNrOfJoints()
        self.pos_ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.pos_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        self.arm_joints = kdl.JntArray(self.num_joints)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        rospy.init_node('ur3e')
        rospy.Subscriber('/joint_states', JointState, self.arm_joint_state_cb)
        self.arm_pos_cli = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.arm_pos_cli.wait_for_server()

    def arm_joint_state_cb(self, msg):
        for i in range(self.num_joints):
            self.arm_joints[i] = msg.position[i]

    def jntarray_to_list(self, q):
        q_list = [q[i] for i in range(self.num_joints)]
        return q_list

    def send_arm_traj(self, q):
        q_list = self.jntarray_to_list(q)

        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_point = JointTrajectoryPoint()
        traj_point.positions = q_list  # use list version
        traj_point.velocities = [0.0] * self.num_joints
        traj_point.time_from_start = rospy.Time(1.5)

        traj.points = [traj_point]
        traj_goal.trajectory = traj

        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()


if __name__ == "__main__":
    home_joint_state = [5.0, -1.80, -0.80, -2.0, 1.57, 0.1]
    # Define the arbitrary joint angles
    arbitrary_joint_state1 = [1.2, -0.8, -1.0, -1.5, 1.5, 0.2]
    arbitrary_joint_state2 = [3.25, -2.189, -0.825, -1.607, 1.491, 2.522]
    arbitrary_joint_state3 = [2.31, -2.036, -0.477, -1.712, 1.55, 2.58]
    arbitrary_joint_state4 = [3.098, -2.622, -0.477, -1.538, 1.511, 2.755]
    arbitrary_joint_state5 = [3.44, -2.622, -0.477, -1.538, 1.511, 2.755]
    robot_arm_motion = RobotArmMotion()

    # Send the robot to home position
    robot_arm_motion.send_arm_traj(home_joint_state)

    # Send the robot to the arbitrary position
    robot_arm_motion.send_arm_traj(arbitrary_joint_state1)
    robot_arm_motion.send_arm_traj(arbitrary_joint_state2)
    robot_arm_motion.send_arm_traj(arbitrary_joint_state3)
    robot_arm_motion.send_arm_traj(arbitrary_joint_state4)
    robot_arm_motion.send_arm_traj(arbitrary_joint_state5)

    # Send the robot back to home position
    robot_arm_motion.send_arm_traj(home_joint_state)
