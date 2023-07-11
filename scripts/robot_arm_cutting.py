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

class SinusoidalMotion(object):
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

    def send_arm_traj(self, q):
        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_point = JointTrajectoryPoint()
        traj_point.positions = q
        traj_point.velocities = [0.0] * self.num_joints
        traj_point.time_from_start = rospy.Time(1.5)

        traj.points = [traj_point]
        traj_goal.trajectory = traj
        
        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()

    def execute_sinusoidal_motion(self, amplitude, frequency):
        # Starting the sinusoidal motion
        duration = 2 * np.pi / frequency
        t = np.arange(0.0, duration, 0.01)
        positions = amplitude * np.sin(2 * np.pi * frequency * t)

        start_pos = kdl.JntArray(self.num_joints)
        self.pos_fk_solver.JntToCart(start_pos, start_pos)

        for i, pos in enumerate(positions):
            desired_pos = start_pos
            desired_pos.p[0] += pos  # Adding sinusoidal motion in X axis
            desired_pos.p[1] += i * 0.01  # Linear motion in Y axis

            q = kdl.JntArray(self.num_joints)
            self.pos_ik_solver.CartToJnt(self.arm_joints, desired_pos, q)
            self.send_arm_traj(q)

if __name__ == "__main__":
    home_joint_state = [5.0, -1.80, -0.80, -2.0, 1.57, 0.1]
    sinusoidal_motion = SinusoidalMotion()

    # User input for amplitude and frequency
    amplitude = float(input("Enter the amplitude (0.01-0.1): "))
    frequency = float(input("Enter the frequency (0.0001-0.001 Hz): "))

    # Check if user input is within acceptable range
    if not (0.01 <= amplitude <= 0.1):
        print("Invalid amplitude! Please enter a value between 0.01 and 0.1.")
        exit(1)
    if not (0.0001 <= frequency <= 0.001):
        print("Invalid frequency! Please enter a value between 0.0001 and 0.001 Hz.")
        exit(1)

    # Send the robot to home position
    sinusoidal_motion.send_arm_traj(home_joint_state)

    # Execute sinusoidal motion with user defined amplitude and frequency
    sinusoidal_motion.execute_sinusoidal_motion(amplitude, frequency)

    # Send the robot back to home position
    sinusoidal_motion.send_arm_traj(home_joint_state)
