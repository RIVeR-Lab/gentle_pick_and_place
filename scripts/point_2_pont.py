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


    def clip_joint_limits(self, q):
        # Define joint limits for each joint (in radians)
        joint_limits = [
            (2.35, 4.7),  # shoulder_pan_joint
            (0, 0),  # shoulder_lift_joint
            (-0.79, 0),  # elbow_joint    
            (-2.09, 1.05),  # wrist_1_joint
            (0.523, 2.618),  # wrist_2_joint
            (-0, 0)   # wrist_3_joint
            ]

        # Clip the joint positions to the valid range
        q_clip = [np.clip(q[i], joint_limits[i][0], joint_limits[i][1]) for i in range(self.num_joints)]
        return q_clip

    def send_arm_traj(self, q):
        # Clip the joint positions to the valid range
        q_clip = self.clip_joint_limits(q)

        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_point = JointTrajectoryPoint()
        traj_point.positions = q_clip
        traj_point.velocities = [0.0] * self.num_joints
        traj_point.time_from_start = rospy.Time(5.0)

        traj.points = [traj_point]
        traj_goal.trajectory = traj
            
        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()


    def arm_joint_state_cb(self, msg):
        for i in range(self.num_joints):
            self.arm_joints[i] = msg.position[i]

    def jntarray_to_list(self, q):
        q_list = [q[i] for i in range(self.num_joints)]
        return q_list

    # def send_arm_traj(self, q):
    #     q_list = self.jntarray_to_list(q)

    #     traj_goal = FollowJointTrajectoryGoal()
    #     traj = JointTrajectory()
    #     traj.joint_names = self.joint_names
    #     traj_point = JointTrajectoryPoint()
    #     traj_point.positions = q_list  # use list version
    #     traj_point.velocities = [0.0] * self.num_joints
    #     traj_point.time_from_start = rospy.Time(5.0)

    #     traj.points = [traj_point]
    #     traj_goal.trajectory = traj

    #     self.arm_pos_cli.send_goal(traj_goal)
    #     self.arm_pos_cli.wait_for_result()

    def xyz_to_jnt(self, x, y, z):
        # Define workspace limits
        min_x, max_x = -0.5, 0.5  # Set your desired limits for X
        min_y, max_y = -0.45, 0.45  # Set your desired limits for Y
        min_z, max_z = 0.0, 0.5   # Set your desired limits for Z

        # Check if the desired position is within the workspace limits
        if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
            rospy.logwarn("Desired position is out of the robot's workspace.")
            return None

        desired_pose = kdl.Frame(kdl.Vector(x, y, z))

        # Use IK solver to compute joint angles
        q_init = kdl.JntArray(self.num_joints)  # Initial guess
        q_out = kdl.JntArray(self.num_joints)  # Output joint array
        result = self.pos_ik_solver.CartToJnt(q_init, desired_pose, q_out)
        if result < 0:
            rospy.logwarn("IK Solution not found")
            return None

        return q_out

    def send_arm_traj_xyz(self, x, y, z):
        q = self.xyz_to_jnt(x, y, z)
        if q is not None:
            self.send_arm_traj(q)



if __name__ == "__main__":
    home_joint_angles = [5.0, -1.80, -0.80, -2.0, 1.57, 0.1] 
    way_point1_xyz = [0.2, 0.45, 0.0]  
    # way_point2_xyz = [0.2, 0.3, 0.4]  
    # way_point3_xyz = [0.3, 0.4, 0.5]  

    robot_arm_motion = RobotArmMotion()

    robot_arm_motion.send_arm_traj(home_joint_angles)  
    robot_arm_motion.send_arm_traj_xyz(*way_point1_xyz)
    # robot_arm_motion.send_arm_traj_xyz(*way_point2_xyz)
    # robot_arm_motion.send_arm_traj_xyz(*way_point3_xyz)
    robot_arm_motion.send_arm_traj(home_joint_angles)  


    # # Defining the Home Position
    # home_joint_state = kdl.JntArray(6)
    # for i, val in enumerate([5.0, -1.80, -0.80, -2.0, 1.57, 0.1]):
    #     home_joint_state[i] = val

    # # Defining the first way point
    # way_point1 = kdl.JntArray(6)
    # for i, val in enumerate([2.2, -2.39, -0.44, -1.902, 1.69, 0.26]):  # also make sure this has the correct number of joints
    #     way_point1[i] = val

    # # Defining the second way point
    # way_point2 = kdl.JntArray(6)
    # for i, val in enumerate([3.23, -2.39, -0.46, -1.79, 1.64, 0.26]):  # also make sure this has the correct number of joints
    #     way_point2[i] = val

    # # Defining the third way point
    # way_point3 = kdl.JntArray(6)
    # for i, val in enumerate([4.0, -2.41, -0.45, -1.75, 1.64, 0.26]):  # also make sure this has the correct number of joints
    #     way_point3[i] = val

    # robot_arm_motion = RobotArmMotion()

    # # Send the robot to home position
    # robot_arm_motion.send_arm_traj(home_joint_state)

    # # Send the robot to the first way point
    # robot_arm_motion.send_arm_traj(way_point1)

    # # Send the robot to the second way point
    # robot_arm_motion.send_arm_traj(way_point2)

    # # Send the robot to the third way point
    # robot_arm_motion.send_arm_traj(way_point3)

    # # Send the robot back the home position
    # robot_arm_motion.send_arm_traj(home_joint_state)
