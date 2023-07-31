#!/usr/bin/env python3
import rospy
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import actionlib

class MoveToPose(object):
    def __init__(self):
        self.base_link = 'base_link'
        self.ee_link = 'tool0'
        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.tree.getNrOfJoints()
        self.pos_ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.arm_joints = kdl.JntArray(self.num_joints)
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        rospy.init_node('move_to_pose')
        rospy.Subscriber('/joint_states', JointState, self.arm_joint_state_cb)
        self.arm_pos_cli = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_pos_cli.wait_for_server()

    def arm_joint_state_cb(self, msg):
        for i in range(self.num_joints):
            self.arm_joints[i] = msg.position[i]

    def ik(self, pos, rot):
        eef_pose = kdl.Frame(kdl.Rotation.RPY(*rot), kdl.Vector(*pos))
        q_sol = kdl.JntArray(self.num_joints)
        result = self.pos_ik_solver.CartToJnt(self.arm_joints, eef_pose, q_sol)
        rospy.logdebug('ik solver result: {}'.format(result))
        return list(q_sol)

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

if __name__ == "__main__":
    mover = MoveToPose()

    pos = [0.4, 0.0, 0.3]  # x, y, z
    rot = [0.0, 0.0, 0.0]  # roll, pitch, yaw
    q_sol = mover.ik(pos, rot)
    mover.send_arm_traj(q_sol)
