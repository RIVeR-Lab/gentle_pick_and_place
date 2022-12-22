#!/usr/bin/env python3
import scipy
import rospy
import actionlib
import numpy as np
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from point_cloud_proc.srv import *
from ur_msgs.srv import *
from scipy.spatial.transform import Rotation
import scipy.linalg as linalg
import copy

# BottlePickPlace represents major functionality of the robot as a Python-style class.
class BottlePickPlace(object):
    def __init__(self):

        # set up IK/FK solvers
        self.base_link = 'base_link'
        self.ee_link = 'tool0'
        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.tree.getNrOfJoints()
        self.pos_ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.pos_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        # set the internal joint attributes
        self.arm_joints = kdl.JntArrayVel(self.num_joints)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # init rospy nodes, setting up subscribers, publishers, clients and services as necessary
        rospy.init_node('ur3e')
        rospy.Subscriber('/joint_states', JointState, self.arm_joint_state_cb)
        self.speed_scaling_pub = rospy.Publisher(
            '/speed_scaling_factor', Float64, queue_size=10)
        self.arm_pos_cli = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.object_cluster_cli = rospy.ServiceProxy('/pick_and_place/cluster_objects', TabletopClustering)
        self.io_cli = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        self.arm_pos_cli.wait_for_server()

    def arm_joint_state_cb(self, msg):
        """
        save the current joint state data. called whenever the '/joint_states' topic is updated.'
        args:
            msg (sensor_msgs.msg.JointState): the current state of the robot
        """
        # save position, swapping the order of the first and third joints in the input
        self.arm_joints.q[0] = msg.position[2]
        self.arm_joints.q[1] = msg.position[1]
        self.arm_joints.q[2] = msg.position[0]
        self.arm_joints.q[3] = msg.position[3]
        self.arm_joints.q[4] = msg.position[4]
        self.arm_joints.q[5] = msg.position[5]

        # save velocity, swapping the order of the first and third joints in the input
        self.arm_joints.qdot[0] = msg.velocity[2]
        self.arm_joints.qdot[1] = msg.velocity[1]
        self.arm_joints.qdot[2] = msg.velocity[0]
        self.arm_joints.qdot[3] = msg.velocity[3]
        self.arm_joints.qdot[4] = msg.velocity[4]
        self.arm_joints.qdot[5] = msg.velocity[5]

        # print statements for getting a static history of the robot's position over time
        # print(msg)
        # print("sequence:", msg.header.seq)
        # print("position:", msg.position)
        # print("velocity:", msg.velocity)
        # print("effort:", msg.effort)
        # print()

    def ik(self, pos, rot):
        """
        Perfrom the inverse kinematics transformation for the end effector/grabber.
        Args:
            pos (3-float list): XYZ coordinates of the desired end effector position
            rot (3-float list): Roll, Pitch, Yaw orientation
        Return:
            q_sol (6-float list): degrees to which all joints should be rotated to reach the pos and rot passed in
        """
        eef_pose = kdl.Frame(
            kdl.Rotation.RPY(*rot),
            kdl.Vector(*pos)
        )
        q_init = self.arm_joints.q
        q_sol = kdl.JntArray(self.num_joints)
        result = self.pos_ik_solver.CartToJnt(q_init, eef_pose, q_sol)
        rospy.logdebug('ik solver result: {}'.format(result))
        return list(q_sol)

    def ik_kdl(self, pose):
        # performs the same work as self.ik, 
        # except the pos and rot args are replaced with a kdl.Frame named pose
        q_init = self.arm_joints.q
        q_sol = kdl.JntArray(self.num_joints)
        result = self.pos_ik_solver.CartToJnt(q_init, pose, q_sol)
        rospy.logdebug('ik solver result: {}'.format(result))
        return list(q_sol)

    def fk(self):
        # returns the current position of the robot end effector 
        # helper funciton for the pick() and place() methods below
        eef_pose = kdl.Frame()
        self.pos_fk_solver.JntToCart(self.arm_joints.q, eef_pose)        
        return eef_pose

    def close_gripper(self):
        # close the gripper on the end affector, picking whatever the gripper is above
        cmd = SetIORequest()
        cmd.fun = True
        cmd.state = False
        cmd.pin = 4
        self.io_cli.call(cmd)

    def open_gripper(self):
        # open the gripper on the end affector, releasing any held items
        cmd = SetIORequest()
        cmd.fun = True
        cmd.state = True
        cmd.pin = 4
        self.io_cli.call(cmd)


    def send_arm_traj(self, q):
        """
        command the robot to go to a specific place.
        Works with joint rotations, aka the output of self.ik or self.ik_kdl
        args:
            q (6-float list): A list of joint rotations
        """
        
        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_point = JointTrajectoryPoint()
        traj_point.positions = q
        traj_point.velocities = [0.0] * self.num_joints # tells the arm to stop moving once it's reached the position descibed by q
        traj_point.time_from_start = rospy.Time(1.5) # says this change will occur over 1.5 seconds

        traj.points = [traj_point]
        traj_goal.trajectory = traj
        
        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()

    def send_arm_traj_mpnb(self, q_list):
        # Multi-Point Non-Blended version of send_arm_traj
        # send the arm to a series of positions described as items in the q_list
        # items in q_list are each viable inputs to sned_arm_traj
        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        points_list = []

        for q_x in range(len(q_list)):
            # perform the JointTrajectoryPoint creation for each q in q_list
            q = q_list[q_x]
            traj_point = JointTrajectoryPoint()
            traj_point.positions = q
            traj_point.velocities = [0.0] * self.num_joints
            traj_point.time_from_start = rospy.Time(3*(q_x+1)/len(q_list)) # the robot should reach the final position in q_list after 3 seconds, and take an equal amoutn of time to reach each preceding position
            points_list.append(traj_point)

        traj.points = points_list
        traj_goal.trajectory = traj

        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()

    # *Depreciated*, but recorded as comment for posterity.
    # calculate the SLERP trajectory between two quaterions.
    # since we don't use Quaternions, but rather XYZRPY, this is not helpful.
    # Might be useful later if we switch to using Quaternions.
    #
    # def slerp_calc(self, q_0, q_1, n_points, time):
    #     # calculate a series of quaternion points along the SLERP trajectory
    #     # that exists between quaternions q_0 and q_1. SLERP formula used is
    #     # the Shoemake/Davis formula
    #     dot_product = np.dot(q_0, q_1)
    #     print(dot_product)
    #     theta = np.arccos(dot_product)
    #     stheta = np.sin(theta)
    #     out = [q_0]
    #     for q_index in range(1, n_points):
    #         t_frac = (1/time) * (q_index/n_points)
    #         sttheta = np.sin(t_frac * theta)
    #         somttheta = np.sin((1 - t_frac) * theta)
    #         coef_0 = somttheta / stheta
    #         coef_1 = sttheta / stheta
    #         q_iter = zip(q_0, q_1)
    #         q_sub_point = [(coef_0 * qz) + (coef_1 * qo) for (qz, qo) in q_iter]
    #         out.append(q_sub_point)
    #     out.append(q_1)
    #     return out

    def send_arm_traj_mid(self, q_m, q_f):
        # perform send_arm_traj but use a midpoint (q_m) and an end point (q_f)
        # similar to send_arm_traj_mpnb; interchangable if q_list = [q_m, q_f]
        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        traj_point_m = JointTrajectoryPoint()
        traj_point_m.positions = q_m
        traj_point_m.velocities = [0.0] * self.num_joints
        traj_point_m.time_from_start = rospy.Time(1.5)

        traj_point_f = JointTrajectoryPoint()
        traj_point_f.positions = q_f
        traj_point_f.velocities = [0.0] * self.num_joints
        traj_point_f.time_from_start = rospy.Time(3)

        traj.points = [traj_point_m, traj_point_f]
        traj_goal.trajectory = traj

        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()

    def cluster_objects(self):
        # call the object clustering server to see what items are on the table
        req = TabletopClusteringRequest()
        res = self.object_cluster_cli(req)
        objects = res.objects
        return objects

    def select_grasp_pose(self, object):
        # given an item pointcluster, calculate where the end effector must be to pick up that item
        grasp_pos = object.pose.position
        grasp_pos = [
            object.pose.position.x,
            object.pose.position.y,
            object.pose.position.z + 0.2
        ]

        grasp_rot_y = np.array([
            object.pmin.x - object.pmax.x,
            object.pmin.y - object.pmax.y,
            0
        ])
        grasp_rot_y = grasp_rot_y / linalg.norm(grasp_rot_y)
        grasp_rot_z = np.array([0.0, 0.0, -1.0])
        grasp_pos_x = np.cross(grasp_rot_y, grasp_rot_z)
        grasp_rot = np.concatenate((
            grasp_pos_x.reshape(3, 1),
            grasp_rot_y.reshape(3, 1),
            grasp_rot_z.reshape(3, 1)
            ), axis=1)
        grasp_rot = Rotation.from_matrix(grasp_rot)
        rotation_offset = Rotation.from_euler(seq='xyz', angles=np.array([0, 0, -np.pi/4]))
        grasp_rot = grasp_rot * rotation_offset
        grasp_rot = grasp_rot.as_euler(seq='xyz')

        return grasp_pos, grasp_rot

    def pick(self):
        # reach downward slightly and close gripper, then return to inital position
        current_pose = self.fk()
        pick_pose = copy.deepcopy(current_pose)
        pick_pose.p[2] -= 0.05
        q = self.ik_kdl(pick_pose)
        self.send_arm_traj(q)
        self.close_gripper()
        rospy.sleep(0.5)
        q = self.ik_kdl(current_pose)
        self.send_arm_traj(q)

    def place(self):
        # reach downward slightly and release gripper, then return to inital position
        current_pose = self.fk()
        pick_pose = copy.deepcopy(current_pose)
        pick_pose.p[2] -= 0.1
        q = self.ik_kdl(pick_pose)
        self.send_arm_traj(q)
        self.open_gripper()
        rospy.sleep(0.5)
        q = self.ik_kdl(current_pose)
        self.send_arm_traj(q)


# below code only runs if this file is executed directly. 
# (if __name__ == "__main__" is a common python check to see if a 
# file is running as an import or from executing as the 'main' file.)
if __name__ == "__main__":
    # load some basic positions, then init a BottlePickPlace robot object.
    home_joint_state = [5.0, -1.80, -0.80, -2.0, 1.57, 0.1]
    home_pos = [0.0, 0.4, 0.3]
    # # home_rot = [np.pi, 0, 0]
    pre_place_pose = [0.4, 0.0, 0.3]
    bottle_place_pose = [0.15, 0.45, 0.3]
    can_place_pose = [0.15, -0.45, 0.3]
    demo = BottlePickPlace()

    demo.open_gripper()
    for i in range(5): # pick up up to five items
        q_sol = demo.ik(home_pos, [np.pi, 0, 0])
        print("q_sol at line 253 equals", q_sol)
        demo.send_arm_traj(home_joint_state)

        # get a random item from the object clustering server.
        # The error that ends this program early when there are no more
        # objects to pick up probably originates here?
        objects = demo.cluster_objects()
        rand_object_idx = np.random.choice(len(objects))
        object = objects[rand_object_idx]
        max_segment_norm = linalg.norm(np.array([
                object.pmax.x - object.pmin.x,
                object.pmax.y - object.pmin.y]))

        # calculate the position and rotation necessary to pick up the selected item.
        grasp_pos, grasp_rot = demo.select_grasp_pose(object)
        print("grasp pose =", grasp_pos, "\ngrasp rotation =", grasp_rot)
        # old call to send_arm_traj
        # q_sol = demo.ik(grasp_pos, grasp_rot)
        # demo.send_arm_traj(q_sol)

        # new call to send_arm_traj_mpnb, using a single XYZ midpoint between the current position and the XYZ of the grasping position
        midpoint_xyz = [0,0,0]
        for lcv in range(3):
            midpoint_xyz[lcv] = (grasp_pos[lcv] + home_pos[lcv]) / 2
        q_sols = [demo.ik(midpoint_xyz, grasp_rot),
                demo.ik(grasp_pos, grasp_rot)]
        demo.send_arm_traj_mpnb(q_sols)

        # alternate call to send_arm_traj_mpnb, using 1/4, 1/2, and 3/4 joint midpoints
        # q_stmp = demo.ik(grasp_pos, grasp_rot)
        # midpoint_q = [0,0,0,0,0,0]
        # fst_qrt_q = [0,0,0,0,0,0]
        # trd_qrt_q = [0,0,0,0,0,0]
        # for lcv in range(6):
        #     midpoint_q[lcv] = (q_stmp[lcv] + home_joint_state[lcv]) / 2
        #     fst_qrt_q[lcv] = (midpoint_q[lcv] + home_joint_state[lcv]) / 2
        #     trd_qrt_q[lcv] = (q_stmp[lcv] + midpoint_q[lcv]) / 2
        # q_sols = [fst_qrt_q, midpoint_q, trd_qrt_q, q_stmp]
        # demo.send_arm_traj_mpnb(q_sols)

        demo.pick()

        # q_sol = demo.ik(pre_place_pose, grasp_rot)
        # demo.send_arm_traj(q_sol)

        # below if statement is where the sorting logic happens. If the max segment norm of the item is less than 0.15, then the object is to be a can; more than 0.15, and it's considered a bottle.
        if max_segment_norm < 0.15:
            place_pose = can_place_pose
        else:
            place_pose = bottle_place_pose
        # q_sol = demo.ik(place_pose, grasp_rot)
        # demo.send_arm_traj(q_sol)

        # go to the pre-place pose, then the place pose, before dropping the item
        q_sols = [demo.ik(pre_place_pose, grasp_rot),
                demo.ik(place_pose, grasp_rot)]
        demo.send_arm_traj_mpnb(q_sols)
        demo.place()
