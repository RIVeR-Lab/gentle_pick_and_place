
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
import tf2_ros

class BottlePickPlace(object):
    def __init__(self):

        self.base_link = 'base_link'
        self.ee_link = 'tool0'
        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.tree.getNrOfJoints()
        self.pos_ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.pos_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

        self.arm_joints = kdl.JntArrayVel(self.num_joints)
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
        self.speed_scaling_pub = rospy.Publisher(
            '/speed_scaling_factor', Float64, queue_size=10)
        # self.speed_scaling_pub.publish(2.0)
        self.arm_pos_cli = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.object_cluster_cli = rospy.ServiceProxy('/pick_and_place/cluster_objects', TabletopClustering)
        self.io_cli = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        self.arm_pos_cli.wait_for_server()

    def arm_joint_state_cb(self, msg):
        self.arm_joints.q[0] = msg.position[2]
        self.arm_joints.q[1] = msg.position[1]
        self.arm_joints.q[2] = msg.position[0]
        self.arm_joints.q[3] = msg.position[3]
        self.arm_joints.q[4] = msg.position[4]
        self.arm_joints.q[5] = msg.position[5]

        self.arm_joints.qdot[0] = msg.velocity[2]
        self.arm_joints.qdot[1] = msg.velocity[1]
        self.arm_joints.qdot[2] = msg.velocity[0]
        self.arm_joints.qdot[3] = msg.velocity[3]
        self.arm_joints.qdot[4] = msg.velocity[4]
        self.arm_joints.qdot[5] = msg.velocity[5]

    def ik(self, pos, rot):
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
        q_init = self.arm_joints.q
        q_sol = kdl.JntArray(self.num_joints)
        result = self.pos_ik_solver.CartToJnt(q_init, pose, q_sol)
        rospy.logdebug('ik solver result: {}'.format(result))
        return list(q_sol)

    def fk(self):
        eef_pose = kdl.Frame()
        self.pos_fk_solver.JntToCart(self.arm_joints.q, eef_pose)        
        return eef_pose

    def close_gripper(self):
        cmd = SetIORequest()
        cmd.fun = True
        cmd.state = False
        cmd.pin = 4
        self.io_cli.call(cmd)

    def open_gripper(self):
        cmd = SetIORequest()
        cmd.fun = True
        cmd.state = True
        cmd.pin = 4
        self.io_cli.call(cmd)


    def send_arm_traj(self, q):
        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_point = JointTrajectoryPoint()
        traj_point.positions = q
        traj_point.velocities = [0.0] * self.num_joints
        traj_point.time_from_start = rospy.Time(2.5)

        traj.points = [traj_point]
        traj_goal.trajectory = traj
        
        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()

    def cluster_objects(self):
        req = TabletopClusteringRequest()
        res = self.object_cluster_cli(req)
        objects = res.objects
        return objects

    def select_grasp_pose(self, object):    
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
        current_pose = self.fk()
        pick_pose = copy.deepcopy(current_pose)
        pick_pose.p[2] -= 0.1
        q = self.ik_kdl(pick_pose)
        self.send_arm_traj(q)
        self.open_gripper()
        rospy.sleep(0.5)
        q = self.ik_kdl(current_pose)
        self.send_arm_traj(q)


if __name__ == "__main__":
    home_joint_state = [5.0, -1.80, -0.80, -2.0, 1.57, 0.1]
    # home_pos = [0.0, 0.4, 0.3]
    # # home_rot = [np.pi, 0, 0]
    pre_place_pose = [0.4, 0.0, 0.3]
    default_bottle_place_pose = [0.15, 0.45, 0.3]
    default_can_place_pose = [0.15, -0.45, 0.3]
    bottle_place_pose = default_bottle_place_pose
    can_place_pose = default_can_place_pose
    demo = BottlePickPlace()

    demo.open_gripper()
    for i in range(5):
        # q_sol = demo.ik(home_pos, [np.pi, 0, 0])
        demo.send_arm_traj(home_joint_state)

        # start a tfBuffer for getting information about the AprilTags relative locations via TF
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try:
            # get the relative loactions of the fake base_link, the tag for the bottle 'bin', and the tag for the can 'bin'
            base_dup_to_bottle = tfBuffer.lookup_transform('base_link_dup', 'bottle_taG', rospy.Time(0), rospy.Duration(1.0))
            base_dup_to_can = tfBuffer.lookup_transform('base_link_dup', 'can_taG', rospy.Time(0), rospy.Duration(1.0))

            # define the shift between tag location and bin location
            # each value is a distance in meters along the relevant positive axis
            tag_to_bin_key = {
                    'x': -0.2,
                    'y': -0.1,
                    'z': 0.3
            }

            # convert the TF messages to pose lists, adding the offset
            ar_bottle_place_pose = [
                    base_dup_to_bottle.transform.translation.x + tag_to_bin_key['x'],
                    base_dup_to_bottle.transform.translation.y + tag_to_bin_key['y'],
                    base_dup_to_bottle.transform.translation.z + tag_to_bin_key['z']
                ]
            ar_can_place_pose = [
                    base_dup_to_can.transform.translation.x + tag_to_bin_key['x'],
                    base_dup_to_can.transform.translation.y + tag_to_bin_key['y'],
                    base_dup_to_can.transform.translation.z + tag_to_bin_key['z']
                ]

            # assign the calculated poses to the reals
            print("AprilTag place poses")
            bottle_place_pose = ar_bottle_place_pose
            can_place_pose = ar_can_place_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # enter the except case if something went wrong with the TF frames specifically
            # (usually this means one or more frame wasn't visable)
            # in this case assign the default poses to the reals
            print("Default place poses")
            bottle_place_pose = default_bottle_place_pose
            can_place_pose = default_can_place_pose

        objects = demo.cluster_objects()
        rand_object_idx = np.random.choice(len(objects))
        object = objects[rand_object_idx]
        max_segment_norm = linalg.norm(np.array([
                object.pmax.x - object.pmin.x,
                object.pmax.y - object.pmin.y]))

        grasp_pos, grasp_rot = demo.select_grasp_pose(object)
        q_sol = demo.ik(grasp_pos, grasp_rot)
        demo.send_arm_traj(q_sol)
        demo.pick()

        q_sol = demo.ik(pre_place_pose, grasp_rot)
        demo.send_arm_traj(q_sol)

        if max_segment_norm < 0.15:
            place_pose = can_place_pose
        else:
            place_pose = bottle_place_pose
        q_sol = demo.ik(place_pose, grasp_rot)
        demo.send_arm_traj(q_sol)
        demo.place()
