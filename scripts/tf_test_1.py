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
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('aptg_tf_tranform_calc')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    base_to_cam = tfBuffer.lookup_transform('base_link', 'camera_base', rospy.Time(0), rospy.Duration(1.0))
    cam_to_tag = tfBuffer.lookup_transform('rgb_camera_link', 'bottle_taG', rospy.Time(0), rospy.Duration(1.0))
    #print(base_to_cam.transform)
    #print(cam_to_tag.transform)
    
    tag_to_tag = tfBuffer.lookup_transform('base_link_dup', 'bottle_taG', rospy.Time(0), rospy.Duration(1.0))
    tag_to_tag_2 = tfBuffer.lookup_transform('base_link_dup', 'can_taG', rospy.Time(0), rospy.Duration(1.0))
    #print(tag_to_tag)
    #print(tag_to_tag.transform)
    print(tag_to_tag.transform.translation)
    print(tag_to_tag_2.transform.translation)
    #print(type(tag_to_tag.transform.translation))

    t_out = TransformStamped()
    t_out.header.stamp = rospy.Time.now()
    t_out.header.frame_id = 'base_link'
    t_out.child_frame_id = 'bottle_taG'

    t1 = base_to_cam.transform.translation
    t2 = cam_to_tag.transform.translation
    t_out.transform.translation.x = t1.x + t2.x
    t_out.transform.translation.y = t1.y + t2.y
    t_out.transform.translation.z = t1.z + t2.z
    t_out.transform.rotation.x = 0
    t_out.transform.rotation.y = 0
    t_out.transform.rotation.z = 0
    t_out.transform.rotation.w = 1
    #print(t_out)

    #broadcaster = tf2_ros.StaticTransformBroadcaster()
    #broadcaster.sendTransform(t_out)
    #rospy.spin()
