#!/usr/bin/env python3
import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

class RobotArmController:
    def __init__(self):
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.home_positions = [5.0, -1.80, -0.80, -2.0, 1.57, 0.1]
        self.amplitude_range = (-0.1, 0.1)  # Range for the amplitude of the sinusoidal wave
        self.frequency_range = (0.1, 2.0)  # Range for the frequency of the sinusoidal wave
        self.duration = 5.0  # Duration of the movement in seconds

        self.client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def move_to_home_position(self):
        joint_traj = JointTrajectory()
        joint_traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_positions
        point.time_from_start = rospy.Duration(2.0)  # Time to reach the home position
        joint_traj.points.append(point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_traj
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def move_in_sinusoidal_wave(self, amplitude, frequency):
        rate = rospy.Rate(10)  # Control loop rate (10 Hz)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()

            # Calculate the desired joint positions based on a sinusoidal wave
            joint_positions = []
            for i in range(len(self.joint_names)):
                joint_positions.append(
                    amplitude * np.sin(2 * np.pi * frequency * t))

            # Create a joint trajectory message
            joint_traj = JointTrajectory()
            joint_traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = rospy.Duration.from_sec(t)
            joint_traj.points.append(point)

            # Send the joint trajectory to the robot controller
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = joint_traj
            self.client.send_goal(goal)

            # Break the loop if the duration has elapsed
            if t >= self.duration:
                break

            rate.sleep()

        self.move_to_home_position()

    def get_user_input(self):
        amplitude = self.get_validated_user_input("Enter the amplitude (between {} and {}): ".format(self.amplitude_range[0], self.amplitude_range[1]), self.amplitude_range)
        frequency = self.get_validated_user_input("Enter the frequency (between {} and {}): ".format(self.frequency_range[0], self.frequency_range[1]), self.frequency_range)
        return amplitude, frequency

    def get_validated_user_input(self, prompt, value_range):
        while True:
            try:
                value = float(input(prompt))
                if value_range[0] <= value <= value_range[1]:
                    return value
                else:
                    print("Input out of range. Please enter a value between {} and {}.".format(value_range[0], value_range[1]))
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

if __name__ == '__main__':
    rospy.init_node('robot_arm_controller_node')
    controller = RobotArmController()
    controller.move_to_home_position()
    amplitude, frequency = controller.get_user_input()
    controller.move_in_sinusoidal_wave(amplitude, frequency)
