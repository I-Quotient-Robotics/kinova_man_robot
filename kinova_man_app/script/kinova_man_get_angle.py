#! /usr/bin/env python

import roslib; roslib.load_manifest('kinova_man_app')
import rospy

import sys
import math

import actionlib
import kinova_msgs.msg

left_arm_driver_prefix = "left_arm_driver"
right_arm_driver_prefix = "right_arm_driver"

joing_angle_suffix = "/out/joint_angles"
joint_angles_action_suffix = "/joints_action/joint_angles"

left_arm_joint_pose = []
right_arm_joint_pose = []

def left_arm_joint_angle_cb(data):
    global left_arm_joint_pose
    data_list = []
    data_list.append(data.joint1)
    data_list.append(data.joint2)
    data_list.append(data.joint3)
    data_list.append(data.joint4)
    data_list.append(data.joint5)
    data_list.append(data.joint6)
    data_list.append(data.joint7)
    left_arm_joint_pose = data_list

    # rospy.loginfo("left arm: {0}".format(data_list))

def right_arm_joint_angle_cb(data):
    global right_arm_joint_pose
    data_list = []
    data_list.append(data.joint1)
    data_list.append(data.joint2)
    data_list.append(data.joint3)
    data_list.append(data.joint4)
    data_list.append(data.joint5)
    data_list.append(data.joint6)
    data_list.append(data.joint7)

    right_arm_joint_pose = data_list
    # rospy.loginfo("right arm: {0}".format(data_list))

def main():
    global left_arm_joint_pose, right_arm_joint_pose
    rospy.init_node("kinova_man_get_angle")

    # left arm angles topic
    rospy.Subscriber(left_arm_driver_prefix+joing_angle_suffix, kinova_msgs.msg.JointAngles, left_arm_joint_angle_cb)

    # left arm angles topic
    rospy.Subscriber(right_arm_driver_prefix+joing_angle_suffix, kinova_msgs.msg.JointAngles, right_arm_joint_angle_cb)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("arm: {0},\n            {1}".format(left_arm_joint_pose, right_arm_joint_pose))
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()