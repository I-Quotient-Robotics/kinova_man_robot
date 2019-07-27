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

# action client
left_arm_client = actionlib.SimpleActionClient(left_arm_driver_prefix+joint_angles_action_suffix, kinova_msgs.msg.ArmJointAnglesAction)
right_arm_client = actionlib.SimpleActionClient(right_arm_driver_prefix+joint_angles_action_suffix, kinova_msgs.msg.ArmJointAnglesAction)

def arm_move(step_name, left_arm_joint_pose, right_arm_joint_pose):
    left_goal = kinova_msgs.msg.ArmJointAnglesGoal()
    left_goal.angles.joint1 = left_arm_joint_pose[0]
    left_goal.angles.joint2 = left_arm_joint_pose[1]
    left_goal.angles.joint3 = left_arm_joint_pose[2]
    left_goal.angles.joint4 = left_arm_joint_pose[3]
    left_goal.angles.joint5 = left_arm_joint_pose[4]
    left_goal.angles.joint6 = left_arm_joint_pose[5]
    left_goal.angles.joint7 = left_arm_joint_pose[6]

    right_goal = kinova_msgs.msg.ArmJointAnglesGoal()
    right_goal.angles.joint1 = right_arm_joint_pose[0]
    right_goal.angles.joint2 = right_arm_joint_pose[1]
    right_goal.angles.joint3 = right_arm_joint_pose[2]
    right_goal.angles.joint4 = right_arm_joint_pose[3]
    right_goal.angles.joint5 = right_arm_joint_pose[4]
    right_goal.angles.joint6 = right_arm_joint_pose[5]
    right_goal.angles.joint7 = right_arm_joint_pose[6]

    # send action
    left_arm_client.send_goal(left_goal)
    right_arm_client.send_goal(right_goal)
    
    # wait for done
    left_arm_client.wait_for_result()
    right_arm_client.wait_for_result()

    rospy.Rate(10).sleep()

    rospy.loginfo(step_name)
    return True

def main():
    rospy.init_node("kinova_man_dance_node")

    rospy.loginfo("wait for server")
    left_arm_client.wait_for_server()
    right_arm_client.wait_for_server()
    rospy.loginfo("server up")

    # arm home
    arm_move( "step_home",  [20.972867965698242, 84.60040283203125, 268.98779296875, 40.71287536621094, 190.1157684326172, 186.07913208007812, 374.65679931640625],
            [340.31231689453125, 273.77520751953125, 109.82598114013672, 310.3317565917969, 109.12030792236328, 188.05908203125, 441.1184387207031])

    # start dance
    for i in range(2):
        arm_move( "step_1_{0}_1".format(i), [37.390350341796875, 100.74581909179688, 232.2755889892578, 40.82371520996094, 190.3650665283203, 186.07386779785156, 374.51275634765625],
            [335.67266845703125, 291.4468688964844, 69.18525695800781, 310.2998962402344, 109.12030792236328, 188.06065368652344, 441.1184387207031])

        arm_move( "step_1_{0}_2".format(i),  [21.627946853637695, 77.11163330078125, 296.83319091796875, 40.894222259521484, 190.3234405517578, 186.07386779785156, 374.5220031738281],
            [342.9836120605469, 265.6162109375, 130.02305603027344, 310.1519470214844, 109.12045288085938, 188.0593719482422, 441.1184387207031])

    # arm_move( "step_3", [37.801483154296875, 84.53439331054688, 237.16064453125, 40.6888542175293, 190.1163330078125, 186.08041381835938, 374.6590576171875],
    #         [322.7987365722656, 273.9111328125, 75.310546875, 310.44384765625, 109.11491394042969, 188.11703491210938, 441.1157531738281])

    # arm_move( "step_4", [37.80138397216797, 84.51075744628906, 296.59796142578125, 40.68875503540039, 190.1166229248047, 186.08055114746094, 374.6590576171875],
    #         [322.78204345703125, 273.86358642578125, 133.3997039794922, 310.4437561035156, 109.11491394042969, 188.05142211914062, 441.1157531738281])

    # arm_move( "step_5", [37.801483154296875, 84.53439331054688, 237.16064453125, 40.6888542175293, 190.1163330078125, 186.08041381835938, 374.6590576171875],
    #         [322.7987365722656, 273.9111328125, 75.310546875, 310.44384765625, 109.11491394042969, 188.11703491210938, 441.1157531738281])

    # arm_move( "step_6", [37.80138397216797, 84.51075744628906, 296.59796142578125, 40.68875503540039, 190.1166229248047, 186.08055114746094, 374.6590576171875],
    #         [322.78204345703125, 273.86358642578125, 133.3997039794922, 310.4437561035156, 109.11491394042969, 188.05142211914062, 441.1157531738281])

    # arm_move( "step_7", [37.801483154296875, 84.53439331054688, 237.16064453125, 40.6888542175293, 190.1163330078125, 186.08041381835938, 374.6590576171875],
    #         [322.7987365722656, 273.9111328125, 75.310546875, 310.44384765625, 109.11491394042969, 188.11703491210938, 441.1157531738281])

    # arm_move( "step_8", [37.80138397216797, 84.51075744628906, 296.59796142578125, 40.68875503540039, 190.1166229248047, 186.08055114746094, 374.6590576171875],
    #         [322.78204345703125, 273.86358642578125, 133.3997039794922, 310.4437561035156, 109.11491394042969, 188.05142211914062, 441.1157531738281])

    arm_move( "step_done", [37.80138397216797, 84.51290893554688, 268.8753662109375, 40.687583923339844, 190.11676025390625, 186.08055114746094, 374.6590576171875],
            [322.78155517578125, 273.8656311035156, 109.68086242675781, 310.4440612792969, 109.1147689819336, 188.0507049560547, 441.1157531738281])

    rospy.loginfo("dance done")

if __name__ == '__main__':
    main()