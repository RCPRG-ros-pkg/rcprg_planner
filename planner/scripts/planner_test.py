#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import sys
import rospy
import math
import copy
import tf

from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import * 
import tf_conversions.posemath as pm
import PyKDL
from cartesian_trajectory_msgs.msg import *
import actionlib

from velma_common.velma_interface import *

from moveit_msgs.msg import *
from moveit_msgs.srv import *

if __name__ == "__main__":

    rospy.init_node('planner_test', anonymous=True)

    rospy.sleep(1)

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    print "waiting for init..."
    velma.waitForInit()
    print "init ok"

#    print "moving to current position"
    js = velma.getLastJointState()
    joint_names = []
    q_start = []
    for joint_name in js[1]:
        joint_names.append(joint_name)
        q_start.append(js[1][joint_name])

    q_map = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.57,
        'right_arm_2_joint':1.57,
        'right_arm_3_joint':1.57,
        'right_arm_4_joint':0.0,
#        'right_arm_5_joint':-1.57,
#        'right_arm_6_joint':0.0,

        'right_arm_5_joint':1.57,
        'right_arm_6_joint':2.8,

#        'right_arm_5_joint':1.57,
#        'right_arm_6_joint':1.57,

        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.57,
        'left_arm_2_joint':-1.57,
        'left_arm_3_joint':-1.7,
        'left_arm_4_joint':0.0,
#        'left_arm_5_joint':1.57,
#        'left_arm_6_joint':0.0
#        'left_arm_5_joint':-1.57,
#        'left_arm_6_joint':-2.8
        'left_arm_5_joint':-1.57,
        'left_arm_6_joint':2.8
        }


    print "waiting for service..."
    rospy.wait_for_service('/planner/plan')

    try:
        print "getting service..."
        plan = rospy.ServiceProxy('/planner/plan', GetMotionPlan)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    req = MotionPlanRequest()
    for i in range(len(joint_names)):
        req.start_state.joint_state.name.append( joint_names[i] )
        req.start_state.joint_state.position.append( q_start[i] )
    req.start_state.is_diff = False
    goal_constraints = Constraints()
    for i in range(len(joint_names)):
        constraint = JointConstraint()
        constraint.joint_name = joint_names[i]
        if joint_names[i] in q_map:
            constraint.position = q_map[ joint_names[i] ]
        else:
            constraint.position = 0.0
        constraint.tolerance_above = 0.1
        constraint.tolerance_below = 0.1
        constraint.weight = 1.0
        goal_constraints.joint_constraints.append( constraint )
    req.goal_constraints.append(goal_constraints)
    req.group_name = "impedance_joints"
    req.num_planning_attempts = 1
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = 0.1

    request = GetMotionPlanRequest()
    request.motion_plan_request = req
    print "calling service..."
    res = plan( request ).motion_plan_response

    print "ok"

    traj = [[],[],None, []]
    time_prev = rospy.Duration()
    for point in res.trajectory.joint_trajectory.points:
        traj[0].append( point.positions )
        traj[1].append( point.velocities )
        traj[3].append( (point.time_from_start - time_prev).to_sec() )
        time_prev = point.time_from_start

    velma.moveJointTraj(traj, res.trajectory.joint_trajectory.joint_names, start_time=0.5)
    velma.waitForJoint()


