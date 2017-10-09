# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy

from moveit_msgs.msg import *
from moveit_msgs.srv import *

def qMapToConstraints(q_map, tolerance=0.01):
    result = Constraints()
    for joint_name in q_map:
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = q_map[ joint_name ]
        constraint.tolerance_above = tolerance
        constraint.tolerance_below = tolerance
        constraint.weight = 1.0
        result.joint_constraints.append( constraint )
    return result

class Planner:
    """
Class used as planner interface.
"""

    def plan(self, js, goal_constraints, group_name, attached_collision_objects=None, is_diff=False,
                    path_constraints=None, trajectory_constraints=None, workspace_parameters=None,
                    planner_id=None, num_planning_attempts=1,
                    allowed_planning_time=10.0, max_velocity_scaling_factor=1.0,
                    max_acceleration_scaling_factor=1.0):

        req = MotionPlanRequest()

        joint_names = []
        q_start = []
        for joint_name in js[1]:
            req.start_state.joint_state.name.append( joint_name )
            req.start_state.joint_state.position.append( js[1][joint_name] )

        if attached_collision_objects:
            req.start_state.attached_collision_objects = attached_collision_objects

        req.start_state.is_diff = is_diff

        req.goal_constraints = goal_constraints

        req.group_name = group_name

        if path_constraints:
            req.path_constraints = path_constraints

        if trajectory_constraints:
            req.trajectory_constraints = trajectory_constraints

        if workspace_parameters:
            req.workspace_parameters = workspace_parameters

        if planner_id:
            req.planner_id = planner_id

        req.num_planning_attempts = num_planning_attempts

        req.allowed_planning_time = allowed_planning_time

        req.max_velocity_scaling_factor = max_velocity_scaling_factor

        req.max_acceleration_scaling_factor = max_acceleration_scaling_factor

        request = GetMotionPlanRequest()
        request.motion_plan_request = req

        res = self.plan_service( request ).motion_plan_response

        if not res:
            return None, None

        traj = [[],[],None, []]
        time_prev = rospy.Duration()
        for point in res.trajectory.joint_trajectory.points:
            traj[0].append( point.positions )
            traj[1].append( point.velocities )
            traj[3].append( (point.time_from_start - time_prev).to_sec() )
            time_prev = point.time_from_start

        if len(traj[0]) > self.max_traj_len:
            return None, None
        return traj, res.trajectory.joint_trajectory.joint_names

    def processWorld(self, octomap):
        req = ApplyPlanningSceneRequest()
        #TODO: req.scene.name
        #TODO: req.scene.robot_state
        #TODO: req.scene.robot_model_name
        #TODO: req.scene.fixed_frame_transforms
        #TODO: req.scene.allowed_collision_matrix
        #TODO: req.scene.link_padding
        #TODO: req.scene.link_scale
        #TODO: req.scene.object_colors
        #TODO: req.scene.world.collision_objects

        req.scene.world.octomap.header.frame_id = "world"
        req.scene.world.octomap.octomap = octomap
        req.scene.is_diff = False
        res = self.processWorld_service(req)
        if not res.success:
            return False
        return True

    def __init__(self, max_traj_len):
        self.max_traj_len = max_traj_len

    def waitForInit(self, timeout_s=None):
        try:
            rospy.wait_for_service('/planner/plan', timeout=timeout_s)
            self.plan_service = rospy.ServiceProxy('/planner/plan', GetMotionPlan)
        except:
            return False

        try:
            rospy.wait_for_service('/planner/processWorld', timeout=timeout_s)
            self.processWorld_service = rospy.ServiceProxy('/planner/processWorld', ApplyPlanningScene)
        except:
            return False
        return True

