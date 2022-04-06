## Contains ROS-based Python interface for planner and some helper functions.
# @file planner.py
# @ingroup python_api
# @namespace planner.planner Contains ROS-based Python interface for planner and some helper functions.

# Copyright (c) 2018, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

import time
import rospy
import copy
import math

import PyKDL
import tf
from moveit_msgs.msg import RobotTrajectory, JointConstraint, Constraints, MotionPlanRequest,\
    OrientationConstraint
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest, ApplyPlanningScene,\
    ApplyPlanningSceneRequest
from trajectory_msgs.msg import JointTrajectory
from threading import RLock
from octomap_msgs.msg import Octomap
import tf_conversions.posemath as pm

def qMapToConstraints(q_map, tolerance=0.01, group=None):
    """!
    This function converts dictionary of joint positions to moveit_msgs.Constraints structure.

    @param q_map        dictionary: A dictionary {name:position} of desired joint positions.
    @param tolerance    float: Tolerance of goal position.
    @param group        list of strings: Names of active joints in planning group.
        Only constraints for these joints should be generated.
    @return Returns filled moveit_msgs.Constraints structure.
    """
    result = Constraints()
    for joint_name in q_map:
        if group != None and not joint_name in group:
            continue
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = q_map[ joint_name ]
        constraint.tolerance_above = tolerance
        constraint.tolerance_below = tolerance
        constraint.weight = 1.0
        result.joint_constraints.append( constraint )
    return result

#def endEffectorPoseToConstraints(effector_name, T_B_G, tolerance=0.01):
#    assert (effector_name == "left" or effector_name=="right")
#    result = Constraints()
#    return result

class OctomapListener:
    """!
    Class used for obtaining occupancy map (as octomap) from octomap server.
    """

    # Private method.
    def _octomap_callback(self, data):
        with self._lock:
            self._octomap = data

    def __init__(self, topic="/octomap_binary"):
        """!
        Initialization, ROS topic subscription.

        @param topic string: octomap ROS topic.
        """
        self._lock = RLock()
        self._octomap = None
        rospy.Subscriber(topic, Octomap, self._octomap_callback)

    def getOctomap(self, timeout_s=None):
        """!
        Get current octomap from octomap server.

        @param timeout_s float: Timeout in seconds.

        @return Octomap object of type octomap_msgs.msg.Octomap, or None if timed out.
        """
        if timeout_s == None:
            with self._lock:
                if self._octomap:
                    return copy.copy(self._octomap)
                return None
        else:
            time_start = time.time()
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
                with self._lock:
                    if self._octomap:
                        return copy.copy(self._octomap)
                time_now = time.time()
                if timeout_s and (time_now-time_start) > timeout_s:
                    return None
        return None

class Planner:
    """!
    Class used as planner interface.
    """
    def plan(self, q_start, goal_constraints, group_name, attached_collision_objects=None, is_diff=False,
                    path_constraints=None, trajectory_constraints=None, workspace_parameters=None,
                    planner_id=None, num_planning_attempts=1,
                    allowed_planning_time=10.0, max_velocity_scaling_factor=1.0,
                    max_acceleration_scaling_factor=1.0, keep_upright=None):
        """!
        Plan motion in joint space.

        @param q_start                  dictionary: A dictionary {name:position} of starting configuration.
        @param goal_constraints         list: A list of goal constraints of type moveit_msgs.msg.Constraints.
        @param group_name               string: Name of joint group to perform planning on.
        @param attached_collision_objects   list: Objects attached to robot represented as a list of objects
                                        of type moveit_msgs.msg.AttachedCollisionObject.
        @param is_diff
        @param path_constraints         Path constraints of type moveit_msgs.msg.Constraints.
        @param trajectory_constraints   Trajectory constraints of type moveit_msgs.msg.TrajectoryConstraints.
        @param workspace_parameters     Parameters of robots workspace of type moveit_msgs.msg.WorkspaceParameters.
        @param planner_id               string: Name of planning algorithm.
        @param num_planning_attempts    int: Maximum number of planning attempts.
        @param allowed_planning_time    float: Maximum allowed time for planning.
        @param max_velocity_scaling_factor      float: Scaling factor for velocity: the bigger, the faster robot moves.
        @param max_acceleration_scaling_factor  float: Scaling factor for acceleration: the bigger, the faster robot accelerates.

        @return Returns trajectory_msgs.msg.JointTrajectory.
        """
        req = MotionPlanRequest()

        joint_names = []
        for joint_name in q_start:
            req.start_state.joint_state.name.append( joint_name )
            req.start_state.joint_state.position.append( q_start[joint_name] )

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

        if not keep_upright is None:
            current_time = rospy.Time.now()
            for side_str, T_B_E in keep_upright.iteritems():
                assert isinstance(T_B_E, PyKDL.Frame)
                # This message contains the definition of an orientation constraint.
                ori_constr = OrientationConstraint()
                ori_constr.header.frame_id = 'world'

                # The robot link this constraint refers to
                ori_constr.link_name = '{}_arm_7_link'.format( side_str )

                #try:
                #self._listener.waitForTransform('torso_base', ori_constr.link_name,
                #                                            current_time, rospy.Duration(5.0))
                #pose = self._listener.lookupTransform('torso_base', ori_constr.link_name,
                #                                                                current_time)
                #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                #        tf2_ros.TransformException):
                #    pose = None
                #T_B_E = pm.fromTf(pose)
                qx, qy, qz, qw = T_B_E.M.GetQuaternion()
                # The desired orientation of the robot link specified as a quaternion
                ori_constr.orientation.x = qx
                ori_constr.orientation.y = qy
                ori_constr.orientation.z = qz
                ori_constr.orientation.w = qw

                # Tolerance on the three vector components of the orientation error (optional)
                ori_constr.absolute_x_axis_tolerance = math.radians(30.0)
                ori_constr.absolute_y_axis_tolerance = math.radians(30.0)
                ori_constr.absolute_z_axis_tolerance = math.radians(200.0)

                # Defines how the orientation error is calculated
                # The error is compared to the tolerance defined above
                #ori_constr.parameterization = OrientationConstraint.XYZ_EULER_ANGLES

                # The different options for the orientation error parameterization
                # - Intrinsic xyz Euler angles (default value)
                #uint8 XYZ_EULER_ANGLES=0
                # - A rotation vector. This is similar to the angle-axis representation,
                # but the magnitude of the vector represents the rotation angle.
                #uint8 ROTATION_VECTOR=1

                # A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
                ori_constr.weight = 1.0

                req.path_constraints.orientation_constraints.append( ori_constr )

        req.num_planning_attempts = num_planning_attempts

        req.allowed_planning_time = allowed_planning_time

        req.max_velocity_scaling_factor = max_velocity_scaling_factor

        req.max_acceleration_scaling_factor = max_acceleration_scaling_factor

        request = GetMotionPlanRequest()
        request.motion_plan_request = req

        try:
            res = self.plan_service( request ).motion_plan_response
        except rospy.service.ServiceException as e:
            print "Planner.plan(): could not plan:"
            print e
            return None

        if not res:
            return None

        if not self._max_traj_len is None and len(res.trajectory.joint_trajectory.points) > self._max_traj_len:
            print('Planner.plan(): Trajectory is too long: {}'.format( len(res.trajectory.joint_trajectory.points) ))
            return None

        return res.trajectory.joint_trajectory

    def splitTrajectory(self, joint_trajectory, max_traj_len):
        """!
        Split a long trajectory into a number of shorter trajectories.

        @param joint_trajectory         trajectory_msgs.msg.JointTrajectory: A joint trajectory to be splitted.
        @param max_traj_len             int: Maximum number of nodes in the result trajectory.
        @return Returns list of trajectory_msgs.msg.JointTrajectory -- the input trajectory split
        into a number of trajectories of length not greater than max_traj_len.
        """
        assert isinstance(joint_trajectory, JointTrajectory)
        if len(joint_trajectory.points) > max_traj_len:
            result = []
            #for idx in range(0, len(traj.trajectory.joint_trajectory.points), max_traj_len):
            idx = 0
            new_traj = JointTrajectory()
            new_traj.header = joint_trajectory.header
            new_traj.joint_names = joint_trajectory.joint_names
            time_base = rospy.Duration(0.0)
            for idx in range(len(joint_trajectory.points)):
                point = copy.copy( joint_trajectory.points[idx] )
                point.time_from_start = point.time_from_start - time_base
                new_traj.points.append( point )
                if len(new_traj.points) >= max_traj_len:
                    result.append( new_traj )
                    new_traj = JointTrajectory()
                    new_traj.header = joint_trajectory.header
                    new_traj.joint_names = joint_trajectory.joint_names
                    point = copy.copy( joint_trajectory.points[idx] )
                    time_base = point.time_from_start - rospy.Duration(0.5)
                    point.time_from_start = point.time_from_start - time_base
                    new_traj.points.append( point )

            if len(new_traj.points) > 0:
                result.append( new_traj )

            return result
        # else
        return [joint_trajectory]

    def processWorld(self, octomap):
        """!
        Updates occupancy map of the planner.

        @param octomap  octomap_msgs.msg.Octomap: Occupancy map.

        @return Returns True if the octomap was succesfully loaded, False otherwise.
        """
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
        """!
        Initialization of Python planner interface.

        @param max_traj_len int: Maximum number of nodes in a planned trajectory.
        """
        self._max_traj_len = max_traj_len
        self._listener = tf.TransformListener()

    def waitForInit(self, timeout_s=None):
        """!
        Wait until planner interface is not initialized.

        @param timeout_s    float: Timeout in seconds.

        @return Returns True if the Python planner interface was succesfully initialized, False otherwise.
        """
        try:
            rospy.wait_for_service('/rcprg_planner/plan', timeout=timeout_s)
            self.plan_service = rospy.ServiceProxy('/rcprg_planner/plan', GetMotionPlan)
        except:
            return False

        try:
            rospy.wait_for_service('/rcprg_planner/processWorld', timeout=timeout_s)
            self.processWorld_service = rospy.ServiceProxy('/rcprg_planner/processWorld', ApplyPlanningScene)
        except:
            return False
        return True

