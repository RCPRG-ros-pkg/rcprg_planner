// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/GetMotionPlan.h>

//TODO: remove this
//#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

/*#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Path.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
*/
#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/utilities.h"
#include "planer_utils/double_joint_collision_checker.h"
#include "std_srvs/Trigger.h"
#include "planner_msgs/Plan.h"

class Planner {
private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::ServiceServer service_reset_;
    ros::ServiceServer service_plan_;

    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

    KDL::Frame int_marker_pose_;

    std::string robot_description_str_;
    std::string robot_semantic_description_str_;

    boost::shared_ptr<self_collision::CollisionModel> col_model_;
    boost::shared_ptr<KinematicModel > kin_model_;
    std::vector<KDL::Frame > links_fk_;


//    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
//    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;

    // ROS parameters
    std::vector<double > wcc_l_constraint_polygon_;
    int wcc_l_joint0_idx_;
    int wcc_l_joint1_idx_;

    std::vector<double > wcc_r_constraint_polygon_;
    int wcc_r_joint0_idx_;
    int wcc_r_joint1_idx_;


    boost::shared_ptr<DoubleJointCC > wcc_l_;
    boost::shared_ptr<DoubleJointCC > wcc_r_;

public:
    Planner() :
        nh_("planner"),
        PI(3.141592653589793),
        markers_pub_(nh_)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        service_reset_ = nh_.advertiseService("reset", &Planner::reset, this);
        service_plan_ = nh_.advertiseService("plan", &Planner::plan, this);

        nh_.getParam("/robot_description", robot_description_str_);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str_);

        nh_.getParam("/velma_core_cs/wcc_l/constraint_polygon", wcc_l_constraint_polygon_);
        nh_.getParam("/velma_core_cs/wcc_l/joint0_idx", wcc_l_joint0_idx_);
        nh_.getParam("/velma_core_cs/wcc_l/joint1_idx", wcc_l_joint1_idx_);

        nh_.getParam("/velma_core_cs/wcc_r/constraint_polygon", wcc_r_constraint_polygon_);
        nh_.getParam("/velma_core_cs/wcc_r/joint0_idx", wcc_r_joint0_idx_);
        nh_.getParam("/velma_core_cs/wcc_r/joint1_idx", wcc_r_joint1_idx_);

        if (wcc_l_constraint_polygon_.size() == 0 || (wcc_l_constraint_polygon_.size()%2) != 0) {
            ROS_ERROR("property \'constraint_polygon\' (l) has wrong size: %lu", wcc_l_constraint_polygon_.size());
        }

        if (wcc_l_joint0_idx_ == wcc_l_joint1_idx_) {
            ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (l) have the same value: %d", wcc_l_joint0_idx_);
        }

        if (wcc_r_constraint_polygon_.size() == 0 || (wcc_r_constraint_polygon_.size()%2) != 0) {
            ROS_ERROR("property \'constraint_polygon\' (r) has wrong size: %lu", wcc_r_constraint_polygon_.size());
        }

        if (wcc_r_joint0_idx_ == wcc_r_joint1_idx_) {
            ROS_ERROR("properties \'joint0_idx\' and \'joint1_idx\' (r) have the same value: %d", wcc_r_joint0_idx_);
        }

        wcc_l_.reset(new DoubleJointCC(0.0, wcc_l_constraint_polygon_));
        wcc_r_.reset(new DoubleJointCC(0.0, wcc_r_constraint_polygon_));

        //
        // collision model
        //
        col_model_ = self_collision::CollisionModel::parseURDF(robot_description_str_);
	    col_model_->parseSRDF(robot_semantic_description_str_);
        col_model_->generateCollisionPairs();
        links_fk_.resize(col_model_->getLinksCount());

        std::string xml_out = robot_description_str_;
        self_collision::CollisionModel::convertSelfCollisionsInURDF(robot_description_str_, xml_out);
//        std::cout << xml_out << std::endl;

        //
        // moveit
        //
        robot_model_loader::RobotModelLoader robot_model_loader( robot_model_loader::RobotModelLoader::Options(xml_out, robot_semantic_description_str_) );
        robot_model_ = robot_model_loader.getModel();

        planning_scene_.reset( new planning_scene::PlanningScene(robot_model_) );

        planning_scene_->setStateFeasibilityPredicate( boost::bind(&Planner::isStateValid, this, _1, _2) );

        planning_pipeline_.reset( new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters") );

//        robot_model_loader_.reset( new robot_model_loader::RobotModelLoader(robot_model_loader::RobotModelLoader::Options(robot_description_str_, robot_semantic_description_str_)) );
//        planning_scene_monitor_.reset( new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_) );
//        planning_scene_monitor_->getRobotModel();

    }

    ~Planner() {
    }

    bool reset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        res.success = true;
        return true;
    }

    bool plan(moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res) {
//    bool plan(planner_msgs::Plan::Request& req, planner_msgs::Plan::Response& res) {

        planning_interface::MotionPlanResponse response;

        planning_interface::MotionPlanRequest request = req.motion_plan_request;

        planning_pipeline_->generatePlan(planning_scene_, request, response);

        response.getMessage( res.motion_plan_response );

        /* Check that the planning was successful */
        if (response.error_code_.val != response.error_code_.SUCCESS)
        {
          ROS_ERROR("Could not compute plan successfully");
          return false;
        }

        return true;
    }

    bool isStateValid(const robot_state::RobotState& ss, bool verbose) {
        DoubleJointCC::Joints q_r(ss.getVariablePosition("right_arm_5_joint"), ss.getVariablePosition("right_arm_6_joint"));
        if ( wcc_r_->inCollision(q_r) ) {
            return false;
        }

        DoubleJointCC::Joints q_l(ss.getVariablePosition("left_arm_5_joint"), ss.getVariablePosition("left_arm_6_joint"));
        if ( wcc_l_->inCollision(q_l) ) {
            return false;
        }

        return true;
    }

    void spin() {
        while (ros::ok()) {
            ros::spinOnce();
        }
    }

/*    void stateOmplToEigen(const ompl::base::State *s, Eigen::VectorXd &x, int ndof) {
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            x(q_idx) = s->as<ompl::base::RealVectorStateSpace::StateType >()->operator[](q_idx);
        }
    }
*/
/*
    void generatePossiblePose(KDL::Frame &T_B_E, Eigen::VectorXd &q, int ndof, const std::string &effector_name, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const boost::shared_ptr<KinematicModel> &kin_model, const std::set<int> &excluded_q_ids=std::set<int>()) {
        while (true) {
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                if (excluded_q_ids.find(q_idx) == excluded_q_ids.end()) {
                    q(q_idx) = randomUniform(kin_model->getLowerLimit(q_idx), kin_model->getUpperLimit(q_idx));
                }
            }
            std::set<int> excluded_link_idx;
            std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            if (!self_collision::checkCollision(col_model, links_fk, excluded_link_idx)) {
                T_B_E = links_fk[col_model->getLinkIndex(effector_name)];
                break;
            }
        }
    }

    void generateBox(std::vector<KDL::Vector > &vertices, std::vector<int> &polygons, double size_x, double size_y, double size_z) {
        vertices.clear();
        polygons.clear();
        const int poly[] = {
        4, 0, 1, 2, 3,
        4, 4, 7, 6, 5,
        4, 0, 4, 5, 1,
        4, 3, 2, 6, 7,
        4, 5, 6, 2, 1,
        4, 0, 3, 7, 4,
        };
        const double vert[] = {
        0.5, -0.5, 0.5,
        0.5, 0.5, 0.5,
        -0.5, 0.5, 0.5,
        -0.5, -0.5, 0.5,
        0.5, -0.5, -0.5,
        0.5, 0.5, -0.5,
        -0.5, 0.5, -0.5,
        -0.5, -0.5, -0.5,
        };
        for (int i=0; i<8; i++) {
            vertices.push_back(KDL::Vector(vert[i*3]*size_x, vert[i*3+1]*size_y, vert[i*3+2]*size_z));
        }
        for (int i=0; i<6*5; i++) {
            polygons.push_back(poly[i]);
        }
    }

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        int_marker_pose_ = KDL::Frame(KDL::Rotation::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w), KDL::Vector(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
//      ROS_INFO_STREAM( feedback->marker_name << " is now at "
//          << feedback->pose.position.x << ", " << feedback->pose.position.y
//          << ", " << feedback->pose.position.z );
    }

    bool isStateValid(const ompl::base::State *s, const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model,
                        int ndof, const VelmaQ5Q6CollisionChecker &wcc1, const VelmaQ5Q6CollisionChecker &wcc2) {
        Eigen::VectorXd x(ndof);
        stateOmplToEigen(s, x, ndof);

        // check collision in wrists
        if (wcc1.inCollision(x) || wcc2.inCollision(x)) {
            return false;
        }

        std::vector<self_collision::CollisionInfo> link_collisions;
        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
        }

        std::set<int> excluded_link_idx;
        return !self_collision::checkCollision(col_model, links_fk, excluded_link_idx);
    }

    void spin() {

        // initialize random seed
        srand(time(NULL));

        // dynamics model
        boost::shared_ptr<DynamicModel > dyn_model( new DynModelVelma() );


        // external collision objects - part of virtual link connected to the base link
        self_collision::Link::VecPtrCollision col_array;

        // the walls
        std::vector<KDL::Vector > vertices;
        std::vector<int > polygons;
        generateBox(vertices, polygons, 0.2, 2.0, 2.0);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, KDL::Frame(KDL::Vector(-0.65, 0.0, 1.3))) );

        generateBox(vertices, polygons, 2.0, 0.2, 2.0);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, KDL::Frame(KDL::Vector(0.35, 1.0, 1.3))) );

        generateBox(vertices, polygons, 2.0, 2.0, 0.2);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, KDL::Frame(KDL::Vector(0.35, 0.0, 2.3))) );

        // the cabinet
        KDL::Frame T_W_C(KDL::Vector(1.2,0,1.5));
        generateBox(vertices, polygons, 0.4, 0.6, 0.02);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,-0.3))) );
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,0.0))) );
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0,0.3))) );

        generateBox(vertices, polygons, 0.02, 0.6, 0.6);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0.2,0,0))) );

        generateBox(vertices, polygons, 0.4, 0.02, 0.6);
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,-0.3,0))) );
        col_array.push_back( self_collision::createCollisionConvex(vertices, polygons, T_W_C*KDL::Frame(KDL::Vector(0,0.3,0))) );

        if (!col_model->addLink("env_link", "torso_base", col_array)) {
            ROS_ERROR("ERROR: could not add external collision objects to the collision model");
            return;
        }
        col_model->generateCollisionPairs();

        VelmaQ5Q6CollisionChecker wrist_cc_r(6, 7, 0.2, false);
        VelmaQ5Q6CollisionChecker wrist_cc_l(13, 14, 0.2, true);

        //
        // robot state
        //
        std::vector<std::string > joint_names;
        joint_names.push_back("torso_0_joint");
        joint_names.push_back("right_arm_0_joint");
        joint_names.push_back("right_arm_1_joint");
        joint_names.push_back("right_arm_2_joint");
        joint_names.push_back("right_arm_3_joint");
        joint_names.push_back("right_arm_4_joint");
        joint_names.push_back("right_arm_5_joint");
        joint_names.push_back("right_arm_6_joint");
        joint_names.push_back("left_arm_0_joint");
        joint_names.push_back("left_arm_1_joint");
        joint_names.push_back("left_arm_2_joint");
        joint_names.push_back("left_arm_3_joint");
        joint_names.push_back("left_arm_4_joint");
        joint_names.push_back("left_arm_5_joint");
        joint_names.push_back("left_arm_6_joint");

        int ndof = joint_names.size();

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.5;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }
//        q[0] = 1.2;
        q[2] = 1.0;

        std::string effector_name = "right_HandPalmLink";
        int effector_idx = col_model->getLinkIndex(effector_name);


        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        Eigen::VectorXd max_q(ndof);
        max_q(0) = 10.0/180.0*PI;               // torso_0_joint
        max_q(1) = max_q(8) = 20.0/180.0*PI;    // arm_0_joint
        max_q(2) = max_q(9) = 20.0/180.0*PI;    // arm_1_joint
        max_q(3) = max_q(10) = 30.0/180.0*PI;   // arm_2_joint
        max_q(4) = max_q(11) = 40.0/180.0*PI;   // arm_3_joint
        max_q(5) = max_q(12) = 50.0/180.0*PI;   // arm_4_joint
        max_q(6) = max_q(13) = 50.0/180.0*PI;   // arm_5_joint
        max_q(7) = max_q(14) = 50.0/180.0*PI;   // arm_6_joint

        boost::shared_ptr<DynamicsSimulatorHandPose> sim( new DynamicsSimulatorHandPose(ndof, 6, effector_name, col_model, kin_model, dyn_model, joint_names, 2.0*max_q) );

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 100000;
        ros::Rate loop_rate(500);

        while (true) {
            generatePossiblePose(r_HAND_target, q, ndof, effector_name, col_model, kin_model);
            sim->setState(q, dq, ddq);
            sim->setTarget(r_HAND_target);
            sim->oneStep();
            if (!sim->inCollision()) {
                break;
            }
        }

        int_marker_pose_ = r_HAND_target;
        // create an interactive marker server on the topic namespace simple_marker
        interactive_markers::InteractiveMarkerServer server("simple_marker");
        tf::Vector3 position;
        position = tf::Vector3( 0, 0, 0);
        make6DofMarker(server, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, r_HAND_target, true);
        // 'commit' changes and send to all clients
        server.applyChanges();


        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {
            lower_limit[q_idx] = kin_model->getLowerLimit(q_idx);
            upper_limit[q_idx] = kin_model->getUpperLimit(q_idx);
        }

        //
        // ompl
        //

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(ndof));
        ompl::base::RealVectorBounds bounds(ndof);

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            bounds.setLow(q_idx, lower_limit(q_idx));
            bounds.setHigh(q_idx, upper_limit(q_idx));
        }
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
        si->setStateValidityChecker( boost::bind(&Planner::isStateValid, this, _1, col_model, kin_model, ndof, wrist_cc_r, wrist_cc_l) );
        si->setStateValidityCheckingResolution(1.0/180.0*PI);
        si->setup();

//        std::string mode = "random_dest";
//        std::string mode = "marker_dest";
        std::string mode = "random_start";

        bool collision_in_prev_step = false;
        while (ros::ok()) {

            // destination pose
            r_HAND_target = KDL::Frame(KDL::Rotation::RotY(90.0/180.0*PI), KDL::Vector(0.95, 0.0, 1.35));

            // obtain random starting configuration
            while (true) {
                KDL::Frame fr_tmp;
                generatePossiblePose(fr_tmp, q, ndof, effector_name, col_model, kin_model);
                dq.setZero();
                ddq.setZero();
                sim->setState(q, dq, ddq);
                sim->setTarget(fr_tmp);
                sim->oneStep();
                if (!sim->inCollision()) {
                    break;
                }
            }

            Eigen::VectorXd init_q(q);

            bool goal_found = false;
            // try several arm configurations
            for (int try_idx = 0; try_idx < 10; try_idx++) {

//                std::cout << "try_idx: " << try_idx << std::endl;
                // set initial state
                dq.setZero();
                ddq.setZero();
                sim->setState(q, dq, ddq);
                sim->setTarget(r_HAND_target);

                std::list<Eigen::VectorXd > path_sim;
                Eigen::VectorXd sim_q(ndof), sim_dq(ndof), sim_ddq(ndof);
                // simulate
                for (int loop_idx = 0; loop_idx < 8000; loop_idx++) {

                    if (!ros::ok()) {
                        return;
                    }

                    sim->oneStep();//&markers_pub_, 3000);
                    if (sim->inCollision()) {
                        std::cout << "collision" << std::endl;
                        break;
                    }

                    sim->getState(sim_q, sim_dq, sim_ddq);

                    path_sim.push_back(sim_q);

                    // calculate forward kinematics for all links
                    for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                        kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), sim_q);
                    }

                    // publish markers and robot state with limited rate
                    ros::Duration time_elapsed = ros::Time::now() - last_time;
                    if (time_elapsed.toSec() > 0.05) {
                        publishJointState(joint_state_pub_, sim_q, joint_names, ign_q, ign_joint_names);
                        int m_id = 0;
                        m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                        markers_pub_.publish();
//                        markers_pub_.clear();
                        last_time = ros::Time::now();
                    }
                    else {
                        markers_pub_.clear();
                    }
                    ros::spinOnce();
//                    loop_rate.sleep();

                    // check if the goal is reached
                    KDL::Twist goal_diff = KDL::diff(links_fk[effector_idx], r_HAND_target, 1.0);
                    if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*PI) {
                        std::cout << "found goal " << try_idx << " " << goal_diff.vel.Norm() << " " << goal_diff.rot.Norm() << std::endl;
                        goal_found = true;
                        break;
                    }
                }

                std::set<int> excluded_q_ids;
                excluded_q_ids.insert(0);
                excluded_q_ids.insert(8);
                excluded_q_ids.insert(9);
                excluded_q_ids.insert(10);
                excluded_q_ids.insert(11);
                excluded_q_ids.insert(12);
                excluded_q_ids.insert(13);
                excluded_q_ids.insert(14);

                if (goal_found) {

                    if ( (init_q-q).norm() < 0.0001) {
                        std::cout << "no need for rrt planning" << std::endl;
                    }
                    else {
                        std::cout << "running rrt planner..." << std::endl;
                        ompl::base::ScopedState<> start(space);
                        ompl::base::ScopedState<> goal(space);

                        for (int q_idx = 0; q_idx < ndof; q_idx++) {
                            start[q_idx] = init_q(q_idx);
                            goal[q_idx] = q(q_idx);
                        }

                        if (!isStateValid(start.get(), col_model, kin_model, ndof, wrist_cc_r, wrist_cc_l)) {
                            std::cout << "start state is invalid" << std::endl;
                        }
                        if (!isStateValid(goal.get(), col_model, kin_model, ndof, wrist_cc_r, wrist_cc_l)) {
                            std::cout << "goal state is invalid" << std::endl;
                        }

                        // enable arm joints only
                        for (int q_idx = 0; q_idx < ndof; q_idx++) {
                            if (excluded_q_ids.find(q_idx) == excluded_q_ids.end()) {
                                bounds.setLow(q_idx, lower_limit(q_idx));
                                bounds.setHigh(q_idx, upper_limit(q_idx));
                            }
                            else {
                                bounds.setLow(q_idx, init_q(q_idx));
                                bounds.setHigh(q_idx, init_q(q_idx));
                            }
                        }
                        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

                        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
                        pdef->clearStartStates();
                        pdef->setStartAndGoalStates(start, goal);

//                        ompl::base::PlannerPtr planner(new ompl::geometric::LBTRRT(si));
//                        ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(si));
                        ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
                        planner->setProblemDefinition(pdef);
                        planner->setup();

                        ompl::base::PlannerStatus status = planner->solve(10.0);

                        if (status) {
                            std::cout << "rrt planner ok" << std::endl;

                            // get rrt path
                            ompl::base::PathPtr path = pdef->getSolutionPath();
                            std::cout << "path length: " << path->length() << std::endl;
                            boost::shared_ptr<ompl::geometric::PathGeometric > ppath = boost::static_pointer_cast<ompl::geometric::PathGeometric >(path);

                            std::list<Eigen::VectorXd > path2;
                            for (int i = 0; i< ppath->getStateCount(); i++) {
                                ompl::base::State *s = ppath->getState(i);
                                Eigen::VectorXd x(ndof);
                                stateOmplToEigen(s, x, ndof);
                                path2.push_back(x);
                            }

                            // join the path
                            path2.insert(path2.end(), path_sim.begin(), path_sim.end());

                            getchar();
                            for (double f = 0.0; f < 1.0; f += 0.005/path->length()) {
                                Eigen::VectorXd x(ndof);
                                getPointOnPath(path2, f, x);

                                std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
                                // calculate forward kinematics for all links
                                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                                    kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
                                }

                                publishJointState(joint_state_pub_, x, joint_names, ign_q, ign_joint_names);
                                int m_id = 0;
                                m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);

//                                std::vector<self_collision::CollisionInfo> link_collisions;
//                                self_collision::getCollisionPairs(col_model, links_fk, 0.2, link_collisions);
//                                for (std::vector<self_collision::CollisionInfo>::const_iterator it = link_collisions.begin(); it != link_collisions.end(); it++) {
//                                    m_id = markers_pub_.addVectorMarker(m_id, it->p1_B, it->p2_B, 1, 1, 1, 1, 0.01, "world");                        
//                                }

//                                std::set<int> excluded_link_idx;
//                                if (self_collision::checkCollision(col_model, links_fk, excluded_link_idx)) {
//                                    std::cout << "collision " << f << std::endl;
//                                }

//                                markers_pub_.addEraseMarkers(m_id, m_id+100);

                                markers_pub_.publish();

                                ros::spinOnce();
                                ros::Duration(0.01).sleep();
                            }

                        }
                        else {
                            std::cout << "rrt planner failed" << std::endl;
                        }
                    }

                    break;
                }

                // change the configuration of the arm to random
                {
                    KDL::Frame fr_tmp;

                    while (true) {
                        KDL::Frame fr_tmp;
                        generatePossiblePose(fr_tmp, q, ndof, effector_name, col_model, kin_model, excluded_q_ids);
                        dq.setZero();
                        ddq.setZero();
                        sim->setState(q, dq, ddq);
                        sim->setTarget(fr_tmp);
                        sim->oneStep();
                        if (!sim->inCollision()) {
                            break;
                        }
                    }
                }
            }
            if (!goal_found) {
                std::cout << "goal not found" << std::endl;
            }
        }

                    //
                    // ompl
                    //
                    ompl::base::ScopedState<> start(space);
                    ompl::base::ScopedState<> goal(space);

                    KDL::Frame T_B_E_current;
                    kin_model->calculateFk(T_B_E_current, effector_name, q);
                    for (int d_idx = 0; d_idx < 3; d_idx++) {
                        start[d_idx] = T_B_E_current.p[d_idx];
                        goal[d_idx] = r_HAND_target.p[d_idx];
                    }

                    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
                    pdef->clearStartStates();
                    pdef->setStartAndGoalStates(start, goal);

        //            ompl::base::PlannerPtr planner(new ompl::geometric::LBTRRT(si));
                    ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(si));
        //            ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
                    planner->setProblemDefinition(pdef);
                    planner->setup();

                    ompl::base::PlannerStatus status = planner->solve(2.0);

                    int m_id = 1000;
                    if (status) {
                        std::cout << "ompl found solution" << std::endl;
                        ompl::base::PathPtr path = pdef->getSolutionPath();
                        std::cout << "ompl path length: " << path->length() << std::endl;
                        boost::shared_ptr<ompl::geometric::PathGeometric > ppath = boost::static_pointer_cast<ompl::geometric::PathGeometric >(path);

                        std::list<Eigen::VectorXd > path2;
                        for (int i = 0; i< ppath->getStateCount(); i++) {
                            ompl::base::State *s = ppath->getState(i);
                            Eigen::VectorXd x(3);
                            stateOmplToEigen(s, x, 3);
                            path2.push_back(x);
                        }
                        for (std::list<Eigen::VectorXd >::const_iterator it1 = path2.begin(), it2=++path2.begin(); it2 != path2.end(); it1++, it2++) {
                            m_id = markers_pub_.addVectorMarker(m_id, KDL::Vector( (*it1)(0), (*it1)(1), (*it1)(2) ), KDL::Vector( (*it2)(0), (*it2)(1), (*it2)(2) ), 0, 0, 1, 1, 0.01, "world");
                        }
                    }
                    markers_pub_.addEraseMarkers(m_id, m_id+1000);

                }
                loop_counter += 1;
            publishTransform(br, r_HAND_target, "effector_dest", "world");


        }

    }
*/
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    Planner planner;
    planner.spin();
    return 0;
}


