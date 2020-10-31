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
//#include "ros/package.h"
//#include <sensor_msgs/JointState.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <interactive_markers/interactive_marker_server.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ApplyPlanningScene.h>
//#include <moveit/robot_state/conversions.h>

//#include <moveit/kinematic_constraints/utils.h>
//#include <eigen_conversions/eigen_msg.h>
//#include <eigen_conversions/eigen_kdl.h>

#include <string>
//#include <stdlib.h>
//#include <stdio.h>

//#include "Eigen/Dense"
//#include "Eigen/LU"

#include <collision_convex_model/collision_convex_model.h>
//#include "kin_dyn_model/kin_model.h"
//#include "planer_utils/random_uniform.h"
//#include "planer_utils/utilities.h"
//#include "planer_utils/double_joint_collision_checker.h"
#include "std_srvs/Trigger.h"

// plugin for robot interface
#include <pluginlib/class_loader.h>
#include <rcprg_planner/robot_interface.h>

namespace rcprg_planner {

class Planner {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_reset_;
    ros::ServiceServer service_plan_;
    ros::ServiceServer service_processWorld_;

    //const double PI;

    //KDL::Frame int_marker_pose_;

    std::string robot_description_str_;
    std::string robot_semantic_description_str_;

    //boost::shared_ptr<self_collision::CollisionModel> col_model_;
    //boost::shared_ptr<KinematicModel > kin_model_;
    //std::vector<KDL::Frame > links_fk_;


//    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
//    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;

    // ROS parameters
    std::string robot_interface_plugin_str_;

    // ROS pluginlib
    pluginlib::ClassLoader<RobotInterface> robot_interface_loader_;
    boost::shared_ptr<RobotInterface> robot_interface_;

public:
    Planner()
        : nh_("rcprg_planner")
        , robot_interface_loader_("rcprg_planner", "rcprg_planner::RobotInterface")
    {

        nh_.getParam("robot_interface_plugin", robot_interface_plugin_str_);
        if (robot_interface_plugin_str_.empty()) {
            ROS_ERROR("The ROS parameter \"robot_interface_plugin\" is empty");
            return;
        }
        ROS_INFO("Trying to load plugin: \"%s\"", robot_interface_plugin_str_.c_str());

        try
        {
            robot_interface_ = robot_interface_loader_.createInstance(robot_interface_plugin_str_);
            ROS_INFO("Loaded plugin: \"%s\"", robot_interface_plugin_str_.c_str());
            //ROS_INFO("loaded velma_ros_plugin::VelmaInterface");
        }
        catch(pluginlib::PluginlibException& ex)
        {
            //handle the class failing to load
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
            return;
        }


        service_reset_ = nh_.advertiseService("reset", &Planner::reset, this);
        service_plan_ = nh_.advertiseService("plan", &Planner::plan, this);
        service_processWorld_ = nh_.advertiseService("processWorld", &Planner::processWorld, this);

        nh_.getParam("/robot_description", robot_description_str_);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str_);

        std::string xml_out;
        self_collision::CollisionModel::convertSelfCollisionsInURDF(robot_description_str_, xml_out);

        //
        // moveit
        //
        robot_model_loader::RobotModelLoader robot_model_loader( robot_model_loader::RobotModelLoader::Options(xml_out, robot_semantic_description_str_) );
        robot_model_ = robot_model_loader.getModel();

        planning_scene_.reset( new planning_scene::PlanningScene(robot_model_) );

        planning_scene_->setStateFeasibilityPredicate( boost::bind(&RobotInterface::isStateValid, robot_interface_.get(), _1, _2) );

        planning_pipeline_.reset( new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters") );

/*
        // for debug only:

        planning_interface::PlannerManagerPtr planner_manager = planning_pipeline_->getPlannerManager();
        planning_interface::PlannerConfigurationMap conf_map = planner_manager->getPlannerConfigurations();

        std::cout << "description: " << planner_manager->getDescription() << std::endl;
        std::vector<std::string > planning_algorithms;
        planner_manager->getPlanningAlgorithms(planning_algorithms);
        for (int i = 0; i < planning_algorithms.size(); ++i) {
            std::cout << planning_algorithms[i] << std::endl;
        }
        for (planning_interface::PlannerConfigurationMap::const_iterator it = conf_map.begin(); it != conf_map.end(); ++it) {
            std::cout << it->first << " " << it->second.name << std::endl;
        }
*/
//        robot_model_loader_.reset( new robot_model_loader::RobotModelLoader(robot_model_loader::RobotModelLoader::Options(robot_description_str_, robot_semantic_description_str_)) );
//        planning_scene_monitor_.reset( new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_) );
//        planning_scene_monitor_->getRobotModel();


        //pluginlib::ClassLoader<velma_planner::RobotInterface> robot_interface_loader_("planner", "velma_planner::RobotInterface");
    }

    ~Planner() {
    }

    bool reset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        res.success = true;
        return true;
    }

    bool processWorld(moveit_msgs::ApplyPlanningScene::Request& req, moveit_msgs::ApplyPlanningScene::Response& res) {
        if (!planning_scene_->processPlanningSceneWorldMsg(req.scene.world)) {
          ROS_ERROR("Error in processPlanningSceneWorldMsg");
          res.success = false;
        }
        res.success = true;
        return true;
    }

    bool plan(moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res) {

        planning_interface::MotionPlanResponse response;

        planning_interface::MotionPlanRequest request = req.motion_plan_request;

        planning_pipeline_->generatePlan(planning_scene_, request, response);

        response.getMessage( res.motion_plan_response );

        /* Check that the planning was successful */
        if (response.error_code_.val != response.error_code_.SUCCESS)
        {
/*
            // for debug only:
            // get the specified start state
            robot_state::RobotState start_state = planning_scene_->getCurrentState();
            robot_state::robotStateMsgToRobotState(planning_scene_->getTransforms(), request.start_state, start_state);

            collision_detection::CollisionRequest creq;
            creq.contacts = true;
            creq.max_contacts = 100;
            creq.verbose = true;
            creq.group_name = request.group_name;
            collision_detection::CollisionResult cres;
            planning_scene_->checkCollision(creq, cres, start_state);
            if (cres.collision) {
                for (collision_detection::CollisionResult::ContactMap::const_iterator it = cres.contacts.begin(), end = cres.contacts.end(); it != end; ++it) {
                    ROS_INFO("contacts between %s and %s:", it->first.first.c_str(), it->first.second.c_str());
                    for (int i = 0; i < it->second.size(); ++i) {
                        ROS_INFO("%lf  %lf  %lf", it->second[i].pos(0), it->second[i].pos(1), it->second[i].pos(2));
                    }
                }
                ROS_INFO("collision");
            }
            else {
                ROS_INFO("no collision");
            }
*/
          ROS_ERROR("Could not compute plan successfully, error: %d. For more detailed error description please refer to moveit_msgs/MoveItErrorCodes", response.error_code_.val);
          return false;
        }

        return true;
    }

    void spin() {
        while (ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
};

}; // namespace rcprg_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "rcprg_planner");
    rcprg_planner::Planner planner;
    planner.spin();
    return 0;
}


