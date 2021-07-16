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

#include <math.h>
#include <mutex>
#include <sys/stat.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
//#include "ros/package.h"
#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <eigen_conversions/eigen_kdl.h>

#include <string>
#include <stdio.h>

#include <collision_convex_model/collision_convex_model.h>
#include "kin_dyn_model/kin_model.h"
#include "std_srvs/Trigger.h"

// plugin for robot interface
#include <pluginlib/class_loader.h>
#include <rcprg_planner/robot_interface.h>

#include <rcprg_ros_utils/marker_publisher.h>

#include <rcprg_planner_msgs/ReachabilityRange.h>

#include "reachability_range.h"

namespace rcprg_planner {

static const double PI=3.14159265359;
typedef std::array<double, 7 > arr_double7;
typedef std::vector<double > vec_double;
typedef std::array<int, 7 > arr_int7;

KDL::Frame pose2KDL(const geometry_msgs::Pose& pose) {
    return KDL::Frame(KDL::Rotation::Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w), KDL::Vector(
        pose.position.x,
        pose.position.y,
        pose.position.z));
}

void getUniformDirections(double angle, std::vector<KDL::Vector >& result) {
    const double PI = 3.14159265358979323846;
    int alpha_angles_count = int(PI/angle)+1;
    if (alpha_angles_count < 2) {
        throw std::invalid_argument("angle is too big");
    }

    for (int i_alpha = 0; i_alpha <= alpha_angles_count; ++i_alpha) {
        double alpha = PI * double(i_alpha) / alpha_angles_count;
        double z = 1.0 * cos(alpha);
        double r = 1.0 * sin(alpha);
        double l = 2*PI*r;
        int beta_angles_count = int(l/angle)+1;
        for (int i_beta = 0; i_beta < beta_angles_count; ++i_beta) {
            double beta = 2.0 * PI * double(i_beta) / beta_angles_count;
            double x = r * cos(beta);
            double y = r * sin(beta);
            result.push_back( KDL::Vector(x, y, z) );
        }
    }
}

class LinearIntervalFunction {
private:
    typedef std::vector<double > Vec;
    Vec x_;
    Vec val_;
public:
    LinearIntervalFunction(const Vec& x = Vec(), const Vec& val = Vec())
    : x_(x)
    , val_(val)
    {
        if (x_.size() != val_.size()) {
            throw std::invalid_argument("x and v are of different size");
        }
    }

    void addPoint(double x, double val) {
        if (x_.size() == 0) {
            x_.push_back( x );
            val_.push_back(val);
            return;
        }

        Vec::iterator it_val = val_.begin();
        for (Vec::iterator it_x = x_.begin(); it_x != x_.end(); ++it_x, ++it_val) {
            if (x < *it_x) {
                x_.insert( it_x, x );
                val_.insert( it_val, val );
                return;
            }
            else if (x == *it_x) {
                throw std::invalid_argument("Added point with the same x twice");
            }
        }
        x_.insert( x_.end(), x );
        val_.insert( val_.end(), val );
    }

    double interpolate(double x) {
        if (x_.size() < 2) {
            throw std::invalid_argument("Could not interpolate the function with only one point");
        }

        if (x < x_[0]) {
            throw std::invalid_argument("x is below the domain range");
        }

        for (int idx = 0; idx < x_.size()-1; ++idx) {
            if (x >= x_[idx] && x <= x_[idx+1]) {
                double f = (x - x_[idx]) / (x_[idx+1] - x_[idx]);
                return (1.0-f) * val_[idx] + f * val_[idx+1];
            }
        }
        throw std::invalid_argument("x is above the domain range");
    }
};

class WorkspaceTool {
private:
    ros::NodeHandle nh_;

    ros::ServiceServer service_reacahbility_range_;

    MarkerPublisher mp_;

    ReachabilityRangePtr rr_;

    std::string robot_description_str_;
    std::string robot_description_semantic_str_;

    boost::shared_ptr<KinematicModel > kin_model_;

    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;

    // ROS parameters
    std::string robot_interface_plugin_str_;

    // ROS pluginlib
    pluginlib::ClassLoader<RobotInterface> robot_interface_loader_;
    boost::shared_ptr<RobotInterface> robot_interface_;

    sensor_msgs::JointState::ConstPtr current_joint_states_ptr_;
    std::mutex current_joint_states_ptr_mutex_;

    std::vector<KDL::Vector > directions_;

    std::string map_filepath_;

public:

    ReachabilityRangePtr createModel() {
        // Those limits are adjusted to cart_imp mode at default starting configuration
        arr_double7 limits_lo({-2.96,-2.09,-2.96,0.2,-2.96,-2.09,-2.96});
        arr_double7 limits_hi({2.96,-0.2,2.96,2.095,2.96,-0.2,2.96});

        std::vector<std::string > joint_names({
            "right_arm_0_joint", "right_arm_1_joint", "right_arm_2_joint", "right_arm_3_joint",
            "right_arm_4_joint", "right_arm_5_joint", "right_arm_6_joint",});

        moveit::core::RobotState ss(robot_model_);
        ss.setToDefaultValues();

        // TODO: Determine rough configurations that fit into the bounding box:
        BoundingBox bb( KDL::Vector(0, -1, 0.5), KDL::Vector(1, 0.3, 1.5) );

        double limit_dist = 10.0/180.0*PI;
        // Define step value for each axis separately
        std::vector<double > val_step_vec = {10.0/180.0*PI, 10.0/180.0*PI, 15.0/180.0*PI,
            15.0/180.0*PI, 20.0/180.0*PI, 20.0/180.0*PI, 20.0/180.0*PI};

        DiscreteNSpace dspace7(7);

        // Determine discrete values for each axis
        const int DOF = joint_names.size();
        for (int idx = 0; idx < DOF; ++idx) {
            vec_double values;
            for (double val = limits_lo[idx]+limit_dist; val < limits_hi[idx]-limit_dist;
                                                                        val+=val_step_vec[idx]) {
                values.push_back( val );
            }
            values.push_back( limits_hi[idx]-limit_dist );
            dspace7.setSlice(idx, values);
        }

        // Select all configurations that lead to the end effector within the bounding box
        DiscreteNSpace::Index index = dspace7.getZeroIndex();
        DiscreteSE3 dspace_se3(bb, 0.05);
        ReachabilityRangePtr rr(new ReachabilityRange(dspace7, dspace_se3));

        KDL::Frame T_W_T0;
        tf::transformEigenToKDL( ss.getGlobalLinkTransform("torso_link0"), T_W_T0);
        KDL::Frame T_T0_W = T_W_T0.Inverse();

        int step = 0;
        do {
            DiscreteNSpace::Value val = dspace7.getIndexValue(index);
            for (int j = 0; j < joint_names.size(); ++j) {
                ss.setVariablePosition(joint_names[j], val[j]);
            }
            ss.update();
            KDL::Frame T_W_Wr;
            tf::transformEigenToKDL( ss.getGlobalLinkTransform("right_arm_7_link"), T_W_Wr);
            if ( bb.isIn(T_W_Wr.p) ) {
                if (planning_scene_->isStateValid(ss)) {
                    KDL::Frame T_T0_Wr = T_T0_W * T_W_Wr;
                    rr->addPoint(index, T_T0_Wr);
                }
            }
            if ((step%1000) == 0) {
                std::cout << "sample " << step << " of " << dspace7.getSpaceSize()
                            << ", valid config.: " << rr->getSamplesCount() << std::endl;
            }
            ++step;
            if (!ros::ok()) {
                break;
            }
            index = dspace7.getNextIndex(index);
        } while (!dspace7.isZeroIndex(index));

        return rr;
    }

    bool canReadFile(const std::string& filepath) const {
        if (FILE *file = fopen(filepath.c_str(), "rb")) {
            fclose(file);
            return true;
        } else {
            return false;
        }
    }

    bool canWriteFile(const std::string& filepath) const {
        if (FILE *file = fopen(filepath.c_str(), "w")) {
            fclose(file);
            return true;
        } else {
            return false;
        }
    }

    std::string extractDirName(const std::string& filepath) const {
        size_t pos = filepath.rfind("/");
        std::string result = filepath;
        result.resize(pos);
        return result;
    }

    /*
    bool createDirectoryForFile(const std::string& filepath) const {
        std::vector<
        std::string dir = extractDirName(filepath);
        if (mkdir(dir.c_str(), 0744) == -1) {
            std::cout<< "Error :  " << strerror(errno) << std::endl;
        }
        else {
            std::cout << "Directory created" << std::endl; 
        }
    }
    */
    WorkspaceTool()
        : nh_("workspace_tool")
        , mp_(nh_, "/velma/current_reachability")
        , robot_interface_loader_("rcprg_planner", "rcprg_planner::RobotInterface")
    {

        nh_.getParam("robot_interface_plugin", robot_interface_plugin_str_);
        if (robot_interface_plugin_str_.empty()) {
            ROS_ERROR("The ROS parameter \"robot_interface_plugin\" is empty");
            throw std::invalid_argument("The ROS parameter \"robot_interface_plugin\" is empty");
        }
        ROS_INFO("Trying to load plugin: \"%s\"", robot_interface_plugin_str_.c_str());

        //try
        //{
        // If this fails, there is nothing more to do
            robot_interface_ = robot_interface_loader_.createInstance(robot_interface_plugin_str_);
            ROS_INFO("Loaded plugin: \"%s\"", robot_interface_plugin_str_.c_str());
            //ROS_INFO("loaded velma_ros_plugin::VelmaInterface");
        //}
        //catch(pluginlib::PluginlibException& ex)
        //{
        //    //handle the class failing to load
        //    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        //    return;
        //}

        nh_.getParam("map_filepath", map_filepath_);
        if (map_filepath_.empty()) {
            ROS_ERROR("The ROS parameter \"map_filepath\" is empty");
            throw std::invalid_argument("The ROS parameter \"map_filepath\" is empty");
        }

        getUniformDirections(35.0/180.0*PI, directions_);
        std::cout << "Using directions: " << directions_.size() << std::endl;

        nh_.getParam("/robot_description", robot_description_str_);
        nh_.getParam("/robot_description_semantic", robot_description_semantic_str_);

        std::string xml_out;
        self_collision::CollisionModel::convertSelfCollisionsInURDF(robot_description_str_, xml_out);

        //nh_.getParam("/velma_core_cs/JntLimit/limits_1", wcc_l_constraint_polygon_);

        //
        // moveit
        //
        robot_model_loader::RobotModelLoader robot_model_loader( robot_model_loader::RobotModelLoader::Options(xml_out, robot_description_semantic_str_) );
        robot_model_ = robot_model_loader.getModel();

        planning_scene_.reset( new planning_scene::PlanningScene(robot_model_) );

        planning_scene_->setStateFeasibilityPredicate( boost::bind(&RobotInterface::isStateValid, robot_interface_.get(), _1, _2) );

        planning_pipeline_.reset( new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters") );
    }

    bool initialize() {
        //const std::string rr_filename = "/home/dseredyn/ws_velma_2019_11/ws_velma_os/rr_map_new_medium";
        //rr_ = createModel();
        //saveModel(rr_filename, rr);

        //const std::string rr_saved_filename =
        //        "/home/dseredyn/ws_velma_2019_11/ws_velma_os/rr_map_new_big";
        //std::cout << "Loading ReachabilityRange from file \"" << rr_saved_filename << "\"..."
        //                                                                            << std::endl;

        if (canReadFile(map_filepath_)) {
            std::cout << "Loading ReachabilityRange from file \""
                        << map_filepath_ << "\"" << std::endl;
            rr_ = loadModel(map_filepath_);
            std::cout << "Loaded ReachabilityRange, number of samples: "
                        << rr_->getSamplesCount() << std::endl;
        }
        else {
            std::cout << "There is no file \""
                        << map_filepath_ << "\", need to compute the map" << std::endl;

            //if (!canWriteFile(map_filepath_)) {
            //    std::cout << "Trying to create directory..." << std::endl;
            //    createDirectoryForFile(map_filepath_);
            //}
            if (canWriteFile(map_filepath_)) {
                rr_ = createModel();
                saveModel(map_filepath_, rr_);
            }
            else {
                std::cout << "ERROR: The file \""
                            << map_filepath_ << "\" cannot be written" << std::endl;
                std::cout << "You need to create the full path for file " << map_filepath_
                                                                                << std::endl;
                return false;
                //throw std::invalid_argument("Could not write map file");
            }
        }
        // Everything is initialized, so start services and spinning
        service_reacahbility_range_ = nh_.advertiseService("reachability_range",
                                                &WorkspaceTool::reachabilityRangeService, this);
        return true;
    }

    void saveData(const std::string& filename, const unsigned char* buf, int size) {
        FILE *f = fopen(filename.c_str(), "wb");
        fwrite(buf, size, 1, f);
        fclose(f);
    }

    unsigned char* readData(const std::string& filename) {
        FILE *f = fopen(filename.c_str(), "rb");

        // obtain file size:
        fseek (f, 0 , SEEK_END);
        int lSize = ftell(f);
        rewind (f);

        unsigned char* buf = new unsigned char[lSize];

        fread(buf, lSize, 1, f);
        fclose(f);
        return buf;
    }

    void saveModel(const std::string& filename, ReachabilityRangePtr rr) {
        int64_t size = int64_t( rr->serialize(0, false) );
        unsigned char* buf = new unsigned char[size];
        rr->serialize(buf, true);
        saveData(filename, buf, size);
        delete[] buf;
    }

    ReachabilityRangePtr loadModel(const std::string& filename) {
        unsigned char* buf = readData(filename);
        ReachabilityRangePtr rr;
        ReachabilityRange::deserialize(rr, buf);
        delete[] buf;
        return rr;
    }

    void visualizeReachabilityRange(const ReachabilityRangePtr& rr) {
        // Visualize
        DiscreteSE3::Index index_se3 = rr->getSE3().getZeroIndex();
        std::vector<KDL::Vector > pts;
        std::vector<KDL::Vector > pts2;
        do {
            const ReachabilityRange::Cell& cell = rr->getCellAtIndex(index_se3);
            if (cell.getSamplesCount() > 0) {
                pts.push_back( rr->getSE3().getIndexValue(index_se3) );
            }
            for (int i = 0; i < cell.getSamplesCount(); ++i) {
                pts2.push_back( cell.getSamples()[i].T.p );
            }
            index_se3 = rr->getSE3().getNextIndex(index_se3);
        } while (!rr->getSE3().isZeroIndex(index_se3));
        // visual marker
        int m_id = 0;
        m_id = mp_.addSphereListMarker(m_id, pts, 1, 0, 0, 0.3, 0.03, "world");
        m_id = mp_.addSphereListMarker(m_id, pts2, 0, 1, 0, 1, 0.01, "world");

        mp_.publish();
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        std::lock_guard<std::mutex> guard(current_joint_states_ptr_mutex_);
        current_joint_states_ptr_ = msg;
    }

    sensor_msgs::JointState getCurrentJointState() {
        std::lock_guard<std::mutex> guard(current_joint_states_ptr_mutex_);
        if (current_joint_states_ptr_) {
            return *current_joint_states_ptr_;
        }
        return sensor_msgs::JointState();
    }

    void showReachabilityRange() {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

        const std::array<std::string, 7 > joint_names_right({
                "right_arm_0_joint", "right_arm_1_joint", "right_arm_2_joint", "right_arm_3_joint",
                "right_arm_4_joint", "right_arm_5_joint", "right_arm_6_joint"});
        const std::array<std::string, 7 > joint_names_left({
                "left_arm_0_joint", "left_arm_1_joint", "left_arm_2_joint", "left_arm_3_joint",
                "left_arm_4_joint", "left_arm_5_joint", "left_arm_6_joint"});

        ros::Subscriber sub = nh_.subscribe("/joint_states", 1000,
                                                        &WorkspaceTool::jointStateCallback, this);

        const std::array<std::string, 2 > end_effector_names({"right_arm_7_link",
                                                                "left_arm_7_link"});
        const std::array<ReachabilityRange::arm_side, 2> arm_sides({ReachabilityRange::ARM_R,
                                                                    ReachabilityRange::ARM_L});
        const std::array<std::array<std::string, 7 >, 2 > joint_names({joint_names_right,
                                                                        joint_names_left});

        LinearIntervalFunction func_size({0, 3, 20000}, {0.04, 0.00001, 0.00001});
        bool is_publishing = false;
        while (ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();

            if (mp_.getNumSubscribers() == 0) {
                if (is_publishing) {
                    std::cout << "no subscribers - stopping publishing" << std::endl;
                    is_publishing = false;
                }
                continue;
            }
            else {
                if (!is_publishing) {
                    std::cout << "detected subscribers - starting publishing" << std::endl;
                    is_publishing = true;
                }
            }

            int m_id = 0;
            for (int arm_idx = 0; arm_idx < end_effector_names.size(); ++arm_idx) {

                try{
                  transformStamped = tfBuffer.lookupTransform("torso_link0",
                                                    end_effector_names[arm_idx], ros::Time(0));
                }
                catch (tf2::TransformException &ex) {
                  ROS_WARN("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  continue;
                }
                KDL::Frame T_T0_Wr;
                T_T0_Wr = tf2::transformToKDL(transformStamped);

                sensor_msgs::JointState js = getCurrentJointState();
                DiscreteNSpace::Value q_current( joint_names[arm_idx].size() );
                for (int jnt_idx = 0; jnt_idx < joint_names[arm_idx].size(); ++jnt_idx) {
                    q_current[jnt_idx] = 0;
                    for (int i = 0; i < js.name.size(); ++i) {
                        if (js.name[i] == joint_names[arm_idx][jnt_idx]) {
                            q_current[jnt_idx] = js.position[i];
                        }
                    }
                    
                }

                for (int i = 0; i < directions_.size(); ++i) {
                    for (double dist = 0.04; dist < 0.2; dist += 0.04) {
                        KDL::Vector offset = directions_[i] * dist;
                        double match_dist = rr_->getMatchDistQ( q_current,
                                KDL::Frame(T_T0_Wr.M, T_T0_Wr.p+offset ), arm_sides[arm_idx] );
                        //double match_dist = rr->getMatchDistT( KDL::Frame(T_T0_Wr.M, T_T0_Wr.p+offset ));
                        double match_val = func_size.interpolate(match_dist);
                        m_id = mp_.addSinglePointMarker(m_id, T_T0_Wr.p+offset, 0, 1, 0, 0.5,
                                                                            match_val, "torso_link0");
                    }
                }
            }
            //std::cout << T_T0_Wr.p.x() << ", " << T_T0_Wr.p.y() << ", " << T_T0_Wr.p.z() << ": " << match_dist << " " << match_val << std::endl;
            mp_.publish();
        }

    }

    bool reachabilityRangeService(rcprg_planner_msgs::ReachabilityRange::Request &req,
                rcprg_planner_msgs::ReachabilityRange::Response &res) {
        if (req.frame_id != std::string("torso_base")) {
            res.success = false;
            res.message = "frame_id must be \"torso_base\"";
            return true;
        }

        ReachabilityRange::arm_side side;
        if (req.arm_side == rcprg_planner_msgs::ReachabilityRange::Request::ARM_R) {
            side = ReachabilityRange::ARM_R;
        }
        else if (req.arm_side == rcprg_planner_msgs::ReachabilityRange::Request::ARM_L) {
            side = ReachabilityRange::ARM_L;
        }
        else {
            res.success = false;
            res.message = "arm_side must be either \"ARM_R\" or \"ARM_L\"";
            return true;
        }

        moveit::core::RobotState ss(robot_model_);
        ss.setToDefaultValues();
        for (int query_idx = 0; query_idx < req.queries.size(); ++query_idx) {
            // Transform pose from the base frame with rotated torso
            ss.setVariablePosition("torso_0_joint", req.queries[query_idx].torso_q);
            ss.update();
            KDL::Frame T_B_T0d;
            tf::transformEigenToKDL( ss.getGlobalLinkTransform("torso_link0"), T_B_T0d);

            KDL::Frame req_T_B_W = pose2KDL( req.queries[query_idx].T_B_W );    

            KDL::Frame T_T0_Wr = T_B_T0d.Inverse() * req_T_B_W;

            double max_match_dist = 0.0;
            for (int i = 0; i < directions_.size(); ++i) {
                KDL::Vector offset = directions_[i] * 0.1;
                double match_dist = rr_->getMatchDistT( KDL::Frame(T_T0_Wr.M, T_T0_Wr.p+offset ),
                                                                                            side);
                if (match_dist > max_match_dist) {
                    max_match_dist = match_dist;
                }
            }
            //std::cout << "reachabilityRangeService: " << max_match_dist << std::endl;
            res.match_dist.push_back( max_match_dist );
        }
        res.success = true;
        //res.sum = req.a + req.b;
        //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
        //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
        return true;
    }

    ~WorkspaceTool() {
    }

    void spin() {
        showReachabilityRange();
    }
};

}; // namespace rcprg_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "rcprg_workspace_tool");
    rcprg_planner::WorkspaceTool workspace_tool;
    if (!workspace_tool.initialize()) {
        std::cout << "ERROR: Could not initialize the WorkspaceTool" << std::endl;
        return 1;
    }
    workspace_tool.spin();
    return 0;
}
