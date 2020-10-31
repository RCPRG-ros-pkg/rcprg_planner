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

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
//#include "ros/package.h"
#include <sensor_msgs/JointState.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <interactive_markers/interactive_marker_server.h>

//#include "kdl_conversions/kdl_msg.h"

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
#include <eigen_conversions/eigen_kdl.h>

#include <string>
//#include <stdlib.h>
#include <stdio.h>

//#include "Eigen/Dense"
//#include "Eigen/LU"

#include <collision_convex_model/collision_convex_model.h>
#include "kin_dyn_model/kin_model.h"
//#include "planer_utils/random_uniform.h"
//#include "planer_utils/utilities.h"
//#include "planer_utils/double_joint_collision_checker.h"
#include "std_srvs/Trigger.h"

// plugin for robot interface
#include <pluginlib/class_loader.h>
#include <rcprg_planner/robot_interface.h>

#include <rcprg_ros_utils/marker_publisher.h>

#include <rcprg_planner_msgs/ReachabilityRange.h>

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

unsigned char* serialize_double(double d, unsigned char* buf, bool write) {
    if (write) {
        *((double*)buf) = d;
    }
    buf += sizeof(double);
    return buf;
}

unsigned char* deserialize_double(double& d, unsigned char* buf) {
    d = *((double*)buf);
    buf += sizeof(double);
    return buf;
}

unsigned char* serialize_int(int i, unsigned char* buf, bool write) {
    if (write) {
        *((int*)buf) = i;
    }
    buf += sizeof(int);
    return buf;
}

unsigned char* deserialize_int(int& d, unsigned char* buf) {
    d = *((int*)buf);
    buf += sizeof(int);
    return buf;
}

unsigned char* serialize_KDL_Rotation(const KDL::Rotation& M, unsigned char* buf, bool write) {
    double x, y, z, w;
    M.GetQuaternion(x, y, z, w);
    buf = serialize_double(x, buf, write);
    buf = serialize_double(y, buf, write);
    buf = serialize_double(z, buf, write);
    buf = serialize_double(w, buf, write);
    return buf;
}

unsigned char* deserialize_KDL_Rotation(KDL::Rotation& M, unsigned char* buf) {
    double x, y, z, w;
    buf = deserialize_double(x, buf);
    buf = deserialize_double(y, buf);
    buf = deserialize_double(z, buf);
    buf = deserialize_double(w, buf);
    M = KDL::Rotation::Quaternion(x, y, z, w);
    return buf;
}

unsigned char* serialize_KDL_Vector(const KDL::Vector& v, unsigned char* buf, bool write) {
    buf = serialize_double(v.x(), buf, write);
    buf = serialize_double(v.y(), buf, write);
    buf = serialize_double(v.z(), buf, write);
    return buf;
}

unsigned char* deserialize_KDL_Vector(KDL::Vector& v, unsigned char* buf) {
    double x, y, z;
    buf = deserialize_double(x, buf);
    buf = deserialize_double(y, buf);
    buf = deserialize_double(z, buf);
    v = KDL::Vector(x, y, z);
    return buf;
}

unsigned char* serialize_KDL_Frame(const KDL::Frame& T, unsigned char* buf, bool write) {
    buf = serialize_KDL_Vector(T.p, buf, write);
    buf = serialize_KDL_Rotation(T.M, buf, write);
    return buf;
}

unsigned char* deserialize_KDL_Frame(KDL::Frame& T, unsigned char* buf) {
    buf = deserialize_KDL_Vector(T.p, buf);
    buf = deserialize_KDL_Rotation(T.M, buf);
    return buf;
}

unsigned char* serialize_arr_int7(const arr_int7& a, unsigned char* buf, bool write) {
    for (int i = 0; i < 7; ++i) {
        buf = serialize_int(a[i], buf, write);
    }
    return buf;
}

unsigned char* serialize_vec_int(const std::vector<int >& v, unsigned char* buf, bool write) {
    buf = serialize_int(v.size(), buf, write);
    for (int i = 0; i < v.size(); ++i) {
        buf = serialize_int(v[i], buf, write);
    }
    return buf;
}

unsigned char* deserialize_vec_int(std::vector<int >& v, unsigned char* buf) {
    int size;
    buf = deserialize_int(size, buf);
    v.resize(size);
    for (int i = 0; i < size; ++i) {
        int val;
        buf = deserialize_int(val, buf);
        v[i] = val;
    }
    return buf;
}

class DiscreteNSpace;
typedef std::shared_ptr<DiscreteNSpace > DiscreteNSpacePtr;
class DiscreteNSpace {
private:
    int dim_;
    std::vector<vec_double > slices_;
public:
    typedef std::vector<int> Index;
    typedef std::vector<double> Value;

    DiscreteNSpace(int dim)
    : dim_(dim)
    {
        slices_.resize(dim_);
    }

    void setSlice(int slice_idx, const vec_double& values) {
        slices_[slice_idx] = values;
    }

    int getSpaceSize() const {
        int result = 1;
        for (int i = 0; i < dim_; ++i) {
            result *= slices_[i].size();
        }
        return result;
    }

    int getSlicesCount() const {
        return dim_;
    }

    vec_double getSlice(int slice_idx) const {
        return slices_[slice_idx];
    }

    Value getIndexValue(const Index& index) const {
        Value result(dim_);
        for (int i = 0; i < dim_; ++i) {
            result[i] = slices_[i][index[i]];
        }
        return result;
    }

    Index getValueIndex(const Value& val) const {
        Index result(dim_);
        for (int slice_idx = 0; slice_idx < dim_; ++slice_idx) {
            double min_dist = 100000.0;
            for (int idx = 0; idx < slices_[slice_idx].size(); ++idx) {
                double dist = fabs( slices_[slice_idx][idx] - val[slice_idx] );
                if (dist < min_dist) {
                    min_dist = dist;
                    result[slice_idx] = idx;
                }
            }
        }
        return result;
    }

    Index getNextIndex(const Index& index) const {
        Index result(index);
        for (int i = 0; i < dim_; ++i) {
            ++result[i];
            if (result[i] >= slices_[i].size()) {
                result[i] = 0;
            }
            else {
                break;
            }
        }
        return result;
    }

    bool isZeroIndex(const Index& index) const {
        for (int i = 0; i < dim_; ++i) {
            if (index[i] != 0) {
                return false;
            }
        }
        return true;
    }

    Index getZeroIndex() const {
        return Index(dim_, 0);
    }

    unsigned char* serialize(unsigned char* buf, bool write) const {
        buf = serialize_int( dim_, buf, write );
        for (int slice_idx = 0; slice_idx < dim_; ++slice_idx) {
            buf = serialize_int( slices_[slice_idx].size(), buf, write );
            for (int idx = 0; idx < slices_[slice_idx].size(); ++idx) {
                buf = serialize_double( slices_[slice_idx][idx], buf, write );
            }            
        }
        return buf;
    }

    static unsigned char* deserialize(DiscreteNSpacePtr& result, unsigned char* buf) {
        int dim;
        buf = deserialize_int( dim, buf );
        result.reset(new DiscreteNSpace(dim));
        for (int slice_idx = 0; slice_idx < dim; ++slice_idx) {
            int slice_size;
            buf = deserialize_int( slice_size, buf );
            vec_double values;
            for (int idx = 0; idx < slice_size; ++idx) {
                double val;
                buf = deserialize_double( val, buf );
                values.push_back( val );
            }
            result->setSlice(slice_idx, values) ;
        }
        return buf;
    }

};

class BoundingBox;
typedef std::shared_ptr<BoundingBox > BoundingBoxPtr;

class BoundingBox {
public:
    KDL::Vector min;
    KDL::Vector max;

    BoundingBox(double xmin, double ymin, double zmin,
                double xmax, double ymax, double zmax)
    : min( xmin, ymin, zmin )
    , max( xmax, ymax, zmax )
    {}

    BoundingBox(const KDL::Vector& _min, const KDL::Vector& _max)
    : min( _min )
    , max( _max )
    {}

    bool isIn(const KDL::Vector& p) {
        return  p.x() >= min.x() && p.x() <= max.x() &&
                p.y() >= min.y() && p.y() <= max.y() &&
                p.z() >= min.z() && p.z() <= max.z();
    }

    unsigned char* serialize(unsigned char* buf, bool write) const {
        buf = serialize_KDL_Vector(min, buf, write);
        buf = serialize_KDL_Vector(max, buf, write);
        return buf;
    }

    static unsigned char* deserialize(BoundingBoxPtr& result, unsigned char* buf) {
        KDL::Vector _min, _max;
        buf = deserialize_KDL_Vector(_min, buf);
        buf = deserialize_KDL_Vector(_max, buf);
        result.reset(new BoundingBox(_min, _max));
        return buf;
    }
};

class DiscreteSE3;
typedef std::shared_ptr<DiscreteSE3 > DiscreteSE3Ptr;

class DiscreteSE3 {
public:
    typedef std::array<int, 3> Index;

private:
    BoundingBox bb_;
    double cell_size_;
    Index max_index_;
public:
    DiscreteSE3(const BoundingBox& bb, double cell_size)
    : bb_(bb)
    , cell_size_(cell_size)
    {
        KDL::Vector v = bb_.max - bb_.min;
        max_index_ = Index({
            int( ceil( v.x()/cell_size_ ) ),
            int( ceil( v.y()/cell_size_ ) ),
            int( ceil( v.z()/cell_size_ ) )});
    }

    Index getCount() const {
        return Index( {max_index_[0]+1, max_index_[1]+1, max_index_[2]+1} );
    }

    bool getPosIndex(const KDL::Vector& p, Index& result) const {
        KDL::Vector v = (p - bb_.min);
        int ix = int( floor( v.x()/cell_size_ ) );
        int iy = int( floor( v.y()/cell_size_ ) );
        int iz = int( floor( v.z()/cell_size_ ) );
        if (ix >= 0 && ix < max_index_[0] &&
                iy >= 0 && iy < max_index_[1] &&
                iz >= 0 && iz < max_index_[2]) {
            result = Index({ix, iy, iz});
            return true;
        }
        return false;
    }

    KDL::Vector getIndexValue(const Index& index) const {
        return bb_.min + KDL::Vector(
            double(index[0])+0.5, double(index[1])+0.5, double(index[2])+0.5) * cell_size_;
    }

    Index getNextIndex(const Index& index) const {
        Index result(index);
        for (int i = 0; i < 3; ++i) {
            ++result[i];
            if (result[i] > max_index_[i]) {
                result[i] = 0;
            }
            else {
                break;
            }
        }
        return result;
    }

    bool isZeroIndex(const Index& index) const {
        for (int i = 0; i < 3; ++i) {
            if (index[i] != 0) {
                return false;
            }
        }
        return true;
    }

    Index getZeroIndex() const {
        Index result;
        for (int i = 0; i < 3; ++i) {
            result[i] = 0;
        }
        return result;
    }

    unsigned char* serialize(unsigned char* buf, bool write) const {
        buf = bb_.serialize(buf, write);
        buf = serialize_double( cell_size_, buf, write );
        return buf;
    }

    static unsigned char* deserialize(DiscreteSE3Ptr& result, unsigned char* buf) {
        BoundingBoxPtr bb;
        buf = BoundingBox::deserialize(bb, buf);
        double cell_size;
        buf = deserialize_double(cell_size, buf);
        result.reset(new DiscreteSE3(*bb, cell_size));
        return buf;
    }

};

/*
\  0  1  2
 ---------
0| 0  1  2
1| 3  4  5
2| 6  7  8
3| 9 10 11
size = [3,4]
idx = [1,2]
offs = 1 + 2*3
*/

class ReachabilityRange;
typedef std::shared_ptr<ReachabilityRange > ReachabilityRangePtr;

class ReachabilityRange {
public:
    class Sample {
    public:
        KDL::Frame T;
        DiscreteNSpace::Index q;

        Sample() {}

        Sample(const DiscreteNSpace::Index& _q, const KDL::Frame& _T)
        : T(_T)
        , q(_q)
        {}

        unsigned char* serialize(unsigned char* buf, bool write) const {
            buf = serialize_KDL_Frame(T, buf, write);
            buf = serialize_vec_int(q, buf, write);
            return buf;
        }

        static unsigned char* deserialize(Sample& result, unsigned char* buf) {
            buf = deserialize_KDL_Frame(result.T, buf);
            buf = deserialize_vec_int(result.q, buf);
            return buf;
        }
    };

    class Cell {
    private:
        std::vector<Sample > samples_;
    public:
        void addSample(const DiscreteNSpace::Index& q, const KDL::Frame& T) {
            samples_.push_back( Sample(q, T) );
        }

        int getSamplesCount() const {
            return samples_.size();
        }

        const std::vector<Sample >& getSamples() const {
            return samples_;
        }
    };
private:

    DiscreteNSpace dspace_conf_;
    DiscreteSE3 dspace_se3_;
    DiscreteSE3::Index se3_size_;
    std::vector<Cell > cells_;
    Cell null_cell_;
    int total_samples_;

    int idx2off(const DiscreteSE3::Index& index) const {
        return index[0] + se3_size_[0] * (index[1] + se3_size_[1]*index[2]);
    }
public:
    ReachabilityRange(const DiscreteNSpace& dspace_conf, const DiscreteSE3& dspace_se3)
    : dspace_conf_( dspace_conf )
    , dspace_se3_( dspace_se3 )
    , se3_size_( dspace_se3.getCount() )
    , total_samples_(0)
    {
        cells_.resize( se3_size_[0] * se3_size_[1] * se3_size_[2] );
    }

    void addPoint(const DiscreteNSpace::Index& q, const KDL::Frame& T) {
        DiscreteSE3::Index index;
        if (dspace_se3_.getPosIndex( T.p, index )) {
            cells_[idx2off(index)].addSample(q, T);
            ++total_samples_;
        }
    }

    void addPoint(const Sample& sample) {
        this->addPoint(sample.q, sample.T);
    }

    const Cell& getCellAtIndex(const DiscreteSE3::Index& index) const {
        return cells_[idx2off(index)];
    }

    const Cell& getCellAtPos(const KDL::Vector& pos) const {
        DiscreteSE3::Index index;
        if (!dspace_se3_.getPosIndex(pos, index)) {
            return null_cell_;
        }
        return this->getCellAtIndex(index);
    }

    double getMatchDistQ(const DiscreteNSpace::Value& _q, const KDL::Frame& T) {
        const Cell& cell = this->getCellAtPos(T.p);
        double min_dist = 10000.0;
        for (int i = 0; i < cell.getSamplesCount(); ++i) {
            //KDL::Twist twist = KDL::diff(T, cell.getSamples()[i].T, 1.0);
            double q_dist = 0;
            DiscreteNSpace::Value q_cell = dspace_conf_.getIndexValue( cell.getSamples()[i].q );
            for (int j = 0; j < _q.size(); ++j) {
                q_dist += (q_cell[j]-_q[j])*(q_cell[j]-_q[j]);
            }
            //double dist = twist.rot.Norm() + twist.vel.Norm()*10.0 + sqrt(q_dist);
            double dist = sqrt(q_dist);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    double getMatchDistT(const KDL::Frame& T) {
        const Cell& cell = this->getCellAtPos(T.p);
        double min_dist = 10000.0;
        for (int i = 0; i < cell.getSamplesCount(); ++i) {
            KDL::Twist twist = KDL::diff(T, cell.getSamples()[i].T, 1.0);
            double dist = twist.rot.Norm() + twist.vel.Norm()*10.0;
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        return min_dist;
    }

    int getSamplesCount() const {
        return total_samples_;
    }

    const DiscreteSE3& getSE3() const {
        return this->dspace_se3_;
    }

    unsigned char* serialize(unsigned char* buf, bool write) const {
        buf = dspace_conf_.serialize(buf, write);
        buf = dspace_se3_.serialize(buf, write);
        buf = serialize_int( this->getSamplesCount(), buf, write );
        for (int cell_idx = 0; cell_idx < cells_.size(); ++cell_idx) {
            for (int sample_idx = 0; sample_idx < cells_[cell_idx].getSamplesCount();
                                                                                ++sample_idx) {
                buf = cells_[cell_idx].getSamples()[sample_idx].serialize(buf, write);
            }
        }
        return buf;
    }

    static unsigned char* deserialize(ReachabilityRangePtr& result, unsigned char* buf) {
        DiscreteNSpacePtr dspace_conf;
        buf = DiscreteNSpace::deserialize(dspace_conf, buf);
        DiscreteSE3Ptr dspace_se3;
        buf = DiscreteSE3::deserialize(dspace_se3, buf);
        int samples_count;
        buf = deserialize_int( samples_count, buf );
        result.reset(new ReachabilityRange(*dspace_conf, *dspace_se3));
        for (int sample_idx = 0; sample_idx < samples_count; ++sample_idx) {
            Sample sample;
            buf = Sample::deserialize( sample, buf );
            result->addPoint( sample );
        }
        return buf;
    }
};

class WorkspaceTool {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_reset_;
    ros::ServiceServer service_plan_;
    ros::ServiceServer service_processWorld_;

    MarkerPublisher mp_;

    ReachabilityRangePtr rr_;
    //const double PI;

    //KDL::Frame int_marker_pose_;

    std::string robot_description_str_;
    std::string robot_semantic_description_str_;

    //boost::shared_ptr<self_collision::CollisionModel> col_model_;
    boost::shared_ptr<KinematicModel > kin_model_;
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

    sensor_msgs::JointState::ConstPtr current_joint_states_ptr_;
    std::mutex current_joint_states_ptr_mutex_;

    std::vector<KDL::Vector > directions_;

public:

    ReachabilityRangePtr createModel() {
        // Those limits are adjusted to cart_imp mode at default starting configuration
        arr_double7 limits_lo({-2.96,-2.09,-2.96,0.2,-2.96,-2.09,-2.96});
        arr_double7 limits_hi({2.96,-0.2,2.96,2.095,2.96,-0.2,2.96});

        std::vector<std::string > joint_names({"right_arm_0_joint", "right_arm_1_joint",
            "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
            "right_arm_6_joint"});

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

        int step = 0;
        do {
            DiscreteNSpace::Value val = dspace7.getIndexValue(index);
            for (int j = 0; j < joint_names.size(); ++j) {
                ss.setVariablePosition(joint_names[j], val[j]);
            }
            ss.update();
            KDL::Frame T_W_Ee;
            tf::transformEigenToKDL( ss.getGlobalLinkTransform("right_arm_7_link"), T_W_Ee);
            if ( bb.isIn(T_W_Ee.p) ) {
                if (planning_scene_->isStateValid(ss)) {
                    rr->addPoint(index, T_W_Ee);
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

    WorkspaceTool()
        : nh_("workspace_tool")
        , mp_(nh_)
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

        getUniformDirections(20.0/180.0*PI, directions_);
        std::cout << "Using directions: " << directions_.size() << std::endl;

        service_reset_ = nh_.advertiseService("reset", &WorkspaceTool::reset, this);
        service_plan_ = nh_.advertiseService("plan", &WorkspaceTool::plan, this);
        service_processWorld_ = nh_.advertiseService("processWorld", &WorkspaceTool::processWorld, this);

        nh_.getParam("/robot_description", robot_description_str_);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str_);

        std::string xml_out;
        self_collision::CollisionModel::convertSelfCollisionsInURDF(robot_description_str_, xml_out);

        //nh_.getParam("/velma_core_cs/JntLimit/limits_1", wcc_l_constraint_polygon_);

        //
        // moveit
        //
        robot_model_loader::RobotModelLoader robot_model_loader( robot_model_loader::RobotModelLoader::Options(xml_out, robot_semantic_description_str_) );
        robot_model_ = robot_model_loader.getModel();

        planning_scene_.reset( new planning_scene::PlanningScene(robot_model_) );

        planning_scene_->setStateFeasibilityPredicate( boost::bind(&RobotInterface::isStateValid, robot_interface_.get(), _1, _2) );

        planning_pipeline_.reset( new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters") );


        const std::string rr_filename = "/home/dseredyn/ws_velma_2019_11/ws_velma_os/rr_map_01";
        //rr_ = createModel();
        //saveModel(rr_filename, rr);

        const std::string rr_saved_filename =
                "/home/dseredyn/ws_velma_2019_11/ws_velma_os/rr_map_big";
        std::cout << "Loading ReachabilityRange from file \"" << rr_saved_filename << "\"..."
                                                                                    << std::endl;
        rr_ = loadModel(rr_saved_filename);
        std::cout << "Loaded ReachabilityRange, number of samples: " << rr_->getSamplesCount()
                                                                                    << std::endl;

        ros::ServiceServer service = nh_.advertiseService("reachability_range",
                                                &WorkspaceTool::reachabilityRangeService, this);

        testReachabilityRange(rr_);


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
            //for (int i = 0; i < valid_frames.size(); ++i) {
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
        return *current_joint_states_ptr_;
    }

    void testReachabilityRange(const ReachabilityRangePtr& rr) {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

        std::vector<std::string > joint_names({"right_arm_0_joint", "right_arm_1_joint",
                "right_arm_2_joint", "right_arm_3_joint", "right_arm_4_joint", "right_arm_5_joint",
                "right_arm_6_joint"});

        ros::Subscriber sub = nh_.subscribe("/joint_states", 1000,
                                                        &WorkspaceTool::jointStateCallback, this);

        while (ros::ok()) {
            ros::spinOnce();

            try{
              transformStamped = tfBuffer.lookupTransform("torso_base", "right_arm_7_link",
                                       ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
              ROS_WARN("%s",ex.what());
              ros::Duration(1.0).sleep();
              continue;
            }
            KDL::Frame T_B_Wr;
            T_B_Wr = tf2::transformToKDL(transformStamped);

            sensor_msgs::JointState js = getCurrentJointState();
            DiscreteNSpace::Value q_current( joint_names.size() );
            for (int jnt_idx = 0; jnt_idx < joint_names.size(); ++jnt_idx) {
                for (int i = 0; i < js.name.size(); ++i) {
                    if (js.name[i] == joint_names[jnt_idx]) {
                        q_current[jnt_idx] = js.position[i];
                    }
                }
                
            }

            //double match_dist = rr->getMatchDist(T_B_Wr);
            //double match_val = (10.0 - match_dist)/10.0;
            //if (match_val < 0) {
            //    match_val = 0.0;
            //}


            int m_id = 0;
            for (int i = 0; i < directions_.size(); ++i) {
                for (double dist = 0.04; dist < 0.2; dist += 0.04) {
                    KDL::Vector offset = directions_[i] * dist;
                    double match_dist = rr->getMatchDistQ( q_current, KDL::Frame(T_B_Wr.M, T_B_Wr.p+offset ) );
                    //double match_dist = rr->getMatchDistT( KDL::Frame(T_B_Wr.M, T_B_Wr.p+offset ));
                    double match_val = (1.0 - match_dist)/1.0;
                    if (match_val < 0.00001) {
                        match_val = 0.00001;
                    }
                    m_id = mp_.addSinglePointMarker(m_id, T_B_Wr.p+offset, 0, 1, 0, 0.5, match_val*0.02, "torso_base");
                }
            }
            //std::cout << T_B_Wr.p.x() << ", " << T_B_Wr.p.y() << ", " << T_B_Wr.p.z() << ": " << match_dist << " " << match_val << std::endl;
            mp_.publish();

            ros::Duration(0.1).sleep();
        }

    }

    bool reachabilityRangeService(rcprg_planner_msgs::ReachabilityRange::Request &req,
                rcprg_planner_msgs::ReachabilityRange::Response &res) {
        if (req.frame_id != std::string("torso_base")) {
            res.success = false;
            res.message = "frame_id must be \"torso_base\"";
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
            ss.setVariablePosition("torso_0_joint", 0.0);
            ss.update();
            KDL::Frame T_B_T0;
            tf::transformEigenToKDL( ss.getGlobalLinkTransform("torso_link0"), T_B_T0);

            KDL::Frame req_T_B_W = pose2KDL( req.queries[query_idx].T_B_W );    

            KDL::Frame T_B_Wr = T_B_T0 * T_B_T0d.Inverse() * req_T_B_W;

            double max_match_dist = 0.0;
            for (int i = 0; i < directions_.size(); ++i) {
                KDL::Vector offset = directions_[i] * 0.1;
                double match_dist = rr_->getMatchDistT( KDL::Frame(T_B_Wr.M, T_B_Wr.p+offset ));
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
    ros::init(argc, argv, "rcprg_workspace_tool");
    rcprg_planner::WorkspaceTool workspace_tool;
    workspace_tool.spin();
    return 0;
}


