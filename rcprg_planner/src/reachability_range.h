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

#ifndef __REACHABILITY_RANGE_H__
#define __REACHABILITY_RANGE_H__

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

typedef std::array<double, 7 > arr_double7;
typedef std::vector<double > vec_double;
typedef std::array<int, 7 > arr_int7;

class DiscreteNSpace;
typedef std::shared_ptr<DiscreteNSpace > DiscreteNSpacePtr;
class DiscreteNSpace {
private:
    int dim_;
    std::vector<vec_double > slices_;
public:
    typedef std::vector<int> Index;
    typedef std::vector<double> Value;

    DiscreteNSpace(int dim);

    void setSlice(int slice_idx, const vec_double& values);

    int getSpaceSize() const;

    int getSlicesCount() const;

    vec_double getSlice(int slice_idx) const;

    Value getIndexValue(const Index& index) const;

    Index getValueIndex(const Value& val) const;

    Index getNextIndex(const Index& index) const;

    bool isZeroIndex(const Index& index) const;

    Index getZeroIndex() const;

    unsigned char* serialize(unsigned char* buf, bool write) const;

    static unsigned char* deserialize(DiscreteNSpacePtr& result, unsigned char* buf);
};

class BoundingBox;
typedef std::shared_ptr<BoundingBox > BoundingBoxPtr;

class BoundingBox {
public:
    KDL::Vector min;
    KDL::Vector max;

    BoundingBox(double xmin, double ymin, double zmin,
                double xmax, double ymax, double zmax);

    BoundingBox(const KDL::Vector& _min, const KDL::Vector& _max);

    bool isIn(const KDL::Vector& p) const;

    unsigned char* serialize(unsigned char* buf, bool write) const;

    static unsigned char* deserialize(BoundingBoxPtr& result, unsigned char* buf);
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
    DiscreteSE3(const BoundingBox& bb, double cell_size);

    Index getCount() const;

    bool getPosIndex(const KDL::Vector& p, Index& result) const;

    KDL::Vector getIndexValue(const Index& index) const;

    Index getNextIndex(const Index& index) const;

    bool isZeroIndex(const Index& index) const;

    Index getZeroIndex() const;

    unsigned char* serialize(unsigned char* buf, bool write) const;

    static unsigned char* deserialize(DiscreteSE3Ptr& result, unsigned char* buf);
};

class ReachabilityRange;
typedef std::shared_ptr<ReachabilityRange > ReachabilityRangePtr;

class ReachabilityRange {
public:
    class Sample {
    public:
        KDL::Frame T;
        DiscreteNSpace::Index q;

        Sample() {}

        Sample(const DiscreteNSpace::Index& _q, const KDL::Frame& _T);

        unsigned char* serialize(unsigned char* buf, bool write) const;

        static unsigned char* deserialize(Sample& result, unsigned char* buf);
    };

    class Cell {
    private:
        std::vector<Sample > samples_;
    public:
        void addSample(const DiscreteNSpace::Index& q, const KDL::Frame& T);

        int getSamplesCount() const;

        const std::vector<Sample >& getSamples() const;
    };
private:

    DiscreteNSpace dspace_conf_;
    DiscreteSE3 dspace_se3_;
    DiscreteSE3::Index se3_size_;
    std::vector<Cell > cells_;
    Cell null_cell_;
    int total_samples_;

    int idx2off(const DiscreteSE3::Index& index) const;
public:
    enum arm_side {ARM_R, ARM_L};

    ReachabilityRange(const DiscreteNSpace& dspace_conf, const DiscreteSE3& dspace_se3);

    void addPoint(const DiscreteNSpace::Index& q, const KDL::Frame& T);

    void addPoint(const Sample& sample);

    const Cell& getCellAtIndex(const DiscreteSE3::Index& index) const;

    const Cell& getCellAtPos(const KDL::Vector& pos) const;

    double getMatchDistQ(const DiscreteNSpace::Value& _q, const KDL::Frame& T_T0_W, arm_side side) const;

    double getMatchDistT(const KDL::Frame& T_T0_W, arm_side side) const;

    int getSamplesCount() const;

    const DiscreteSE3& getSE3() const;

    unsigned char* serialize(unsigned char* buf, bool write) const;

    static unsigned char* deserialize(ReachabilityRangePtr& result, unsigned char* buf);
};
};  // namespace rcprg_planner

#endif  // __REACHABILITY_RANGE_H__
