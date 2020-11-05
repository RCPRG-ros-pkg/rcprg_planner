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

#include "reachability_range.h"

using namespace rcprg_planner;

static const double PI=3.14159265359;


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

DiscreteNSpace::DiscreteNSpace(int dim)
: dim_(dim)
{
    slices_.resize(dim_);
}

void DiscreteNSpace::setSlice(int slice_idx, const vec_double& values) {
    slices_[slice_idx] = values;
}

int DiscreteNSpace::getSpaceSize() const {
    int result = 1;
    for (int i = 0; i < dim_; ++i) {
        result *= slices_[i].size();
    }
    return result;
}

int DiscreteNSpace::getSlicesCount() const {
    return dim_;
}

vec_double DiscreteNSpace::getSlice(int slice_idx) const {
    return slices_[slice_idx];
}

DiscreteNSpace::Value DiscreteNSpace::getIndexValue(const DiscreteNSpace::Index& index) const {
    DiscreteNSpace::Value result(dim_);
    for (int i = 0; i < dim_; ++i) {
        result[i] = slices_[i][index[i]];
    }
    return result;
}

DiscreteNSpace::Index DiscreteNSpace::getValueIndex(const DiscreteNSpace::Value& val) const {
    DiscreteNSpace::Index result(dim_);
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

DiscreteNSpace::Index DiscreteNSpace::getNextIndex(const DiscreteNSpace::Index& index) const {
    DiscreteNSpace::Index result(index);
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

bool DiscreteNSpace::isZeroIndex(const DiscreteNSpace::Index& index) const {
    for (int i = 0; i < dim_; ++i) {
        if (index[i] != 0) {
            return false;
        }
    }
    return true;
}

DiscreteNSpace::Index DiscreteNSpace::getZeroIndex() const {
    return DiscreteNSpace::Index(dim_, 0);
}

unsigned char* DiscreteNSpace::serialize(unsigned char* buf, bool write) const {
    buf = serialize_int( dim_, buf, write );
    for (int slice_idx = 0; slice_idx < dim_; ++slice_idx) {
        buf = serialize_int( slices_[slice_idx].size(), buf, write );
        for (int idx = 0; idx < slices_[slice_idx].size(); ++idx) {
            buf = serialize_double( slices_[slice_idx][idx], buf, write );
        }            
    }
    return buf;
}

unsigned char* DiscreteNSpace::deserialize(DiscreteNSpacePtr& result, unsigned char* buf) {
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

BoundingBox::BoundingBox(double xmin, double ymin, double zmin,
            double xmax, double ymax, double zmax)
: min( xmin, ymin, zmin )
, max( xmax, ymax, zmax )
{}

BoundingBox::BoundingBox(const KDL::Vector& _min, const KDL::Vector& _max)
: min( _min )
, max( _max )
{}

bool BoundingBox::isIn(const KDL::Vector& p) const {
    return  p.x() >= min.x() && p.x() <= max.x() &&
            p.y() >= min.y() && p.y() <= max.y() &&
            p.z() >= min.z() && p.z() <= max.z();
}

unsigned char* BoundingBox::serialize(unsigned char* buf, bool write) const {
    buf = serialize_KDL_Vector(min, buf, write);
    buf = serialize_KDL_Vector(max, buf, write);
    return buf;
}

unsigned char* BoundingBox::deserialize(BoundingBoxPtr& result, unsigned char* buf) {
    KDL::Vector _min, _max;
    buf = deserialize_KDL_Vector(_min, buf);
    buf = deserialize_KDL_Vector(_max, buf);
    result.reset(new BoundingBox(_min, _max));
    return buf;
}


DiscreteSE3::DiscreteSE3(const BoundingBox& bb, double cell_size)
: bb_(bb)
, cell_size_(cell_size)
{
    KDL::Vector v = bb_.max - bb_.min;
    max_index_ = Index({
        int( ceil( v.x()/cell_size_ ) ),
        int( ceil( v.y()/cell_size_ ) ),
        int( ceil( v.z()/cell_size_ ) )});
}

DiscreteSE3::Index DiscreteSE3::getCount() const {
    return Index( {max_index_[0]+1, max_index_[1]+1, max_index_[2]+1} );
}

bool DiscreteSE3::getPosIndex(const KDL::Vector& p, DiscreteSE3::Index& result) const {
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

KDL::Vector DiscreteSE3::getIndexValue(const DiscreteSE3::Index& index) const {
    return bb_.min + KDL::Vector(
        double(index[0])+0.5, double(index[1])+0.5, double(index[2])+0.5) * cell_size_;
}

DiscreteSE3::Index DiscreteSE3::getNextIndex(const DiscreteSE3::Index& index) const {
    DiscreteSE3::Index result(index);
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

bool DiscreteSE3::isZeroIndex(const DiscreteSE3::Index& index) const {
    for (int i = 0; i < 3; ++i) {
        if (index[i] != 0) {
            return false;
        }
    }
    return true;
}

DiscreteSE3::Index DiscreteSE3::getZeroIndex() const {
    DiscreteSE3::Index result;
    for (int i = 0; i < 3; ++i) {
        result[i] = 0;
    }
    return result;
}

unsigned char* DiscreteSE3::serialize(unsigned char* buf, bool write) const {
    buf = bb_.serialize(buf, write);
    buf = serialize_double( cell_size_, buf, write );
    return buf;
}

unsigned char* DiscreteSE3::deserialize(DiscreteSE3Ptr& result, unsigned char* buf) {
    BoundingBoxPtr bb;
    buf = BoundingBox::deserialize(bb, buf);
    double cell_size;
    buf = deserialize_double(cell_size, buf);
    result.reset(new DiscreteSE3(*bb, cell_size));
    return buf;
}



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

ReachabilityRange::Sample::Sample(const DiscreteNSpace::Index& _q, const KDL::Frame& _T)
: T(_T)
, q(_q)
{}

unsigned char* ReachabilityRange::Sample::serialize(unsigned char* buf, bool write) const {
    buf = serialize_KDL_Frame(T, buf, write);
    buf = serialize_vec_int(q, buf, write);
    return buf;
}

unsigned char* ReachabilityRange::Sample::deserialize(Sample& result, unsigned char* buf) {
    buf = deserialize_KDL_Frame(result.T, buf);
    buf = deserialize_vec_int(result.q, buf);
    return buf;
}


void ReachabilityRange::Cell::addSample(const DiscreteNSpace::Index& q, const KDL::Frame& T) {
    samples_.push_back( Sample(q, T) );
}

int ReachabilityRange::Cell::getSamplesCount() const {
    return samples_.size();
}

const std::vector<ReachabilityRange::Sample >& ReachabilityRange::Cell::getSamples() const {
    return samples_;
}

ReachabilityRange::ReachabilityRange(const DiscreteNSpace& dspace_conf, const DiscreteSE3& dspace_se3)
: dspace_conf_( dspace_conf )
, dspace_se3_( dspace_se3 )
, se3_size_( dspace_se3.getCount() )
, total_samples_(0)
{
    cells_.resize( se3_size_[0] * se3_size_[1] * se3_size_[2] );
}

int ReachabilityRange::idx2off(const DiscreteSE3::Index& index) const {
    return index[0] + se3_size_[0] * (index[1] + se3_size_[1]*index[2]);
}

void ReachabilityRange::addPoint(const DiscreteNSpace::Index& q, const KDL::Frame& T) {
    DiscreteSE3::Index index;
    if (dspace_se3_.getPosIndex( T.p, index )) {
        cells_[idx2off(index)].addSample(q, T);
        ++total_samples_;
    }
}

void ReachabilityRange::addPoint(const Sample& sample) {
    this->addPoint(sample.q, sample.T);
}

const ReachabilityRange::Cell& ReachabilityRange::getCellAtIndex(const DiscreteSE3::Index& index) const {
    return cells_[idx2off(index)];
}

const ReachabilityRange::Cell& ReachabilityRange::getCellAtPos(const KDL::Vector& pos) const {
    DiscreteSE3::Index index;
    if (!dspace_se3_.getPosIndex(pos, index)) {
        return null_cell_;
    }
    return this->getCellAtIndex(index);
}

double ReachabilityRange::getMatchDistQ(const DiscreteNSpace::Value& _q, const KDL::Frame& T_T0_W, arm_side side) const {
    double side_factor;
    KDL::Frame T_T0_W_side;
    if (side == ARM_R) {
        T_T0_W_side = T_T0_W;
        side_factor = 1;
    }
    else {
        T_T0_W_side = KDL::Frame(KDL::Rotation::RotX(PI)*T_T0_W.M,
                                    KDL::Vector(T_T0_W.p.x(), -T_T0_W.p.y(), T_T0_W.p.z()));
        side_factor = -1;
    }

    const Cell& cell = this->getCellAtPos(T_T0_W_side.p);
    double min_dist = 10000.0;
    for (int i = 0; i < cell.getSamplesCount(); ++i) {
        //KDL::Twist twist = KDL::diff(T, cell.getSamples()[i].T, 1.0);
        double q_dist = 0;
        DiscreteNSpace::Value q_cell = dspace_conf_.getIndexValue( cell.getSamples()[i].q );
        for (int j = 0; j < _q.size(); ++j) {
            q_dist += (q_cell[j]-side_factor*_q[j])*(q_cell[j]-side_factor*_q[j]);
        }
        //double dist = twist.rot.Norm() + twist.vel.Norm()*10.0 + sqrt(q_dist);
        double dist = sqrt(q_dist);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

double ReachabilityRange::getMatchDistT(const KDL::Frame& T_T0_W, arm_side side) const {
    KDL::Frame T_T0_W_side;
    if (side == ARM_R) {
        T_T0_W_side = T_T0_W;
    }
    else {
        T_T0_W_side = KDL::Frame(KDL::Rotation::RotX(PI)*T_T0_W.M,
                                    KDL::Vector(T_T0_W.p.x(), -T_T0_W.p.y(), T_T0_W.p.z()));
    }

    const Cell& cell = this->getCellAtPos(T_T0_W_side.p);
    double min_dist = 10000.0;
    //std::cout << "getMatchDistT: samples: " << cell.getSamplesCount() << std::endl;
    for (int i = 0; i < cell.getSamplesCount(); ++i) {
        KDL::Twist twist = KDL::diff(T_T0_W_side, cell.getSamples()[i].T, 1.0);
        double dist = twist.rot.Norm() + twist.vel.Norm()*10.0;
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

int ReachabilityRange::getSamplesCount() const {
    return total_samples_;
}

const DiscreteSE3& ReachabilityRange::getSE3() const {
    return this->dspace_se3_;
}

unsigned char* ReachabilityRange::serialize(unsigned char* buf, bool write) const {
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

unsigned char* ReachabilityRange::deserialize(ReachabilityRangePtr& result, unsigned char* buf) {
    DiscreteNSpacePtr dspace_conf;
    buf = DiscreteNSpace::deserialize(dspace_conf, buf);
    DiscreteSE3Ptr dspace_se3;
    buf = DiscreteSE3::deserialize(dspace_se3, buf);
    int samples_count;
    buf = deserialize_int( samples_count, buf );

    result.reset(new ReachabilityRange(*dspace_conf, *dspace_se3));
    for (int sample_idx = 0; sample_idx < samples_count; ++sample_idx) {
        ReachabilityRange::Sample sample;
        buf = ReachabilityRange::Sample::deserialize( sample, buf );
        result->addPoint( sample );
    }
    return buf;
}
//};  // namespace rcprg_planner
