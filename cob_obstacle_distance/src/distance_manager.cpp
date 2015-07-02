/*
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_obstacle_distance
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Implementation of the DistanceManager definitions.
 ****************************************************************/


#include "cob_obstacle_distance/distance_manager.hpp"

#include <string.h>
#include <stdint.h>

#include <ros/ros.h>

#include <fcl/collision_object.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/collision_data.h>

#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include "cob_obstacle_distance/ObstacleDistance.h"
#include "cob_obstacle_distance/ObstacleDistances.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl_conversions/kdl_msg.h>

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

uint32_t DistanceManager::seq_nr_ = 0;

DistanceManager::DistanceManager(ros::NodeHandle& nh) : nh_(nh)
{}

DistanceManager::~DistanceManager()
{
}

int DistanceManager::init()
{
    KDL::Tree robot_structure;
    if (!kdl_parser::treeFromParam("/robot_description", robot_structure)){
        ROS_ERROR("Failed to construct kdl tree from parameter '/robot_description'.");
        return -1;
    }

    if(!nh_.getParam("joint_names", this->joints_))
    {
        ROS_ERROR("Failed to get parameter \"joint_names\".");
        return -2;
    }


    if(!nh_.getParam("root_frame", this->root_frame_))
    {
        ROS_ERROR("Failed to get parameter \"chain_base_link\".");
        return -3;
    }

    if(!nh_.getParam("chain_base_link", this->chain_base_link_))
    {
        ROS_ERROR("Failed to get parameter \"chain_base_link\".");
        return -3;
    }

    if (!nh_.getParam("chain_tip_link", this->chain_tip_link_))
    {
        ROS_ERROR("Failed to get parameter \"chain_tip_link\".");
        return -4;
    }

    robot_structure.getChain(this->chain_base_link_, this->chain_tip_link_, this->chain_);
    if(chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return -5;
    }

    for(uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        KDL::Segment s = chain_.getSegment(i);
        this->segments_.push_back(s.getName());
        ROS_INFO_STREAM("Managing Segment Name: " << s.getName());
    }


    // Latched and continue in case there is no subscriber at the moment for a marker
    this->marker_pub_ = this->nh_.advertise<visualization_msgs::Marker>("obstacle_distance/marker", 1, true);
    this->obstacle_distances_pub_ = this->nh_.advertise<cob_obstacle_distance::ObstacleDistances>("obstacle_distance", 1);

    obstacle_mgr_.reset(new ShapesManager(this->marker_pub_));
    object_of_interest_mgr_.reset(new ShapesManager(this->marker_pub_));

    adv_chn_fk_solver_vel_.reset(new AdvancedChainFkSolverVel_recursive(chain_));

    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());


    return 0;
}

void DistanceManager::clear()
{
    this->obstacle_mgr_->clear();
    this->object_of_interest_mgr_->clear();
}

bool DistanceManager::waitForMarkerSubscriber()
{
    while (this->marker_pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return false;
      }

      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    return true;
}

void DistanceManager::addObstacle(t_ptr_IMarkerShape s)
{
    this->obstacle_mgr_->addShape(s);
}

void DistanceManager::addObjectOfInterest(t_ptr_IMarkerShape s)
{
    this->object_of_interest_mgr_->addShape(s);
}

void DistanceManager::drawObstacles(bool enforceDraw)
{
    this->obstacle_mgr_->draw(enforceDraw);
}

void DistanceManager::drawObjectsOfInterest(bool enforceDraw)
{
    this->object_of_interest_mgr_->draw(enforceDraw);
}

bool DistanceManager::collide(t_ptr_IMarkerShape s1, t_ptr_IMarkerShape s2)
{
    fcl::CollisionObject x = s1->getCollisionObject();
    fcl::CollisionObject y = s2->getCollisionObject();
    fcl::CollisionResult result;
    fcl::CollisionRequest request(1, true);
    fcl::collide(&x, &y, request, result);
    ROS_INFO_STREAM("isCollision: " << result.isCollision() << std::endl);
}

void DistanceManager::calculate()
{
    cob_obstacle_distance::ObstacleDistances obstacle_distances;

    // Transform needs to be calculated only once for robot structure
    // and is same for all obstacles.
    if(this->obstacle_distances_.size() > 0)
    {
        if (!this->transform())
        {
            ROS_ERROR("Failed to transform Return with no publish!");
            return;
        }

        KDL::FrameVel p_dot_out;
        KDL::JntArrayVel jnt_arr(last_q_, last_q_dot_);
        adv_chn_fk_solver_vel_->JntToCart(jnt_arr, p_dot_out);
    }

    for(t_map_ObstacleDistance_iter it = this->obstacle_distances_.begin(); it != this->obstacle_distances_.end(); ++it)
    {
        std::string frame_of_interest_name = it->first;
        std::vector<std::string>::const_iterator str_it = std::find(this->segments_.begin(),
                                                                    this->segments_.end(),
                                                                    frame_of_interest_name);
        if(this->segments_.end() == str_it)
        {
            ROS_ERROR_STREAM("Could not find: " << frame_of_interest_name << ". Continue ...");
            continue;
        }

        uint16_t idx = str_it - this->segments_.begin();

        /* ******* Start Transformation part ************** */
        KDL::FrameVel frame_vel = adv_chn_fk_solver_vel_->getFrameVelAtSegment(idx);
        KDL::Frame frame_pos = frame_vel.GetFrame();
        Eigen::Vector3d chainbase2frame_pos(frame_pos.p.x(),
                                            frame_pos.p.y(),
                                            frame_pos.p.z());
        Eigen::Vector3d abs_jnt_pos = tf_cb_frame_bl_.inverse() * chainbase2frame_pos;

        Eigen::Quaterniond q;
        tf::quaternionKDLToEigen (frame_pos.M, q);
        /* ******* End Transformation part ************** */

        // Representation of segment_of_interest as specific shape
        fcl::Box b(0.1, 0.1, 0.1);
        fcl::Sphere s(0.1);
        fcl::Cylinder c(0.1, 0.1);
        t_ptr_IMarkerShape ooi;

        if(!this->getMarkerShape(it->second.shape_type, abs_jnt_pos, q, ooi))
        {
            return;
        }

        fcl::CollisionObject ooi_co = ooi->getCollisionObject();
        ooi.reset();
        fcl::DistanceResult dist_result;
        bool setDistResult = false;
        fcl::CollisionObject result_collision_obj = ooi_co;
        fcl::FCL_REAL last_dist = std::numeric_limits<fcl::FCL_REAL>::max();
        for(ShapesManager::t_iter it = this->obstacle_mgr_->begin(); it != this->obstacle_mgr_->end(); ++it)
        {
            fcl::CollisionObject collision_obj = (*it)->getCollisionObject();
            fcl::DistanceResult tmpResult;
            fcl::DistanceRequest distRequest(true);
            fcl::FCL_REAL dist = fcl::distance(&ooi_co, &collision_obj, distRequest, tmpResult);
            if (dist < last_dist)
            {
                setDistResult = true;
                dist_result = tmpResult;
                result_collision_obj = collision_obj;
                last_dist = dist;
            }
        }

        if(setDistResult)
        {
            fcl::Vec3f t = result_collision_obj.getTransform().transform(dist_result.nearest_points[1]); // Transform from "base_link" frame!!!
            fcl::Vec3f a = ooi_co.getTransform().transform(dist_result.nearest_points[0]);
            fcl::Quaternion3f qt = result_collision_obj.getQuatRotation();
            fcl::Quaternion3f qa = ooi_co.getQuatRotation();
            fcl::Quaternion3f diff_a_t = qa - qt;

            Eigen::Vector3d abs_obst_vector(t[VEC_X],
                                       t[VEC_Y],
                                       t[VEC_Z]);
            Eigen::Vector3d obst_vector = tf_cb_frame_bl_ * abs_obst_vector;
            Eigen::Vector3d tmp_obst_vector = tf_cb_frame_bl_.rotation() * abs_obst_vector + tf_cb_frame_bl_.translation();

            Eigen::Vector3d abs_jnt_pos_update(a[VEC_X],
                                               a[VEC_Y],
                                               a[VEC_Z]);

            Eigen::Vector3d rel_base_link_frame_pos = tf_cb_frame_bl_ * abs_jnt_pos_update;
            Eigen::Vector3d dist_vector = rel_base_link_frame_pos - obst_vector; // expressed in arm base link frame

            // vector from frame origin to collision point
            Eigen::Vector3d rel_frame_origin_to_collision_pnt = rel_base_link_frame_pos - chainbase2frame_pos;
            ROS_INFO_STREAM("Frame \"" << frame_of_interest_name << "\": Minimal distance: " << dist_result.min_distance);

            cob_obstacle_distance::ObstacleDistance od_msg;
            od_msg.distance = dist_result.min_distance;
            od_msg.distance_vector.position.x = dist_vector(VEC_X);
            od_msg.distance_vector.position.y = dist_vector(VEC_Y);
            od_msg.distance_vector.position.z = dist_vector(VEC_Z);
            od_msg.distance_vector.orientation.x = diff_a_t.getX();
            od_msg.distance_vector.orientation.y = diff_a_t.getY();
            od_msg.distance_vector.orientation.z = diff_a_t.getZ();
            od_msg.distance_vector.orientation.w = diff_a_t.getW();
            od_msg.header.frame_id = frame_of_interest_name;
            od_msg.header.stamp = ros::Time::now();
            od_msg.header.seq = seq_nr_;
            tf::twistKDLToMsg(frame_vel.GetTwist(), od_msg.frame_twist);
            tf::vectorEigenToMsg(rel_frame_origin_to_collision_pnt, od_msg.collision_pnt_vector);
            obstacle_distances.distances.push_back(od_msg);
        }
    }

    if(obstacle_distances.distances.size() > 0)
    {
        this->obstacle_distances_pub_.publish(obstacle_distances);
    }
}

bool DistanceManager::transform()
{
    bool success = true;
    try
    {
        tf::StampedTransform cb_transform_bl;
        tf_listener_.waitForTransform(chain_base_link_, root_frame_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(chain_base_link_, root_frame_, ros::Time(0), cb_transform_bl);
        tf::transformTFToEigen(cb_transform_bl, tf_cb_frame_bl_);
    }
    catch (tf::TransformException &ex)
    {
        success = false;
        ROS_ERROR("%s",ex.what());
    }

    return success;
}

void DistanceManager::jointstateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    uint16_t count = 0;

    for(uint16_t j = 0; j < chain_.getNrOfJoints(); ++j)
    {
        for(uint16_t i = 0; i < msg->name.size(); i++)
        {
            if(strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }

    if(joints_.size() == count)
    {
        last_q_ = q_temp;
        last_q_dot_ = q_dot_temp;
    }
    else
    {
        ROS_ERROR("Failed to jointstate_cb");
    }
}

bool DistanceManager::registerPointOfInterest(cob_obstacle_distance::Registration::Request& request,
                                              cob_obstacle_distance::Registration::Response& response)
{
    ROS_INFO_STREAM("Registering a point / frame of interest!");
    if (this->obstacle_distances_.count(request.frame_id))
    {
        response.success = true;
        response.message = "Element " + request.frame_id + " already existent.";
    }
    else
    {
        try
        {
            this->obstacle_distances_[request.frame_id] = ObstacleDistance(request.shape_type);
            response.success = true;
            response.message = "Successfully inserted element " + request.frame_id + ".";
        }
        catch(...)
        {
            response.success = false;
            response.message = "Failed to insert element " + request.frame_id + "!";
            ROS_ERROR_STREAM(response.message);
        }
    }

    return true;
}

bool DistanceManager::predictDistance(cob_obstacle_distance::PredictDistance::Request& request,
                                      cob_obstacle_distance::PredictDistance::Response& response)
{
    // Transform needs to be calculated only once for robot structure
    // and is same for all obstacles.
    if (!this->transform())
    {
        ROS_ERROR("Failed to transform Return with no publish!");
        return true;
    }

    KDL::FrameVel p_dot_out;
    KDL::JntArrayVel jnt_arr(last_q_, last_q_dot_);
    adv_chn_fk_solver_vel_->JntToCart(jnt_arr, p_dot_out);

    for(uint32_t frame_idx = 0; frame_idx < request.frame_id.size(); ++frame_idx)
    {
        KDL::JntArray joint_pos(request.joint_pos.size());
        std::string frame_id = request.frame_id.at(frame_idx);
        for(uint32_t lv = 0; lv < request.joint_pos.size(); ++lv)
        {
            joint_pos(lv) = request.joint_pos.at(lv);
        }

        std::vector<std::string>::const_iterator str_it = std::find(this->segments_.begin(),
                                                                    this->segments_.end(),
                                                                    frame_id);
        uint16_t idx = str_it - this->segments_.begin();

        /* ******* Start Transformation part ************** */
        KDL::FrameVel frame_vel = adv_chn_fk_solver_vel_->getFrameVelAtSegment(idx);
        KDL::Frame frame_pos = frame_vel.GetFrame();
        Eigen::Vector3d chainbase2frame_pos(frame_pos.p.x(),
                                            frame_pos.p.y(),
                                            frame_pos.p.z());
        Eigen::Vector3d abs_jnt_pos = tf_cb_frame_bl_.inverse() * chainbase2frame_pos;
        Eigen::Quaterniond q;
        tf::quaternionKDLToEigen (frame_pos.M, q);
        /* ******* End Transformation part ************** */

        // Representation of segment_of_interest as specific shape
        t_ptr_IMarkerShape ooi;
        if(!DistanceManager::getMarkerShape(visualization_msgs::Marker::SPHERE, abs_jnt_pos, q, ooi))
        {
            return true;
        }

        fcl::CollisionObject ooi_co = ooi->getCollisionObject();
        ooi.reset();
        fcl::CollisionObject result_collision_obj = ooi_co;
        fcl::FCL_REAL last_dist = std::numeric_limits<fcl::FCL_REAL>::max();
        fcl::FCL_REAL dist;
        for(ShapesManager::t_iter it = this->obstacle_mgr_->begin(); it != this->obstacle_mgr_->end(); ++it)
        {
            fcl::CollisionObject collision_obj = (*it)->getCollisionObject();
            fcl::DistanceResult tmpResult;
            fcl::DistanceRequest distRequest(true);
            dist = fcl::distance(&ooi_co, &collision_obj, distRequest, tmpResult);
            if (dist < last_dist)
            {
                result_collision_obj = collision_obj;
                last_dist = dist;
            }
        }

        response.min_distances.push_back(dist);
    }

    return true;
}

bool DistanceManager::getMarkerShape(uint32_t shape_type, const Eigen::Vector3d& abs_pos, const Eigen::Quaterniond& quat_pos, t_ptr_IMarkerShape& segment_of_interest_marker_shape)
{
    // Representation of segment_of_interest as specific fcl::Shape
    fcl::Box b(0.1, 0.1, 0.1);
    fcl::Sphere s(0.05);
    fcl::Cylinder c(0.05, 0.1);

    geometry_msgs::Pose pose;
    pose.position.x = abs_pos(VEC_X);
    pose.position.y = abs_pos(VEC_Y);
    pose.position.z = abs_pos(VEC_Z);
    pose.orientation.x = quat_pos.x();
    pose.orientation.y = quat_pos.y();
    pose.orientation.z = quat_pos.z();
    pose.orientation.w = quat_pos.w();

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 0.0;

    switch(shape_type)
    {
        case visualization_msgs::Marker::CUBE:
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Box>(this->root_frame_, b, pose, col));
            break;
        case visualization_msgs::Marker::SPHERE:
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Sphere>(this->root_frame_, s, pose, col));
            break;
        case visualization_msgs::Marker::CYLINDER:
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Cylinder>(this->root_frame_, c, pose, col));
            break;
        default:
           ROS_ERROR("Failed to process request due to unknown shape type: %d", shape_type);
           return false;
    }

    return true;
}
