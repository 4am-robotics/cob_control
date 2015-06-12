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

#include <ros/package.h>

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

DistanceManager::DistanceManager(ros::NodeHandle& nh) : nh_(nh)
{}

DistanceManager::~DistanceManager()
{
}

int DistanceManager::init()
{
    KDL::Tree robot_structure;
    std::string robot_desc_string;
    nh_.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_structure)){
        ROS_ERROR("Failed to construct kdl tree from parameter '/robot_description'.");
        std::string path = ros::package::getPath("cob_obstacle_distance");
        std::string testdata_path = ros::package::getPath("cob_obstacle_distance") + "/testdata/robot_description.xml";
        if (!kdl_parser::treeFromFile(testdata_path,
                                      robot_structure))
        {
            ROS_ERROR("Failed to construct kdl tree from test data. ");
            return -1;
        }

        ROS_WARN_STREAM("Constructed from test data in \"" << testdata_path << "\".");
    }

    if(!nh_.getParam("robo_namespace", this->robo_namespace_))
    {
        ROS_ERROR("Failed to get parameter \"robo_namespace\".");
        return -2;
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

    if(!nh_.getParam("joint_names", this->joints_))
    {
        ROS_ERROR("Failed to get parameter \"joint_names\".");
        return -5;
    }

    robot_structure.getChain(this->chain_base_link_, this->chain_tip_link_, this->chain_);
    if(chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return -6;
    }

    for(uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        KDL::Segment s = chain_.getSegment(i);
        this->segments_.push_back(s.getName());
        ROS_INFO_STREAM("Managing Segment Name: " << s.getName());
    }

    this->marker_pub_ = this->nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    this->obstacle_distances_pub_ = this->nh_.advertise<cob_obstacle_distance::ObstacleDistances>(this->robo_namespace_, 1);

    obstacle_mgr_.reset(new ShapesManager(this->marker_pub_));
    object_of_interest_mgr_.reset(new ShapesManager(this->marker_pub_));

    adv_chn_fk_solver_pos_.reset(new AdvancedChainFkSolverPos_recursive(chain_));

    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());

    jnt2jac_.reset(new KDL::ChainJntToJacSolver(chain_));
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

void DistanceManager::addObstacle(tPtrMarkerShapeBase s)
{
    this->obstacle_mgr_->addShape(s);
}

void DistanceManager::addObjectOfInterest(tPtrMarkerShapeBase s)
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

bool DistanceManager::collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2)
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
    bool initial = true;
    cob_obstacle_distance::ObstacleDistances obstacle_distances;
    for(t_map_ObstacleDistance_iter it = this->obstacle_distances_.begin(); it != this->obstacle_distances_.end(); ++it)
    {
        if(initial)
        {
            if (!this->transform())
            {
                ROS_ERROR("Failed to transform Return with no publish!");
                return;
            }

            initial = false;
        }

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
        KDL::Frame p_out;
        int retVal = adv_chn_fk_solver_pos_->JntToCart(last_q_, p_out);
        KDL::Frame frame_pos = adv_chn_fk_solver_pos_->getFrameAtSegment(idx);
        Eigen::Vector3d chainbase2fourth_jnt_pos(frame_pos.p.x(),
                                                 frame_pos.p.y(),
                                                 frame_pos.p.z());
        Eigen::Vector3d abs_jnt_pos = tf_cb_frame_bl_.inverse() * chainbase2fourth_jnt_pos;
        /* ******* End Transformation part ************** */

        // Representation of segment_of_interest as specific shape
        fcl::Box b(0.1, 0.1, 0.1);
        fcl::Sphere s(0.1);
        fcl::Cylinder c(0.1, 0.1);
        tPtrMarkerShapeBase ooi;
        if(!DistanceManager::getMarkerShape(it->second.shape_type, abs_jnt_pos, ooi))
        {
            return;
        }

        fcl::CollisionObject ooi_co = ooi->getCollisionObject();
        ooi.reset();
        fcl::DistanceResult dist_result;
        bool setDistResult = false;
        fcl::CollisionObject result_collision_obj = ooi_co;
        fcl::FCL_REAL lastDist = std::numeric_limits<fcl::FCL_REAL>::max();
        ROS_INFO_STREAM("Iteration over obstacles for segment of interest: " << frame_of_interest_name);
        for(ShapesManager::iterator it = this->obstacle_mgr_->begin(); it != this->obstacle_mgr_->end(); ++it)
        {
            fcl::CollisionObject collision_obj = (*it)->getCollisionObject();
            fcl::DistanceResult tmpResult;
            fcl::DistanceRequest distRequest(true, 1.0, 100.0);
            fcl::FCL_REAL dist = fcl::distance(&ooi_co, &collision_obj, distRequest, tmpResult);
            if (dist < lastDist)
            {
                setDistResult = true;
                dist_result = tmpResult;
                result_collision_obj = collision_obj;
            }
        }

        if(setDistResult)
        {
            ROS_INFO_STREAM("Minimum distance: " << dist_result.min_distance);
            fcl::Vec3f t = dist_result.nearest_points[1] + result_collision_obj.getTranslation(); // Translation from "base_link" frame!!!
            fcl::Quaternion3f q = result_collision_obj.getQuatRotation();

            Eigen::Vector3d obst_vector(t[VEC_X],
                                       t[VEC_Y],
                                       t[VEC_Z]);

            ROS_INFO_STREAM("absJntPosition: " << abs_jnt_pos);
            Eigen::Vector3d dist_vector = abs_jnt_pos - obst_vector;
            cob_obstacle_distance::ObstacleDistance od_msg;
            od_msg.distance = dist_result.min_distance;
            od_msg.distance_vector.x = dist_vector(VEC_X);
            od_msg.distance_vector.y = dist_vector(VEC_Y);
            od_msg.distance_vector.z = dist_vector(VEC_Z);
            od_msg.id.frame_id = frame_of_interest_name;
            od_msg.id.stamp = ros::Time::now();
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
        tf_listener_.waitForTransform(chain_base_link_, "base_link", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(chain_base_link_, "base_link", ros::Time(0), cb_transform_bl);
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

bool DistanceManager::getMarkerShape(uint32_t shape_type, const Eigen::Vector3d& abs_pos, tPtrMarkerShapeBase& segment_of_interest_marker_shape)
{
    // Representation of segment_of_interest as specific fcl::Shape
    fcl::Box b(0.1, 0.1, 0.1);
    fcl::Sphere s(0.1);
    fcl::Cylinder c(0.1, 0.1);
    switch(shape_type)
    {
        case visualization_msgs::Marker::CUBE:
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Box>(b, abs_pos(0), abs_pos(1), abs_pos(2)));
            break;
        case visualization_msgs::Marker::SPHERE:
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Sphere>(s, abs_pos(0), abs_pos(1), abs_pos(2)));
            break;
        case visualization_msgs::Marker::CYLINDER:
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Cylinder>(c, abs_pos(0), abs_pos(1), abs_pos(2)));
            break;
        default:
           ROS_ERROR("Failed to process request due to unknown shape type: %d", shape_type);
           return false;
    }

    return true;
}
