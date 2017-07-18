/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <limits>
#include <string>
#include <vector>

#include "cob_obstacle_distance/distance_manager.hpp"


#include <stdint.h>

#include <ros/ros.h>

#include <fcl/collision_object.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/collision_data.h>

#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include "cob_control_msgs/ObstacleDistance.h"
#include "cob_control_msgs/ObstacleDistances.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/pointer_cast.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl_conversions/kdl_msg.h>

#include <urdf/model.h>

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/SolidPrimitive.h>

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2


uint32_t DistanceManager::seq_nr_ = 0;

DistanceManager::DistanceManager(ros::NodeHandle& nh) : nh_(nh), stop_sca_threads_(false)
{}

DistanceManager::~DistanceManager()
{
    this->clear();
}

int DistanceManager::init()
{
    this->stop_sca_threads_ = false;

    // Latched and continue in case there is no subscriber at the moment for a marker
    this->marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_distance/marker", 10, true);
    this->obstacle_distances_pub_ = this->nh_.advertise<cob_control_msgs::ObstacleDistances>("obstacle_distance", 1);
    obstacle_mgr_.reset(new ShapesManager(this->marker_pub_));
    object_of_interest_mgr_.reset(new ShapesManager(this->marker_pub_));
    KDL::Tree robot_structure;
    if (!kdl_parser::treeFromParam("/robot_description", robot_structure))
    {
        ROS_ERROR("Failed to construct kdl tree from parameter '/robot_description'.");
        return -1;
    }

    if (!nh_.getParam("joint_names", this->joints_))
    {
        ROS_ERROR("Failed to get parameter \"joint_names\".");
        return -2;
    }

    if (!nh_.getParam("root_frame", this->root_frame_id_))
    {
        ROS_ERROR("Failed to get parameter \"chain_base_link\".");
        return -3;
    }

    if (!nh_.getParam("chain_base_link", this->chain_base_link_))
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
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return -5;
    }

    for (uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        KDL::Segment s = chain_.getSegment(i);
        this->segments_.push_back(s.getName());
        ROS_INFO_STREAM("Managing Segment Name: " << s.getName());
    }

    adv_chn_fk_solver_vel_.reset(new AdvancedChainFkSolverVel_recursive(chain_));
    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
    if (!this->link_to_collision_.initParameter(this->root_frame_id_, "/robot_description"))
    {
        ROS_ERROR("Failed to initialize robot model from URDF by parameter \"/robot_description\".");
        return -6;
    }
    else
    {
        XmlRpc::XmlRpcValue scm;
        bool success = false;
        if (nh_.getParam("self_collision_map", scm))
        {
            success = this->link_to_collision_.initSelfCollision(scm, obstacle_mgr_);
        }

        if (!success)
        {
            ROS_WARN("Parameter 'self_collision_map' not found or map empty.");
        }

        for (LinkToCollision::MapSelfCollisions_t::iterator it = this->link_to_collision_.getSelfCollisionsIterBegin();
                it != this->link_to_collision_.getSelfCollisionsIterEnd();
                it++)
        {
            self_collision_transform_threads_.push_back(std::thread(&DistanceManager::transformSelfCollisionLinks, this, it->first));
        }
    }

    return 0;
}

void DistanceManager::clear()
{
    this->stop_sca_threads_ = true;
    for (std::vector<std::thread>::iterator it = this->self_collision_transform_threads_.begin();
            it != this->self_collision_transform_threads_.end(); it++)
    {
        it->join();
    }

    this->obstacle_mgr_->clear();
    this->object_of_interest_mgr_->clear();
}


void DistanceManager::addObstacle(const std::string& id, PtrIMarkerShape_t s)
{
    this->obstacle_mgr_->addShape(id, s);
}


void DistanceManager::addObjectOfInterest(const std::string& id, PtrIMarkerShape_t s)
{
    this->object_of_interest_mgr_->addShape(id, s);
}


void DistanceManager::drawObstacles()
{
    this->obstacle_mgr_->draw();
}


void DistanceManager::drawObjectsOfInterest()
{
    this->object_of_interest_mgr_->draw();
}


void DistanceManager::calculate()
{
    cob_control_msgs::ObstacleDistances obstacle_distances;

    // Transform needs to be calculated only once for robot structure
    // and is same for all obstacles.
    if (this->object_of_interest_mgr_->count() > 0)
    {
        KDL::FrameVel p_dot_out;
        KDL::JntArrayVel jnt_arr(last_q_, last_q_dot_);
        adv_chn_fk_solver_vel_->JntToCart(jnt_arr, p_dot_out);
    }

    for (ShapesManager::MapIter_t it = this->object_of_interest_mgr_->begin(); it != this->object_of_interest_mgr_->end(); ++it)
    {
        std::string object_of_interest_name = it->first;
        std::vector<std::string>::const_iterator str_it = std::find(this->segments_.begin(),
                                                                    this->segments_.end(),
                                                                    object_of_interest_name);
        if (this->segments_.end() == str_it)
        {
            ROS_ERROR_STREAM("Could not find: " << object_of_interest_name << ". Skipping it ...");
            continue;
        }

        // Representation of segment_of_interest as specific shape
        PtrIMarkerShape_t ooi = it->second;
        uint16_t idx = str_it - this->segments_.begin();
        geometry_msgs::Pose origin_p = ooi->getOriginRelToFrame();
        KDL::Frame origin_f;
        tf::poseMsgToKDL(origin_p, origin_f);

        // ******* Start Transformation part **************
        KDL::FrameVel frame_vel = adv_chn_fk_solver_vel_->getFrameVelAtSegment(idx);
        KDL::Frame frame_pos = frame_vel.GetFrame();
        KDL::Frame frame_with_offset = frame_pos * origin_f;

        Eigen::Vector3d chainbase2frame_pos(frame_with_offset.p.x(),
                                            frame_with_offset.p.y(),
                                            frame_with_offset.p.z());

        Eigen::Affine3d tmp_tf_cb_frame_bl = this->getSynchedCbToBlTransform();
        Eigen::Affine3d tmp_inv_tf_cb_frame_bl = tmp_tf_cb_frame_bl.inverse();
        Eigen::Vector3d abs_jnt_pos = tmp_inv_tf_cb_frame_bl * chainbase2frame_pos;

        Eigen::Quaterniond q;
        tf::quaternionKDLToEigen(frame_with_offset.M, q);
        Eigen::Matrix3d x = (tmp_inv_tf_cb_frame_bl * q).rotation();
        Eigen::Quaterniond q_1(x);
        // ******* End Transformation part **************

        geometry_msgs::Vector3 v3;
        geometry_msgs::Quaternion quat;
        tf::quaternionEigenToMsg(q_1, quat);
        tf::vectorEigenToMsg(abs_jnt_pos, v3);
        ooi->updatePose(v3, quat);

        fcl::CollisionObject ooi_co = ooi->getCollisionObject();
        fcl::FCL_REAL last_dist = std::numeric_limits<fcl::FCL_REAL>::max();
        {  // introduced the block to lock this critical section until block leaved.
            std::lock_guard<std::mutex> lock(obstacle_mgr_mtx_);
            for (ShapesManager::MapIter_t it = this->obstacle_mgr_->begin(); it != this->obstacle_mgr_->end(); ++it)
            {
                const std::string obstacle_id = it->first;
                if (this->link_to_collision_.ignoreSelfCollisionPart(object_of_interest_name, obstacle_id))
                {
                    // Ignore elements that can never be in collision
                    // (specified in parameter and parent / child frames)
                    continue;
                }

                PtrIMarkerShape_t obstacle = it->second;
                fcl::CollisionObject collision_obj = obstacle->getCollisionObject();
                fcl::DistanceResult dist_result;
                fcl::DistanceRequest dist_request(true, 5.0, 0.01);
                fcl::FCL_REAL dist = fcl::distance(&ooi_co, &collision_obj, dist_request, dist_result);


                Eigen::Vector3d abs_obst_vector(dist_result.nearest_points[1][VEC_X],
                                                dist_result.nearest_points[1][VEC_Y],
                                                dist_result.nearest_points[1][VEC_Z]);
                Eigen::Vector3d obst_vector = tmp_tf_cb_frame_bl * abs_obst_vector;

                Eigen::Vector3d abs_jnt_pos_update(dist_result.nearest_points[0][VEC_X],
                                                   dist_result.nearest_points[0][VEC_Y],
                                                   dist_result.nearest_points[0][VEC_Z]);

                // vector from arm base link frame to nearest collision point on frame
                Eigen::Vector3d rel_base_link_frame_pos = tmp_tf_cb_frame_bl * abs_jnt_pos_update;
                ROS_DEBUG_STREAM("Link \"" << object_of_interest_name << "\": Minimal distance: " << dist_result.min_distance);
                if (dist_result.min_distance < MIN_DISTANCE)
                {
                    cob_control_msgs::ObstacleDistance od_msg;
                    od_msg.distance = dist_result.min_distance;
                    od_msg.link_of_interest = object_of_interest_name;
                    od_msg.obstacle_id = obstacle_id;
                    od_msg.header.frame_id = chain_base_link_;
                    od_msg.header.stamp = ros::Time::now();
                    od_msg.header.seq = seq_nr_;
                    tf::vectorEigenToMsg(obst_vector, od_msg.nearest_point_obstacle_vector);
                    tf::vectorEigenToMsg(rel_base_link_frame_pos, od_msg.nearest_point_frame_vector);
                    tf::vectorEigenToMsg(chainbase2frame_pos, od_msg.frame_vector);
                    obstacle_distances.distances.push_back(od_msg);
                }
            }
        }
    }

    if (obstacle_distances.distances.size() > 0)
    {
        this->obstacle_distances_pub_.publish(obstacle_distances);
    }
}


void DistanceManager::transform()
{
    while (!this->stop_sca_threads_)
    {
        try
        {
            tf::StampedTransform cb_transform_bl;
            ros::Time time = ros::Time(0);
            if (tf_listener_.waitForTransform(chain_base_link_, root_frame_id_, time, ros::Duration(5.0)))
            {
                std::lock_guard<std::mutex> lock(mtx_);
                tf_listener_.lookupTransform(chain_base_link_, root_frame_id_, time, cb_transform_bl);
                tf::transformTFToEigen(cb_transform_bl, tf_cb_frame_bl_);
            }
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        ros::Duration(0.1).sleep();
    }
}


void DistanceManager::transformSelfCollisionLinks(const std::string link_name)
{
    ROS_INFO_STREAM("Starting transform_listener thread for link name: " << link_name);
    while (!this->stop_sca_threads_)
    {
        try
        {
            ros::Time time = ros::Time(0);
            // https://github.com/ros-visualization/rviz/issues/702: TF listener is thread safe
            if (tf_listener_.waitForTransform(root_frame_id_, link_name, time, ros::Duration(5.0)))
            {
                std::lock_guard<std::mutex> lock(obstacle_mgr_mtx_);
                tf::StampedTransform stamped_transform;
                geometry_msgs::Transform msg_transform;
                tf_listener_.lookupTransform(root_frame_id_, link_name, time, stamped_transform);
                PtrIMarkerShape_t shape_ptr;
                if (this->obstacle_mgr_->getShape(link_name, shape_ptr))
                {
                    geometry_msgs::Pose origin_p = shape_ptr->getOriginRelToFrame();
                    tf::Transform tf_origin_pose;
                    tf::poseMsgToTF(origin_p, tf_origin_pose);
                    tf::Transform updated_pose = tf::Transform(stamped_transform) * tf_origin_pose;
                    tf::transformTFToMsg(updated_pose, msg_transform);
                    shape_ptr->updatePose(msg_transform.translation, msg_transform.rotation);
                }
            }
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        ros::Duration(0.1).sleep();
    }
}


void DistanceManager::jointstateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    uint16_t count = 0;

    for (uint16_t j = 0; j < chain_.getNrOfJoints(); j++)
    {
        for (uint16_t i = 0; i < msg->name.size(); i++)
        {
            if (strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }

    if (joints_.size() == count)
    {
        last_q_ = q_temp;
        last_q_dot_ = q_dot_temp;
    }
    else
    {
        ROS_ERROR("jointstateCb: received unexpected 'joint_states'");
    }
}


void DistanceManager::registerObstacle(const moveit_msgs::CollisionObject::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(obstacle_mgr_mtx_);

    const std::string frame_id = msg->header.frame_id;
    tf::StampedTransform frame_transform_root;
    Eigen::Affine3d tf_frame_root;

    if (msg->meshes.size() > 0 && msg->primitives.size() > 0)
    {
        ROS_ERROR_STREAM("registerObstacle: Can either build mesh or primitive but not both in one message for one id.");
        return;
    }

    if (msg->operation == msg->ADD && this->obstacle_mgr_->count(msg->id) > 0)
    {
        ROS_ERROR_STREAM("registerObstacle: Element " << msg->id << " exists already. ADD not allowed!");
        return;
    }

    try
    {
        ros::Time time = ros::Time(0);
        tf_listener_.waitForTransform(root_frame_id_, msg->header.frame_id, time, ros::Duration(0.5));
        tf_listener_.lookupTransform(root_frame_id_, msg->header.frame_id, time, frame_transform_root);
        tf::transformTFToEigen(frame_transform_root, tf_frame_root);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("TransformException: %s", ex.what());
        return;
    }

    if ((msg->type.db.length() > 0 && 0 < msg->mesh_poses.size()) ||
       (msg->meshes.size() > 0 && msg->meshes.size() == msg->mesh_poses.size()))
    {
        this->buildObstacleMesh(msg, frame_transform_root);
    }

    if (msg->primitives.size() > 0 && msg->primitives.size() == msg->primitive_poses.size())
    {
        this->buildObstaclePrimitive(msg, frame_transform_root);
    }

    this->drawObstacles();
}


void DistanceManager::buildObstacleMesh(const moveit_msgs::CollisionObject::ConstPtr& msg, const tf::StampedTransform& transform)
{
    uint32_t m_size = msg->mesh_poses.size();
    const std::string package_file_name = msg->type.db;  // using db field for package name instead of db json string

    if (msg->mesh_poses.size() > 1)
    {
        ROS_WARN("Currenty only one mesh per message is supported! So only the first one is processed ...");
        m_size = 1;
    }

    if (package_file_name.length() <= 0 && msg->mesh_poses.size() != msg->meshes.size())
    {
       ROS_ERROR("Mesh poses and meshes do not have the same size. If package resource string is empty then the sizes must be equal!");
       return;
    }

    if (msg->ADD == msg->operation)
    {
        for (uint32_t i = 0; i < m_size; ++i)
        {
            geometry_msgs::Pose p = msg->mesh_poses[i];
            tf::Pose tf_p;
            tf::poseMsgToTF(p, tf_p);
            tf::Pose new_tf_p = transform * tf_p;
            tf::poseTFToMsg(new_tf_p, p);
            PtrIMarkerShape_t sptr_Bvh;
            if (package_file_name.length() > 0)
            {
                sptr_Bvh.reset(new MarkerShape<BVH_RSS_t>(this->root_frame_id_,
                                                          package_file_name,
                                                          p,
                                                          g_shapeMsgTypeToVisMarkerType.obstacle_color_));
            }
            else
            {
                shape_msgs::Mesh m = msg->meshes[i];  // is only filled in case of no package file name has been given
                sptr_Bvh.reset(new MarkerShape<BVH_RSS_t>(this->root_frame_id_,
                                                          m,
                                                          p,
                                                          g_shapeMsgTypeToVisMarkerType.obstacle_color_));
            }

            this->addObstacle(msg->id, sptr_Bvh);
        }
    }
    else if (msg->MOVE == msg->operation)
    {
        PtrIMarkerShape_t sptr;
        for (uint32_t i = 0; i < m_size; ++i)
        {
            if (this->obstacle_mgr_->getShape(msg->id, sptr))
            {
                geometry_msgs::Pose p = msg->mesh_poses[i];
                tf::Pose tf_p;
                tf::poseMsgToTF(p, tf_p);
                tf::Pose new_tf_p = transform * tf_p;
                tf::poseTFToMsg(new_tf_p, p);
                sptr->updatePose(p);
            }
        }
    }
    else if (msg->REMOVE == msg->operation)
    {
        this->obstacle_mgr_->removeShape(msg->id);
    }
    else
    {
        ROS_ERROR_STREAM("Operation not supported!");
    }
}


void DistanceManager::buildObstaclePrimitive(const moveit_msgs::CollisionObject::ConstPtr& msg, const tf::StampedTransform& transform)
{
    uint32_t p_size = msg->primitives.size();
    if (msg->primitives.size() > 1)
    {
        ROS_WARN("Currenty only one primitive per message is supported! So only the first one is processed ...");
        p_size = 1;
    }

    if (msg->ADD == msg->operation)
    {
        for (uint32_t i = 0; i < p_size; ++i)
        {
            shape_msgs::SolidPrimitive sp = msg->primitives[i];
            geometry_msgs::Pose p = msg->primitive_poses[i];
            tf::Pose tf_p;
            tf::poseMsgToTF(p, tf_p);
            tf::Pose new_tf_p = transform * tf_p;
            tf::poseTFToMsg(new_tf_p, p);

            PtrIMarkerShape_t sptr;
            Eigen::Vector3d dim;
            if (shape_msgs::SolidPrimitive::BOX == sp.type)
            {
                dim(FCL_BOX_X) = sp.dimensions[shape_msgs::SolidPrimitive::BOX_X];
                dim(FCL_BOX_Y) = sp.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                dim(FCL_BOX_Z) = sp.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
            }
            else if (shape_msgs::SolidPrimitive::SPHERE == sp.type)
            {
                dim(FCL_RADIUS) = sp.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            }
            else if (shape_msgs::SolidPrimitive::CYLINDER == sp.type)
            {
                dim(FCL_RADIUS) = sp.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
                dim(FCL_CYL_LENGTH) = sp.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
            }
            else
            {
                ROS_ERROR_STREAM("Shape type not supported: " << sp.type);
                break;
            }

            uint32_t shape_type = g_shapeMsgTypeToVisMarkerType.map_[sp.type];
            this->link_to_collision_.getMarkerShapeFromType(shape_type,
                                                             p,
                                                             msg->id,
                                                             dim,
                                                             sptr);
            this->addObstacle(msg->id, sptr);
        }
    }
    else if (msg->MOVE == msg->operation)
    {
        PtrIMarkerShape_t sptr;
        for (uint32_t i = 0; i < p_size; ++i)
        {
            if (this->obstacle_mgr_->getShape(msg->id, sptr))
            {
                geometry_msgs::Pose p = msg->primitive_poses[i];
                tf::Pose tf_p;
                tf::poseMsgToTF(p, tf_p);
                tf::Pose new_tf_p = transform * tf_p;
                tf::poseTFToMsg(new_tf_p, p);
                sptr->updatePose(p);
            }
        }
    }
    else if (msg->REMOVE == msg->operation)
    {
        this->obstacle_mgr_->removeShape(msg->id);
    }
    else
    {
        ROS_ERROR_STREAM("Operation not supported!");
    }
}


bool DistanceManager::registerLinkOfInterest(cob_srvs::SetString::Request& request,
                                              cob_srvs::SetString::Response& response)
{
    if (this->object_of_interest_mgr_->count(request.data) > 0)
    {
        response.success = true;
        response.message = "Element " + request.data + " already registered.";
    }
    else
    {
        try
        {
            PtrIMarkerShape_t ooi;
            Eigen::Quaterniond q;
            Eigen::Vector3d v3;
            if (this->link_to_collision_.getMarkerShapeFromUrdf(v3, q, request.data, ooi))
            {
                this->addObjectOfInterest(request.data, ooi);
                response.success = true;
                response.message = "Successfully registered element " + request.data + ".";
            }
            else
            {
                response.success = false;
                response.message = "Failed to register element " + request.data + "!";
            }
        }
        catch(...)
        {
            response.success = false;
            response.message = "Failed to register element " + request.data + "!";
            ROS_ERROR_STREAM(response.message);
        }
    }

    ROS_INFO_STREAM("DistanceManager::Registration: " << response.message);
    return true;
}


Eigen::Affine3d DistanceManager::getSynchedCbToBlTransform()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return this->tf_cb_frame_bl_;
}
