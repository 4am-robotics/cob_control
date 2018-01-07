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


#ifndef DISTANCE_MANAGER_HPP_
#define DISTANCE_MANAGER_HPP_

#include <vector>
#include <thread>
#include <mutex>
#include <boost/scoped_ptr.hpp>
#include <cob_obstacle_distance/link_to_collision.hpp>

#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Dense>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/CollisionObject.h>
#include "cob_srvs/SetString.h"

#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"
#include "cob_obstacle_distance/shapes_manager.hpp"
#include "cob_obstacle_distance/chainfk_solvers/advanced_chainfksolver_recursive.hpp"
#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"


class DistanceManager
{
    private:
        std::string root_frame_id_;
        std::string chain_base_link_;
        std::string chain_tip_link_;

        boost::scoped_ptr<ShapesManager> obstacle_mgr_;
        boost::scoped_ptr<ShapesManager> object_of_interest_mgr_;

        std::vector<std::thread> self_collision_transform_threads_;
        std::mutex mtx_;
        std::mutex obstacle_mgr_mtx_;
        bool stop_sca_threads_;

        boost::scoped_ptr<AdvancedChainFkSolverVel_recursive> adv_chn_fk_solver_vel_;
        KDL::Chain chain_;

        ros::NodeHandle& nh_;
        ros::Publisher marker_pub_;
        ros::Publisher obstacle_distances_pub_;
        tf::TransformListener tf_listener_;
        Eigen::Affine3d tf_cb_frame_bl_;

        std::vector<std::string> joints_;
        std::vector<std::string> segments_;
        KDL::JntArray last_q_;
        KDL::JntArray last_q_dot_;

        LinkToCollision link_to_collision_;

        static uint32_t seq_nr_;

        /**
         * Build an obstacle from a message containing a mesh.
         * @param msg Msg struct that contains mesh info.
         * @param transform The transformation from a frame in msg header to root_frame_id.
         */
        void buildObstacleMesh(const moveit_msgs::CollisionObject::ConstPtr& msg, const tf::StampedTransform& transform);

        /**
         * Build an obstacle from a message containing a primitive shape.
         * @param msg Msg struct that contains mesh info.
         * @param transform The transformation from a frame in msg header to root_frame_id.
         */
        void buildObstaclePrimitive(const moveit_msgs::CollisionObject::ConstPtr& msg, const tf::StampedTransform& transform);

    public:
        /**
         * @param nh Reference to the ROS node handle.
         */
        DistanceManager(ros::NodeHandle& nh);

        ~DistanceManager();

        inline const std::string getRootFrame() const
        {
            return this->root_frame_id_;
        }

        /**
         * Clears all managed obstacles and objects of interest.
         */
        void clear();

        /**
         * Add a new obstacle to the obstacles that shall be managed.
         * @param s Pointer to an already created MarkerShape that represent an obstacle.
         */
        void addObstacle(const std::string& id, PtrIMarkerShape_t s);

        /**
         * Add a new object of interest that shall be investigated for collisions.
         * @param s Pointer to an already created MarkerShape that represent the object of interest (i.e. shape in reference frame of segment).
         */
        void addObjectOfInterest(const std::string& id, PtrIMarkerShape_t s);

        /**
         * Simply draw all obstacle markers in RVIZ.
         */
        void drawObstacles();

        /**
         * Simply draw all object of interest markers in RVIZ.
         */
        void drawObjectsOfInterest();

        /**
         * Updates the joint states.
         * @param msg Joint state message.
         */
        void jointstateCb(const sensor_msgs::JointState::ConstPtr& msg);

        /**
         * Registers an obstacle via message.
         * @param msg MoveIt CollisionObject message type.
         */
        void registerObstacle(const moveit_msgs::CollisionObject::ConstPtr& msg);

        /**
         * Initialization of ROS robot structure, parameters, publishers and subscribers.
         * @return Error status. If 0 then success.
         */
        int init();

        /**
         * tf Transformation thread between chain_base_link (arm_right_base_link or arm_left_base_link) and the root frame (e.g. base_link)).
         * Runs endless.
         */
        void transform();

        /**
         * Thread that runs endless to listen to transforms for the self collision parts of the robot.
         * This will directly update the self collision obstacle pose.
         * @param link_name The link name of the self collision checking part. Similar to link name in URDF.
         */
        void transformSelfCollisionLinks(const std::string link_name);

        /**
         * Calculate the distances between the objects of interest (reference frames at KDL::segments) and obstacles.
         * Publishes them on the obstacle_distance topic according to robot_namespace (arm_right, arm_left, ...)
         */
        void calculate();

        /**
         * Registers a new link of interest for distance computation.
         * @param request The service request for registration of a new link of interest (e.g. link name)
         * @param response Success message.
         * @return Registration service call successfull or not.
         */
        bool registerLinkOfInterest(cob_srvs::SetString::Request& request,
                                    cob_srvs::SetString::Response& response);

        /**
         * Get method with mutex access on transform data.
         * @return Inverse transformation between chain base and base link.
         */
        Eigen::Affine3d getSynchedCbToBlTransform();
};

#endif /* DISTANCE_MANAGER_HPP_ */
