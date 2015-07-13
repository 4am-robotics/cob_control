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
 *   This header contains a definition of a distance calculation management class.
 *   It provides space for obstacles shapes and object of interest shapes in extra shape management objects.
 *   Provides methods for remote calls.
 ****************************************************************/

#ifndef DISTANCE_MANAGER_HPP_
#define DISTANCE_MANAGER_HPP_

#include <vector>
#include <unordered_map>
#include <boost/scoped_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>

#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <Eigen/LU> // necessary to use several methods on EIGEN Matrices.

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>

#include <kdl/chainfksolvervel_recursive.hpp>
#include <ros/ros.h>

#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"
#include "cob_obstacle_distance/shapes_manager.hpp"
#include "cob_obstacle_distance/chainfk_solvers/advanced_chainfksolver_recursive.hpp"
#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"
#include "cob_obstacle_distance/Registration.h"
#include "cob_obstacle_distance/PredictDistance.h"
#include "cob_obstacle_distance/frame_to_collision.hpp"


class DistanceManager
{
    private:
        typedef std::unordered_map<std::string, ObstacleDistance> MapObstacleDistance_t;
        typedef std::unordered_map<std::string, ObstacleDistance>::iterator MapObstacleDistanceIter_t;
        typedef std::unordered_map<std::string, ObstacleDistance>::const_iterator MapObstacleDistanceConstIter_t;

        std::string root_frame_id_;
        std::string chain_base_link_;
        std::string chain_tip_link_;

        MapObstacleDistance_t obstacle_distances_;
        boost::scoped_ptr<ShapesManager> obstacle_mgr_;
        boost::scoped_ptr<ShapesManager> object_of_interest_mgr_;

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

        static uint32_t seq_nr_;

        FrameToCollision frame2collision_;

        int counter_;

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
         * @param enforceDraw Draw again, also marker already exists.
         */
        void drawObstacles(bool enforceDraw = false);

        /**
         * Simply draw all object of interest markers in RVIZ.
         * @param enforceDraw Draw again, also marker already exists.
         */
        void drawObjectsOfInterest(bool enforceDraw = false);

        /**
         * Check whether a collision between two given shapes has been occurred or not.
         * @param s1 First shape to be checked against second shape.
         * @param s2 Second shape to be checked against first shape.
         */
        bool collide(PtrIMarkerShape_t s1, PtrIMarkerShape_t s2);

        /**
         * Updates the joint states.
         * @param msg Joint state message.
         */
        void jointstateCb(const sensor_msgs::JointState::ConstPtr& msg);

        /**
         * Initialization of ROS robot structure, parameters, publishers and subscribers.
         * @return Error status. If 0 then success.
         */
        int init();

        /**
         * tf Transformation between chain_base_link (arm_right_base_link or arm_left_base_link) and the root frame (e.g. base_link)).
         * @return True if transformation was successfull.
         */
        bool transform();

        /**
         * Calculate the distances between the objects of interest (reference frames at KDL::segments) and obstacles.
         * Publishes them on the obstacle_distance topic according to robot_namespace (arm_right, arm_left, ...)
         */
        void calculate();

        /**
         * Wait loop until a marker topic subscriber (RVIZ) is available.
         * @return True if subscriber could be found.
         */
        bool waitForMarkerSubscriber();

        /**
         * Registers a new point of interest at a given frame id.
         * @param request The service request for registration of a point of interest (i.e. reference frame id corresponding to segment)
         * @param response Success message.
         * @return Registration service call successfull or not.
         */
        bool registerPointOfInterest(cob_obstacle_distance::Registration::Request& request,
                                     cob_obstacle_distance::Registration::Response& response);

        bool predictDistance(cob_obstacle_distance::PredictDistance::Request& request,
                             cob_obstacle_distance::PredictDistance::Response& response);
};

#endif /* DISTANCE_MANAGER_HPP_ */
