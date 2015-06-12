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

#include "cob_obstacle_distance/marker_shapes.hpp"
#include "cob_obstacle_distance/shapes_manager.hpp"
#include "cob_obstacle_distance/chainfk_solvers/advanced_chainfksolverpos_recursive.h"
#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"
#include "cob_obstacle_distance/Registration.h"

#include <ros/ros.h>

class DistanceManager
{
    private:
        typedef std::unordered_map<std::string, ObstacleDistance> t_map_ObstacleDistance;
        typedef std::unordered_map<std::string, ObstacleDistance>::iterator t_map_ObstacleDistance_iter;
        typedef std::unordered_map<std::string, ObstacleDistance>::const_iterator t_map_ObstacleDistance_citer;

        std::string chain_base_link_;
        std::string chain_tip_link_;
        std::string robo_namespace_;

        t_map_ObstacleDistance obstacle_distances_;
        boost::scoped_ptr<ShapesManager> obstacle_mgr_;
        boost::scoped_ptr<ShapesManager> object_of_interest_mgr_;

        boost::scoped_ptr<AdvancedChainFkSolverPos_recursive> adv_chn_fk_solver_pos_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt2jac_;
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

    public:
        DistanceManager(ros::NodeHandle& nh);

        ~DistanceManager();

        void clear();
        void addObstacle(tPtrMarkerShapeBase s);
        void addObjectOfInterest(tPtrMarkerShapeBase s);
        void drawObstacles(bool enforceDraw = false);
        void drawObjectsOfInterest(bool enforceDraw = false);
        bool collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2);
        void jointstateCb(const sensor_msgs::JointState::ConstPtr& msg);
        int init();
        bool transform();
        void calculate();
        bool waitForMarkerSubscriber();
        bool registerPointOfInterest(cob_obstacle_distance::Registration::Request& request,
                                     cob_obstacle_distance::Registration::Response& response);

        inline std::string getRoboNamespace() const
        {
            return this->robo_namespace_;
        }

        static int getMarkerShape(uint32_t shape_type, const Eigen::Vector3d& abs_pos, tPtrMarkerShapeBase& segment_of_interest_marker_shape);
};

#endif /* DISTANCE_MANAGER_HPP_ */
