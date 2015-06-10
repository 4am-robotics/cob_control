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

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <Eigen/LU> // necessary to use several methods on EIGEN Matrices.

#include "cob_obstacle_distance/ObjectOfInterest.h"
#include "cob_obstacle_distance/PartialValues.h"
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

        typedef std::unordered_map<std::string, ObstacleDistancesPublisher> t_map_ObstacleDistancesPublisher;
        typedef std::unordered_map<std::string, ObstacleDistancesPublisher>::iterator t_map_ObstacleDistancesPublisher_iter;
        typedef std::unordered_map<std::string, ObstacleDistancesPublisher>::const_iterator t_map_ObstacleDistancesPublisher_citer;

        boost::scoped_ptr<ShapesManager> obstacleMgr_;
        boost::scoped_ptr<ShapesManager> objectOfInterestMgr_;
        const ros::Publisher& pub_;
        const ros::Publisher& obstacle_dist_pub_;

        ros::Publisher distPub_;

        boost::scoped_ptr<AdvancedChainFkSolverPos_recursive> adChnFkSolverPos_;
        KDL::Chain chain_;
        const std::string chain_base_link_ = "arm_right_base_link"; // TODO: hard coded - change!
        const std::string chain_tip_link_ = "arm_right_7_link"; // TODO: hard coded - change!

        tf::TransformListener tf_listener_;
        tf::StampedTransform cb_transform_bl_;

        KDL::JntArray last_q_;
        KDL::JntArray last_q_dot_;

        std::vector<std::string> joints_;

        KDL::Frame cb_frame_bl_;

        Eigen::Matrix3d chain_base_rot_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt2jac_;

        ros::NodeHandle& nh_;

        t_map_ObstacleDistance obstacle_distances_;

        //t_map_ObstacleDistancesPublisher obstacle_distances_publishers_;


    public:

        DistanceManager(const ros::Publisher& pub,
                        const ros::Publisher& obstacle_dist_pub,
                        ros::NodeHandle& nh);
        ~DistanceManager();

        void clear();
        void addObstacle(tPtrMarkerShapeBase s);
        void addObjectOfInterest(tPtrMarkerShapeBase s);
        void drawObstacles(bool enforceDraw = false);
        void drawObjectsOfInterest(bool enforceDraw = false);
        bool collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2);
        bool getSmallestDistance(cob_obstacle_distance::ObjectOfInterest::Request& request,
                                 cob_obstacle_distance::ObjectOfInterest::Response& response);
        bool getAnotherSmallestDistance(cob_obstacle_distance::PartialValues::Request& request,
                                        cob_obstacle_distance::PartialValues::Response& response);

        void jointstateCb(const sensor_msgs::JointState::ConstPtr& msg);
        int init();
        bool transform();
        void calculate();

        inline int16_t getIndex(const std::string& name)
        {
            int16_t idx = -1;
            for(uint32_t i = 0; i < this->joints_.size(); ++i)
            {
                if(0 == this->joints_.at(i).compare(name))
                {
                    idx = i;
                    break;
                }
            }

            return idx;
        }

        bool registerPointOfInterest(cob_obstacle_distance::Registration::Request& request,
                      cob_obstacle_distance::Registration::Response& response);
};





#endif /* DISTANCE_MANAGER_HPP_ */
