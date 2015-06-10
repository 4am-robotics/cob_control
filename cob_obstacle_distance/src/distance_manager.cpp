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

#include "fcl/collision_object.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/collision_data.h"

#include <std_msgs/Float64.h>

#include <string.h>
#include <stdint.h>

#include "cob_obstacle_distance/ObstacleDistance.h"
#include "cob_obstacle_distance/ObstacleDistances.h"

#define VEC_X 0
#define VEC_Y 1
#define VEC_Z 2

DistanceManager::DistanceManager(const ros::Publisher& pub,
                                 const ros::Publisher& obstacle_dist_pub,
                                 ros::NodeHandle& nh) : pub_(pub), obstacle_dist_pub_(obstacle_dist_pub), nh_(nh)
{
    this->obstacleMgr_.reset(new ShapesManager(pub));
    this->objectOfInterestMgr_.reset(new ShapesManager(pub));
}

DistanceManager::~DistanceManager()
{
}

void DistanceManager::clear()
{
    this->obstacleMgr_->clear();
    this->objectOfInterestMgr_->clear();
}

void DistanceManager::addObstacle(tPtrMarkerShapeBase s)
{
    this->obstacleMgr_->addShape(s);
}

void DistanceManager::addObjectOfInterest(tPtrMarkerShapeBase s)
{
    this->objectOfInterestMgr_->addShape(s);
}

void DistanceManager::drawObstacles(bool enforceDraw)
{
    this->obstacleMgr_->draw(enforceDraw);
}

void DistanceManager::drawObjectsOfInterest(bool enforceDraw)
{
    this->objectOfInterestMgr_->draw(enforceDraw);
}

bool DistanceManager::collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2)
{
    fcl::CollisionObject x = s1->getCollisionObject();
    fcl::CollisionObject y = s2->getCollisionObject();
    fcl::CollisionResult result;
    fcl::CollisionRequest request(1, true);

    fcl::collide(&x, &y, request, result);

    ROS_INFO_STREAM("isCollision: " << result.isCollision() << std::endl);

    fcl::DistanceResult resultD;
    fcl::DistanceRequest requestD(true, 1.0, 100.0);


    const clock_t begin_time = clock();
    fcl::FCL_REAL dist = fcl::distance(&x, &y, requestD, resultD);
    ROS_INFO_STREAM("Duration of distance calc: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC);

    ROS_INFO_STREAM("Min distance: " << dist << std::endl);
    ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << resultD.nearest_points[0] << std::endl);
    ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (resultD.nearest_points[0] + x.getTranslation()) << std::endl);
    ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << resultD.nearest_points[1] << std::endl);
    ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (resultD.nearest_points[1] + y.getTranslation()) << std::endl);

//            fcl::DistanceResult resultD2;
//            fcl::DistanceRequest requestD2(true, 10.0, 1.0);
//            const clock_t begin_time2 = clock();
//            fcl::FCL_REAL dist2 = fcl::distance(&x, &y, requestD2, resultD2);
//            ROS_INFO_STREAM("Duration of distance calc (with errors activated): " << float( clock () - begin_time2 ) /  CLOCKS_PER_SEC);
//
//            ROS_INFO_STREAM("Min distance: " << dist2 << std::endl);
//            ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << resultD2.nearest_points[0] << std::endl);
//            ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (resultD2.nearest_points[0] + x.getTranslation()) << std::endl);
//            ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << resultD2.nearest_points[1] << std::endl);
//            ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (resultD2.nearest_points[1] + y.getTranslation()) << std::endl);
}


bool DistanceManager::getAnotherSmallestDistance(cob_obstacle_distance::PartialValues::Request& request,
                                          cob_obstacle_distance::PartialValues::Response& response)
{
    ROS_INFO_STREAM("Retrieved service request!!! ");
    bool success = true;
    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    fcl::Box b(0.1, 0.1, 0.1);
    fcl::Sphere s(0.1);
    fcl::Cylinder c(0.1, 0.1);

    chain_base_rot_ << cb_frame_bl_.M.data[0],cb_frame_bl_.M.data[1],cb_frame_bl_.M.data[2],
                       cb_frame_bl_.M.data[3],cb_frame_bl_.M.data[4],cb_frame_bl_.M.data[5],
                       cb_frame_bl_.M.data[6],cb_frame_bl_.M.data[7],cb_frame_bl_.M.data[8];

    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(last_q_.rows());

    /* ******* Transformation part - currently joint 4 ************** */
    KDL::Frame pOut;
    int retVal = adChnFkSolverPos_->JntToCart(last_q_, pOut);
    KDL::Frame joint4pos = adChnFkSolverPos_->getPostureAtJnt(request.jntNumber - 1);

    Eigen::Vector3d base2chainbaseTranslation(cb_frame_bl_.p.x(),
                                              cb_frame_bl_.p.y(),
                                              cb_frame_bl_.p.z());
    Eigen::Vector3d baseLink2armBase = (-1) * chain_base_rot_.inverse() * base2chainbaseTranslation;


    Eigen::Vector3d chainbase2fourthJntPos(joint4pos.p.x(),
                                           joint4pos.p.y(),
                                           joint4pos.p.z());

    //Eigen::Vector3d testo2 = chain_base_rot * base_pos_rot.inverse() * chainbase2fourthJntPos;
    Eigen::Vector3d armBase2JntPos = chain_base_rot_.inverse() * chainbase2fourthJntPos;

    Eigen::Vector3d absJntPosition = baseLink2armBase + armBase2JntPos;

    ROS_INFO_STREAM("Abs joint pos: " << absJntPosition);

    KDL::Jacobian new_jac_chain(last_q_.rows());

    jnt2jac_->JntToJac(last_q_, new_jac_chain, request.jntNumber);

    Eigen::Matrix<double,6,Eigen::Dynamic> jac = new_jac_chain.data;
    Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, jac.cols());

    m << jac.row(0),
         jac.row(1),
         jac.row(2);





    tPtrMarkerShapeBase ooi;
    switch(request.shapeType)
    {
        case visualization_msgs::Marker::CUBE:
            ooi.reset(new MarkerShape<fcl::Box>(b, absJntPosition(0), absJntPosition(1), absJntPosition(2)));
            break;
        case visualization_msgs::Marker::SPHERE:
            ooi.reset(new MarkerShape<fcl::Sphere>(s, absJntPosition(0), absJntPosition(1), absJntPosition(2)));
            break;
        case visualization_msgs::Marker::CYLINDER:
            ooi.reset(new MarkerShape<fcl::Cylinder>(c, absJntPosition(0), absJntPosition(1), absJntPosition(2)));
            break;
        default:
            ROS_ERROR("Failed to process request due to unknown shape type: %d", request.shapeType);
            success = false;
            return false;
    }

    //     visualization_msgs::Marker marker = ooi->getMarker();
    //    pub_.publish(marker);

        ROS_INFO_STREAM("Trying to get collision object!");
        fcl::CollisionObject ooiCo = ooi->getCollisionObject();

        ooi.reset();










    fcl::DistanceResult distResult;
    bool setDistResult = false;
    //boost::shared_ptr<fcl::CollisionGeometry> sptr;
    fcl::CollisionObject resultOCo = ooiCo;

    fcl::FCL_REAL lastDist = std::numeric_limits<fcl::FCL_REAL>::max();

    ROS_INFO_STREAM("Iteration over obstacles");
    for(ShapesManager::iterator it = this->obstacleMgr_->begin(); it != this->obstacleMgr_->end(); ++it)
    {
        fcl::CollisionObject oCo = (*it)->getCollisionObject();
        fcl::DistanceResult tmpResult;
        fcl::DistanceRequest distRequest(true, 1.0, 100.0);

        ROS_INFO_STREAM("Next calculation of distance: ");

        fcl::FCL_REAL dist = fcl::distance(&ooiCo, &oCo, distRequest, tmpResult);
        ROS_INFO_STREAM("Calculated distance: " << dist);

        if (dist < lastDist)
        {
            setDistResult = true;
            distResult = tmpResult;
            resultOCo = oCo;
        }
    }


    //ROS_INFO_STREAM("Found a dist result? " << setDistResult);
    if(setDistResult)
    {
        //ROS_INFO_STREAM("Min distance: " << distResult.min_distance << std::endl);
        //ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << distResult.nearest_points[0] << std::endl);
        //ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (distResult.nearest_points[0] + ooiCo.getTranslation()) << std::endl);
        //ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << distResult.nearest_points[1] << std::endl);
        //ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (distResult.nearest_points[1] + resultOCo.getTranslation()) << std::endl);

        ROS_INFO_STREAM("Minimum distance: " << distResult.min_distance);
        //response.distance = static_cast<double>(distResult.min_distance);

        fcl::Vec3f t = distResult.nearest_points[1] + resultOCo.getTranslation(); // Translation from "base_link" frame!!!
        fcl::Quaternion3f q = resultOCo.getQuatRotation();

//        response.obstacle.orientation.w = q.getW();
//        response.obstacle.orientation.x = q.getX();
//        response.obstacle.orientation.y = q.getY();
//        response.obstacle.orientation.z = q.getZ();
//
//        response.obstacle.position.x = t[0];
//        response.obstacle.position.y = t[1];
//        response.obstacle.position.z = t[2];



        Eigen::Vector3d obstVector(t[0], t[1], t[2]);
        Eigen::Vector3d distVector = absJntPosition - obstVector;


        if (distResult.min_distance > 0.0)
        {
            partialValues =  2.0 * ((distResult.min_distance - request.activationDistance) / distResult.min_distance) * m.transpose() * distVector;
        }
        else
        {
            partialValues =  2.0 * (((1.0e-9) - request.activationDistance) / (1.0e-9)) * m.transpose() * distVector;
        }

        std_msgs::Float64 f64MinDist;
        f64MinDist.data = static_cast<double>(distResult.min_distance);
        this->distPub_.publish(f64MinDist);


        for(int i = 0; i < partialValues.rows(); ++i)
        {
            response.partialValues.push_back((double) partialValues(i));
        }


    }



//    if (success)
//    {
//        this->addObjectOfInterest(ooi); // TODO: generate id or slot to return? and adapt later because pose changes with time!
//    }

    return true;


}




bool DistanceManager::getSmallestDistance(cob_obstacle_distance::ObjectOfInterest::Request& request,
                                          cob_obstacle_distance::ObjectOfInterest::Response& response)
{

    ROS_INFO_STREAM("Retrieved service request!!! ");
    response = cob_obstacle_distance::ObjectOfInterest::Response();
    bool success = true;
    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    fcl::Box b(0.1, 0.1, 0.1);
    fcl::Sphere s(0.1);
    fcl::Cylinder c(0.1, 0.1);

    tPtrMarkerShapeBase ooi;
    switch(request.shapeType)
    {
        case visualization_msgs::Marker::CUBE:
            ooi.reset(new MarkerShape<fcl::Box>(b, request.p, col));
            break;
        case visualization_msgs::Marker::SPHERE:
            ooi.reset(new MarkerShape<fcl::Sphere>(s, request.p, col));
            break;
        case visualization_msgs::Marker::CYLINDER:
            ooi.reset(new MarkerShape<fcl::Cylinder>(c, request.p, col));
            break;
        default:
            ROS_ERROR("Failed to process request due to unknown shape type: %d", request.shapeType);
            success = false;
            return false;
    }

//     visualization_msgs::Marker marker = ooi->getMarker();
//    pub_.publish(marker);

    ROS_INFO_STREAM("Trying to get collision object!");
    fcl::CollisionObject ooiCo = ooi->getCollisionObject();

    ooi.reset();

    fcl::DistanceResult distResult;
    bool setDistResult = false;
    //boost::shared_ptr<fcl::CollisionGeometry> sptr;
    fcl::CollisionObject resultOCo = ooiCo;

    fcl::FCL_REAL lastDist = std::numeric_limits<fcl::FCL_REAL>::max();

    ROS_INFO_STREAM("Iteration over obstacles");
    for(ShapesManager::iterator it = this->obstacleMgr_->begin(); it != this->obstacleMgr_->end(); ++it)
    {
        fcl::CollisionObject oCo = (*it)->getCollisionObject();
        fcl::DistanceResult tmpResult;
        fcl::DistanceRequest distRequest(true, 1.0, 100.0);

        ROS_INFO_STREAM("Next calculation of distance: ");

        fcl::FCL_REAL dist = fcl::distance(&ooiCo, &oCo, distRequest, tmpResult);
        ROS_INFO_STREAM("Calculated distance: " << dist);

        if (dist < lastDist)
        {
            setDistResult = true;
            distResult = tmpResult;
            resultOCo = oCo;
        }
    }


    //ROS_INFO_STREAM("Found a dist result? " << setDistResult);
    if(setDistResult)
    {
        //ROS_INFO_STREAM("Min distance: " << distResult.min_distance << std::endl);
        //ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << distResult.nearest_points[0] << std::endl);
        //ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (distResult.nearest_points[0] + ooiCo.getTranslation()) << std::endl);
        //ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << distResult.nearest_points[1] << std::endl);
        //ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (distResult.nearest_points[1] + resultOCo.getTranslation()) << std::endl);

        ROS_INFO_STREAM("Minimum distance: " << distResult.min_distance);
        response.distance = static_cast<double>(distResult.min_distance);

        fcl::Vec3f t = distResult.nearest_points[1] + resultOCo.getTranslation(); // Translation from "base_link" frame!!!
        fcl::Quaternion3f q = resultOCo.getQuatRotation();

        response.obstacle.orientation.w = q.getW();
        response.obstacle.orientation.x = q.getX();
        response.obstacle.orientation.y = q.getY();
        response.obstacle.orientation.z = q.getZ();

        response.obstacle.position.x = t[VEC_X];
        response.obstacle.position.y = t[VEC_Y];
        response.obstacle.position.z = t[VEC_Z];
    }

    std_msgs::Float64 f64MinDist;
    f64MinDist.data = static_cast<double>(response.distance);
    this->distPub_.publish(f64MinDist);

//    if (success)
//    {
//        this->addObjectOfInterest(ooi); // TODO: generate id or slot to return? and adapt later because pose changes with time!
//    }


    ROS_INFO_STREAM("Return " << setDistResult);

    return true;
}

int DistanceManager::init()
{
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile("/home/fxm-mb/projects_ws/src/cob_control/cob_obstacle_distance/testdata/robot_description.xml",
                                  my_tree)) // TODO: HARD CODE - CHANGE
    {
        ROS_ERROR("Failed to construct kdl tree");
        return -2;
    }

    my_tree.getChain(chain_base_link_, chain_tip_link_, chain_);
    if(chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return -3;
    }

    adChnFkSolverPos_.reset(new AdvancedChainFkSolverPos_recursive(chain_));

    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());


    joints_.push_back("arm_right_1_joint");
    joints_.push_back("arm_right_2_joint");
    joints_.push_back("arm_right_3_joint");
    joints_.push_back("arm_right_4_joint");
    joints_.push_back("arm_right_5_joint");
    joints_.push_back("arm_right_6_joint");
    joints_.push_back("arm_right_7_joint");

    jnt2jac_.reset(new KDL::ChainJntToJacSolver(chain_));

    distPub_ = nh_.advertise<std_msgs::Float64>("minimal_distance", 1);

//    for(std::vector<std::string>::const_iterator it = this->robot_namespaces_.cbegin(); it != this->robot_namespaces_.cend(); ++it)
//    {
//        this->obstacle_distances_publishers_[*it].pub = nh_.advertise<cob_obstacle_distance::ObstacleDistances>(*it, 1);
//    }

    return 0;
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

        std::string poi_name = it->first;

        uint32_t idx = this->getIndex(poi_name);
        uint32_t jntNumber = idx + 1;

        chain_base_rot_ << cb_frame_bl_.M.data[0],cb_frame_bl_.M.data[1],cb_frame_bl_.M.data[2],
                           cb_frame_bl_.M.data[3],cb_frame_bl_.M.data[4],cb_frame_bl_.M.data[5],
                           cb_frame_bl_.M.data[6],cb_frame_bl_.M.data[7],cb_frame_bl_.M.data[8];

        Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(last_q_.rows());

        /* ******* Transformation part ************** */
        KDL::Frame pOut;
        int retVal = adChnFkSolverPos_->JntToCart(last_q_, pOut);
        KDL::Frame joint4pos = adChnFkSolverPos_->getPostureAtJnt(idx);

        Eigen::Vector3d base2chainbaseTranslation(cb_frame_bl_.p.x(),
                                                  cb_frame_bl_.p.y(),
                                                  cb_frame_bl_.p.z());
        Eigen::Vector3d baseLink2armBase = (-1) * chain_base_rot_.inverse() * base2chainbaseTranslation;


        Eigen::Vector3d chainbase2fourthJntPos(joint4pos.p.x(),
                                               joint4pos.p.y(),
                                               joint4pos.p.z());

        Eigen::Vector3d armBase2JntPos = chain_base_rot_.inverse() * chainbase2fourthJntPos;
        Eigen::Vector3d absJntPosition = baseLink2armBase + armBase2JntPos;

        fcl::Box b(0.1, 0.1, 0.1);
        fcl::Sphere s(0.1);
        fcl::Cylinder c(0.1, 0.1);
        tPtrMarkerShapeBase ooi;
        switch(it->second.shape_type)
        {
            case visualization_msgs::Marker::CUBE:
                ooi.reset(new MarkerShape<fcl::Box>(b, absJntPosition(0), absJntPosition(1), absJntPosition(2)));
                break;
            case visualization_msgs::Marker::SPHERE:
                ooi.reset(new MarkerShape<fcl::Sphere>(s, absJntPosition(0), absJntPosition(1), absJntPosition(2)));
                break;
            case visualization_msgs::Marker::CYLINDER:
                ooi.reset(new MarkerShape<fcl::Cylinder>(c, absJntPosition(0), absJntPosition(1), absJntPosition(2)));
                break;
            default:
               ROS_ERROR("Failed to process request due to unknown shape type: %d", it->second.shape_type);
               return;
        }

        ROS_INFO_STREAM("Trying to get collision object!");
        fcl::CollisionObject ooiCo = ooi->getCollisionObject();

        ooi.reset();

        fcl::DistanceResult distResult;
        bool setDistResult = false;
        fcl::CollisionObject resultOCo = ooiCo;

        fcl::FCL_REAL lastDist = std::numeric_limits<fcl::FCL_REAL>::max();

        ROS_INFO_STREAM("Iteration over obstacles for poi: " << poi_name);
        for(ShapesManager::iterator it = this->obstacleMgr_->begin(); it != this->obstacleMgr_->end(); ++it)
        {
            fcl::CollisionObject oCo = (*it)->getCollisionObject();
            fcl::DistanceResult tmpResult;
            fcl::DistanceRequest distRequest(true, 1.0, 100.0);

            ROS_INFO_STREAM("Next calculation of distance: ");

            fcl::FCL_REAL dist = fcl::distance(&ooiCo, &oCo, distRequest, tmpResult);
            ROS_INFO_STREAM("Calculated distance: " << dist);

            if (dist < lastDist)
            {
                setDistResult = true;
                distResult = tmpResult;
                resultOCo = oCo;
            }
        }

        //ROS_INFO_STREAM("Found a dist result? " << setDistResult);
        if(setDistResult)
        {
            ROS_INFO_STREAM("Minimum distance: " << distResult.min_distance);
            fcl::Vec3f t = distResult.nearest_points[1] + resultOCo.getTranslation(); // Translation from "base_link" frame!!!
            fcl::Quaternion3f q = resultOCo.getQuatRotation();

            Eigen::Vector3d obstVector(t[VEC_X],
                                       t[VEC_Y],
                                       t[VEC_Z]);



            ROS_INFO_STREAM("absJntPosition: " << absJntPosition);
            ROS_INFO_STREAM("obstVector: " << obstVector);

            Eigen::Vector3d distVector = absJntPosition - obstVector;

            cob_obstacle_distance::ObstacleDistance od_msg;
            od_msg.distance = distResult.min_distance;
            od_msg.distance_vector.x = distVector(VEC_X);
            od_msg.distance_vector.y = distVector(VEC_Y);
            od_msg.distance_vector.z = distVector(VEC_Z);
            od_msg.id.frame_id = poi_name;
            od_msg.id.stamp = ros::Time::now();

            obstacle_distances.distances.push_back(od_msg);
        }
    }

    if(obstacle_distances.distances.size() > 0)
    {
        this->obstacle_dist_pub_.publish(obstacle_distances);
    }
}


bool DistanceManager::transform()
{
    // ROS_INFO("Transform");
    bool success = true;
    try
    {
        //ROS_INFO("waitForTransform");
        tf_listener_.waitForTransform(chain_base_link_, "base_link", ros::Time(0), ros::Duration(0.5));
        //ROS_INFO("lookupTransform");
        tf_listener_.lookupTransform(chain_base_link_, "base_link", ros::Time(0), cb_transform_bl_);

        cb_frame_bl_.p = KDL::Vector(cb_transform_bl_.getOrigin().x(),
                                    cb_transform_bl_.getOrigin().y(),
                                    cb_transform_bl_.getOrigin().z());
        cb_frame_bl_.M = KDL::Rotation::Quaternion(cb_transform_bl_.getRotation().x(),
                                                  cb_transform_bl_.getRotation().y(),
                                                  cb_transform_bl_.getRotation().z(),
                                                  cb_transform_bl_.getRotation().w());
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
    if (this->obstacle_distances_.count(request.joint_name))
    {
        response.success = true;
        response.message = "Element " + request.joint_name + " already existent.";
    }
    else
    {
        try
        {
            // TODO: Robot namespace is not necessary anymore.
            this->obstacle_distances_[request.joint_name] = ObstacleDistance(request.robot_namespace, request.shape_type);
            response.success = true;
            response.message = "Successfully inserted element " + request.joint_name + ".";
            ROS_INFO_STREAM("For robot namespace: " << this->obstacle_distances_[request.joint_name].robot_namespace);

        }
        catch(...)
        {
            response.success = false;
            response.message = "Failed to insert element " + request.joint_name + " of robot namespace " + request.robot_namespace + "!";
            ROS_ERROR_STREAM(response.message);
        }
    }

    return true;
}
