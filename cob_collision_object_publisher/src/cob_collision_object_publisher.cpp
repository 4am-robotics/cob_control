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
 *   ROS package name: cob_collision_object_publisher
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Main to initialize a node and start publishing markers.
 *
 ****************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_object.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/collision_data.h"

#include <ctime>

#include "cob_collision_object_publisher/ObjectOfInterest.h"
#include "cob_collision_object_publisher/marker_shapes.hpp"


typedef boost::shared_ptr< IMarkerShape > tPtrMarkerShapeBase;

class ShapesManager
{
    private:
        std::vector<tPtrMarkerShapeBase> shapes_;
        typedef std::vector<tPtrMarkerShapeBase>::iterator tcIter;
        ros::Publisher pub_;

    public:

        ShapesManager(ros::Publisher &pub)
        {
            this->pub_ = pub;
        }

        ~ShapesManager()
        {
            this->clear();
        }

        void addShape(tPtrMarkerShapeBase s)
        {
            this->shapes_.push_back(s);
        }

        void draw(bool enforceDraw = false)
        {
            for(tcIter iter = shapes_.begin(); iter != shapes_.end(); ++iter)
            {
                if(!((*iter)->isDrawn()) || enforceDraw)
                {
                    ROS_INFO_STREAM("Publishing marker #" << (*iter)->getId() << std::endl);
                    visualization_msgs::Marker marker = (*iter)->getMarker();
                    this->pub_.publish(marker);
                    (*iter)->setDrawn();
                }
            }
        }

        void clear()
        {
            this->shapes_.clear();
        }
};


class DistanceManager
{
    public:

        DistanceManager(ros::Publisher &pub)
        {
            obstacleMgr_.reset(new ShapesManager(pub));
            objectOfInterestMgr_.reset(new ShapesManager(pub));
        }

        ~DistanceManager()
        {
        }

        void clear()
        {
        }

        void addObstacle(tPtrMarkerShapeBase s)
        {
            this->obstacleMgr_->addShape(s);
        }

        void addObjectOfInterest(tPtrMarkerShapeBase s)
        {
            this->objectOfInterestMgr_->addShape(s);
        }

        void drawObstacles(bool enforceDraw = false)
        {
            this->obstacleMgr_->draw(enforceDraw);
        }

        void drawObjectsOfInterest(bool enforceDraw = false)
        {
            this->objectOfInterestMgr_->draw(enforceDraw);
        }

        bool collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2)
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

        bool registerObjectOfInterest(cob_collision_object_publisher::ObjectOfInterest::Request& request,
                                      cob_collision_object_publisher::ObjectOfInterest::Response& response)
        {
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
                    success = false;
            }

            if (success)
            {
                this->addObjectOfInterest(ooi); // TODO: generate id or slot to return? and adapt later because pose changes with time!
            }

            return success;
        }


    private:
        boost::scoped_ptr<ShapesManager> obstacleMgr_;
        boost::scoped_ptr<ShapesManager> objectOfInterestMgr_;
};


bool waitForSubscriber(ros::Publisher &pub)
{
    while (pub.getNumSubscribers() < 1)
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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_collision_object_publisher");
    ros::NodeHandle nh;
    ros::Rate r(1.0);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);



    ROS_INFO("Starting basic_shapes ...\r\n");


    fcl::Box b(1, 1, 1);

    DistanceManager sm(marker_pub);

    ros::ServiceServer service = nh.advertiseService("registerObjectOfInterest", &DistanceManager::registerObjectOfInterest, &sm);

//    tPtrMarkerShapeBase sptr_Cube(new MarkerShape<fcl::Box>(b, 1.0, 1.0, 1.0));
//    tPtrMarkerShapeBase sptr_Cube2(new MarkerShape<fcl::Box>(b, -1.0, -1.0, 1.0));
//    tPtrMarkerShapeBase sptr_Sphere(new MarkerShape<fcl::Sphere>(1.0, -1.0, -1.0));
//    tPtrMarkerShapeBase sptr_Cyl(new MarkerShape<fcl::Cylinder>(-1.0, 1.0, -1.0));

    tPtrMarkerShapeBase sptr_Cube(new MarkerShape<fcl::Box>(b, 1.0, 1.0, 1.0));
    tPtrMarkerShapeBase sptr_Cube2(new MarkerShape<fcl::Box>(b, -1.0, -1.0, 1.0));
    tPtrMarkerShapeBase sptr_Sphere(new MarkerShape<fcl::Sphere>(1.0, -1.0, -1.0));
    tPtrMarkerShapeBase sptr_Cyl(new MarkerShape<fcl::Cylinder>(-1.0, 1.0, -1.0));

    //IMarkerShape *ims = new XMarkerShapeBase<fcl::Box>(b, 1.0, 1.0, 1.0);
    // XMarkerShapeBase<fcl::Box> testo(b, 1.0, 1.0, 1.0);


    sm.addObstacle(sptr_Cube);
    sm.addObstacle(sptr_Cube2);
    sm.addObstacle(sptr_Sphere);
    sm.addObstacle(sptr_Cyl);
    //sm.addShape(sphere);
    //sm.addShape(cylinder);

    sm.collide(sptr_Cube, sptr_Cube2);

    while (ros::ok() && nh.ok())
    {
      if(!waitForSubscriber(marker_pub))
      {
          break;
      }

      ROS_INFO_ONCE("Subscriber to the marker has been created");
      sm.drawObstacles();

      r.sleep();
    }


    ROS_INFO_ONCE("Clean up!!!");
    sm.clear();

    sptr_Cube2.reset();
    sptr_Cube.reset();
    sptr_Sphere.reset();
    sptr_Cyl.reset();

    return 0;
}

