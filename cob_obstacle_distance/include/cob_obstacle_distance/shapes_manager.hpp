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
 *   This header contains a definition of a simple shapes management class.
 *   The class collects shapes and executes actions on them.
 *   Uses a publisher to publish the manages shapes to rviz.
 ****************************************************************/
#ifndef SHAPES_MANAGER_HPP_
#define SHAPES_MANAGER_HPP_

#include <ros/ros.h>
#include <vector>

#include "cob_obstacle_distance/marker_shapes_interface.hpp"

class ShapesManager
{
    private:
        std::vector<tPtrMarkerShapeBase> shapes_;
        typedef std::vector<tPtrMarkerShapeBase>::iterator tcIter;
        const ros::Publisher& pub_;

    public:
        typedef std::vector<tPtrMarkerShapeBase>::iterator iterator;
        typedef std::vector<tPtrMarkerShapeBase>::const_iterator const_iterator;

        ShapesManager(const ros::Publisher &pub);
        ~ShapesManager();

        void addShape(tPtrMarkerShapeBase s);
        void draw(bool enforceDraw = false);
        void clear();

        iterator begin() {return this->shapes_.begin(); }
        const_iterator begin() const {return this->shapes_.begin(); }
        iterator end() {return this->shapes_.end(); }
        const_iterator end() const {return this->shapes_.end(); }
};

#endif /* SHAPES_MANAGER_HPP_ */
