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

/// Class to manage fcl::Shapes and connect with RVIZ marker type.
class ShapesManager
{
    private:
        std::vector<tPtrMarkerShapeBase> shapes_;
        typedef std::vector<tPtrMarkerShapeBase>::iterator tcIter;
        const ros::Publisher& pub_;

    public:
        typedef std::vector<tPtrMarkerShapeBase>::iterator iterator;
        typedef std::vector<tPtrMarkerShapeBase>::const_iterator const_iterator;

        /**
         * Ctor
         * @param pub Publisher on a marker topic (visualize marker in RVIZ).
         */
        ShapesManager(const ros::Publisher &pub);

        ~ShapesManager();

        /**
         * Adds a new shape to the manager.
         * @param s Pointer to an already created marker shape.
         */
        void addShape(tPtrMarkerShapeBase s);

        /**
         * Draw the marker managed by the ShapesManager
         * @param enforce_draw Enforce drawing also in case of marker has already been drawn.
         */
        void draw(bool enforce_draw = false);

        /**
         * Clear the managed shapes.
         */
        void clear();

        iterator begin() {return this->shapes_.begin(); }
        const_iterator begin() const {return this->shapes_.begin(); }
        iterator end() {return this->shapes_.end(); }
        const_iterator end() const {return this->shapes_.end(); }
};

#endif /* SHAPES_MANAGER_HPP_ */
