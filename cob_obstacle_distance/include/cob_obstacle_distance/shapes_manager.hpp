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
#include <unordered_map>

#include "cob_obstacle_distance/marker_shapes_interface.hpp"

/// Class to manage fcl::Shapes and connect with RVIZ marker type.
class ShapesManager
{
    private:
        std::unordered_map<std::string, Ptr_IMarkerShape_t> shapes_2_;
        const ros::Publisher& pub_;

    public:
        typedef std::unordered_map<std::string, Ptr_IMarkerShape_t>::iterator Map_iter_t;
        typedef std::unordered_map<std::string, Ptr_IMarkerShape_t>::const_iterator Map_const_iterator_t;

        /**
         * Ctor
         * @param pub Publisher on a marker topic (visualize marker in RVIZ).
         */
        ShapesManager(const ros::Publisher &pub);

        ~ShapesManager();

        /**
         * Adds a new shape to the manager.
         * @param id Key to access the marker shape.
         * @param s Pointer to an already created marker shape.
         */
        void addShape(const std::string& id, Ptr_IMarkerShape_t s);

        /**
         * Tries to return the marker shape if ID is correct.
         * @param id Key to access the marker shape.
         * @param s Pointer to an already created marker shape.
         * @return State of success.
         */
        bool getShape(const std::string& id, Ptr_IMarkerShape_t& s);


        /**
         * Draw the marker managed by the ShapesManager
         * @param enforce_draw Enforce drawing also in case of marker has already been drawn.
         */
        void draw(bool enforce_draw = false);

        /**
         * Clear the managed shapes.
         */
        void clear();

        Map_iter_t begin() {return this->shapes_2_.begin(); }
        Map_const_iterator_t begin() const {return this->shapes_2_.begin(); }
        Map_iter_t end() {return this->shapes_2_.end(); }
        Map_const_iterator_t end() const {return this->shapes_2_.end(); }
};

#endif /* SHAPES_MANAGER_HPP_ */
