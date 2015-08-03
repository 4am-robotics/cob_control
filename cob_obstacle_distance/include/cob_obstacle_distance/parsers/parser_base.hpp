/*
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
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
 * \date Date of creation: July, 2015
 *
 * \brief
 *   Definition of a base class for mesh file parsers.
 *
 */
#ifndef PARSER_BASE_HPP_
#define PARSER_BASE_HPP_

#include <stdint.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/math/vec_3f.h>

#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"

class ParserBase
{
    protected:
        std::string file_path_;

    public:
        /**
         * Base class ctor
         * @param file_path Can be an URI name (e.g. package:// ...) or a full path.
         */
        ParserBase(const std::string& file_path)
        : file_path_(file_path)
        {

        }

        virtual ~ParserBase()
        {

        }

        /**
         * Return the member file path.
         * @return A const string containing the member file path.
         */
        inline const std::string getFilePath() const
        {
            return this->file_path_;
        }

        /**
         * Tries to read from the given file path and fills a triangle vector.
         * @param tri_vec A vector of triangles that shall be filled by the read method.
         * @return Success status (0 means ok)
         */
        virtual int8_t read(std::vector<TriangleSupport>& tri_vec) = 0;

        template <typename T>
        int8_t createBVH(fcl::BVHModel<T>& bvh);
};


/**
 * Direct implementation in the header file is necessary only for templated methods!!!.
 * -> Allows implicit usage without one must giving <..>
 * @param bvh A reference to a fcl::BVHModel instance that shall be filled with triangles.
 * @return Success status (0 means success)
 */
template <typename T>
int8_t ParserBase::createBVH(fcl::BVHModel<T>& bvh)
{
    int8_t success = -1;
    std::vector<TriangleSupport> tri_vec;
    if(0 == this->read(tri_vec))
    {
        bvh.beginModel();
        for(TriangleSupport t :  tri_vec)
        {
            bvh.addTriangle(t.a, t.b, t.c);
        }

        bvh.endModel();
        bvh.computeLocalAABB();
        success = 0;
    }

    return success;
}


#endif /* PARSER_BASE_HPP_ */
