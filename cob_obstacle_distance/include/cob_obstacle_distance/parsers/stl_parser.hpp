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
 *   Definition of a class for an STL file parser.
 *
 */
#ifndef STL_PARSER_HPP_
#define STL_PARSER_HPP_

#include "cob_obstacle_distance/parsers/parser_base.hpp"

class StlParser : public ParserBase
{
    private:

        double toDouble(char* facet, uint8_t start_idx);

        fcl::Vec3f toVec3f(char* facet);

    public:
        StlParser(const std::string& file_path)
        : ParserBase(file_path)
        {

        }

        virtual ~StlParser()
        {

        }

        int8_t read(std::vector<TriangleSupport>& tri_vec);
};

#endif /* STL_PARSER_HPP_ */
