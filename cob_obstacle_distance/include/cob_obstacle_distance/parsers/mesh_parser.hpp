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
 *   Definition of a class for a generic mesh file parser using the assimp library.
 *
 */
#ifndef MESH_PARSER_HPP_
#define MESH_PARSER_HPP_

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "cob_obstacle_distance/parsers/parser_base.hpp"

class MeshParser : public ParserBase
{
    private:
        int8_t toVec3f(uint32_t num_current_face, aiVector3D* vertex, fcl::Vec3f& out);

    public:
        MeshParser(const std::string& file_path)
        : ParserBase(file_path)
        {

        }

        virtual ~MeshParser()
        {

        }

        int8_t read(std::vector<TriangleSupport>& tri_vec);
};

#endif /* MESH_PARSER_HPP_ */
