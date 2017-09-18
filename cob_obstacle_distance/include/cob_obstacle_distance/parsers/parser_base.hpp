/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

        template <typename T>
        int8_t createBVH(std::shared_ptr<fcl::BVHModel<T> > ptr_bvh);
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

template <typename T>
int8_t ParserBase::createBVH(std::shared_ptr<fcl::BVHModel<T> > ptr_bvh)
{
    int8_t success = -1;
    std::vector<TriangleSupport> tri_vec;
    if(0 == this->read(tri_vec))
    {
        ptr_bvh->beginModel();
        for(TriangleSupport t :  tri_vec)
        {
            ptr_bvh->addTriangle(t.a, t.b, t.c);
        }

        ptr_bvh->endModel();
        ptr_bvh->computeLocalAABB();
        success = 0;
    }

    return success;
}


#endif /* PARSER_BASE_HPP_ */
