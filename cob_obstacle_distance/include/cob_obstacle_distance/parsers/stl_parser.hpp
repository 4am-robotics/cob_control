/*
 * stl_parser.hpp
 *
 *  Created on: Jul 3, 2015
 *      Author: fxm-mb
 */

#ifndef STL_PARSER_HPP_
#define STL_PARSER_HPP_

#include <stdint.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/math/vec_3f.h>


struct TriangleSupport
{
    fcl::Vec3f a;
    fcl::Vec3f b;
    fcl::Vec3f c;
};

class StlParser
{
    private:
        std::string file_path_;

        double toDouble(char* facet, uint8_t start_idx);

        fcl::Vec3f toVec3f(char* facet);

    public:
        StlParser(const std::string& file_path)
        : file_path_(file_path)
        {

        }

        ~StlParser()
        {

        }

        inline const std::string getFilePath() const
        {
            return this->file_path_;
        }

        int8_t read(std::vector<TriangleSupport>& tri_vec);

        template <typename T>
        int8_t createBVH(fcl::BVHModel<T>& bvh);
};


/**
 * Direct implementation in the header file is necessary only for templated methods!!!.
 * -> Allows implicit usage without one must giving <..>
 */
template <typename T>
int8_t StlParser::createBVH(fcl::BVHModel<T>& bvh)
{
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
}


#endif /* STL_PARSER_HPP_ */
