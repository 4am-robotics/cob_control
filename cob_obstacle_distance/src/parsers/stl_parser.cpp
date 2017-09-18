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


#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <fstream>

#include "cob_obstacle_distance/parsers/stl_parser.hpp"
#include "cob_obstacle_distance/helpers/helper_functions.hpp"

/**
 * Hard coded STL file parsing according to binary file specification in https://en.wikipedia.org/wiki/STL_%28file_format%29.
 */
int8_t StlParser::read(std::vector<TriangleSupport>& tri_vec)
{
    char header_info[80] = "";
    char nTri[4];
    uint32_t nTriLong;

    std::string file_path = this->file_path_;
    if (!boost::filesystem::exists(this->file_path_))
    {
        file_path = resolveURI(this->file_path_);
    }

    std::ifstream myFile(
        file_path.c_str(),
        std::ios::in | std::ios::binary);

    if (!myFile)
    {
        ROS_ERROR_STREAM("Could not read file: " << file_path);
        return -1;
    }

    // read 80 byte header
    myFile.read(header_info, 80);
    ROS_DEBUG_STREAM("header: " << header_info);

    // read 4-byte ulong
    myFile.read(nTri, 4);
    nTriLong = *((uint32_t*) nTri);
    ROS_DEBUG_STREAM("Number of Triangles: " << nTriLong);

    // now read in all the triangles
    for (int i = 0; i < nTriLong; i++)
    {
        char facet[50];
        if (myFile)
        {
            // read one 50-byte triangle
            myFile.read(facet, 50);

            // populate each point of the triangle
            // facet + 12 skips the triangle's unit normal

            TriangleSupport t;
            t.a = this->toVec3f(facet + 12);
            t.b = this->toVec3f(facet + 24);
            t.c = this->toVec3f(facet + 36);

            tri_vec.push_back(t);
        }
        else
        {
            ROS_ERROR_STREAM("File handle is not valid anymore: " << file_path);
            return -2;
        }
    }

    return 0;
}

/**
 * Converter method from position in file to a 3d vector.
 * @param facet Pointer to the current face / triangle in file.
 * @return An fcl::Vec3f containing the 3 vertices describing a triangle.
 */
fcl::Vec3f StlParser::toVec3f(char* facet)
{
    double x = this->toDouble(facet, 0);
    double y = this->toDouble(facet, 4);
    double z = this->toDouble(facet, 8);

    fcl::Vec3f v3(x, y, z);
    return v3;
}

/**
 * Conversion of a 32 bit binary value into double.
 * @param facet Current face / triangle.
 * @param start_idx Index in binary face description.
 * @return Double value.
 */
double StlParser::toDouble(char* facet, uint8_t start_idx)
{
    char f1[4] = {  facet[start_idx],
                    facet[start_idx + 1],
                    facet[start_idx + 2],
                    facet[start_idx + 3]};
    float f_val = *((float*) f1);
    double d_val = static_cast<double>(f_val);
    return d_val;
}

