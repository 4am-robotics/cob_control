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

#include "cob_obstacle_distance/parsers/mesh_parser.hpp"
#include "cob_obstacle_distance/helpers/helper_functions.hpp"

#define MAX_NUM_MESHES 1

/**
 * Read from a mesh file by using assimp Importer.
 * Iterates through the faces and tries to convert the corresponding vertices into a triangle vector.
 * @param tri_vec Reference to a triangle vector storing the mesh data.
 * @return Success status (0 means ok).
 */
int8_t MeshParser::read(std::vector<TriangleSupport>& tri_vec)
{
    std::string file_path = this->file_path_;
    if (!boost::filesystem::exists(this->file_path_))
    {
        file_path = resolveURI(this->file_path_);
    }

    // Create an instance of the Importer class
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file_path,
                                             0);
    if (!scene)
    {
        ROS_ERROR_STREAM("Assimp::Importer Error: " << importer.GetErrorString());
        return -1;
    }

    if (0 >= scene->mNumMeshes)
    {
        ROS_ERROR("Found no meshes. Check mesh file. Aborting ...");
        return -2;
    }

    if (MAX_NUM_MESHES < scene->mNumMeshes)
    {
        ROS_WARN("Found more than one mesh in mesh file. The current implementation can only process %d mesh!!!", static_cast<uint32_t>(MAX_NUM_MESHES));
    }

    aiMesh** mesh = scene->mMeshes;
    aiVector3D* vertex = mesh[0]->mVertices;  // point to the first vertex

    ROS_DEBUG_STREAM("mesh[0]->mNumVertices: " << mesh[0]->mNumVertices);
    ROS_DEBUG_STREAM("mesh[0]->mNumFaces: " << mesh[0]->mNumFaces);  // num of faces == num of triangles in STL

    // now read in all the triangles
    for (uint32_t i = 0; i < mesh[0]->mNumFaces; ++i)
    {
        // populate each point of the triangle
        TriangleSupport t;
        if (0 != this->toVec3f(i, vertex++, t.a))  // after processing the pointer will be increased to point to the next vertex!
        {
            break;
        }

        if (0 != this->toVec3f(i, vertex++, t.b))
        {
            break;
        }

        if (0 != this->toVec3f(i, vertex++, t.c))
        {
            break;
        }

        tri_vec.push_back(t);
    }

    return 0;
}

/**
 * Uses the current vertex (assimp 3d vector) and converts into fcl::Vec3f.
 * @param num_current_face The number of the current face for error logging.
 * @param vertex Pointer to the current vertex.
 * @param out The fcl vector description.
 * @return Success status (0 means ok).
 */
int8_t MeshParser::toVec3f(uint32_t num_current_face, aiVector3D* vertex, fcl::Vec3f& out)
{
    if (!vertex)
    {
        ROS_ERROR("No valid vertex found at face %d", num_current_face);
        return -3;
    }

    out = fcl::Vec3f((*vertex)[0], (*vertex)[1], (*vertex)[2]);
    return 0;
}

