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
