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
