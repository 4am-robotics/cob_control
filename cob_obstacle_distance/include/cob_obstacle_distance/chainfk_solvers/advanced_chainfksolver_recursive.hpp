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


#ifndef ADVANCED_CHAINFKSOLVERPOS_RECURSIVE_H_
#define ADVANCED_CHAINFKSOLVERPOS_RECURSIVE_H_

#include <kdl/chainfksolver.hpp>
#include <vector>
#include <stdint.h>

typedef std::vector<KDL::Frame> FrameVector_t;
typedef std::vector<KDL::FrameVel> FrameVelVector_t;

/**
 * Implementation of a recursive forward position kinematics
 * algorithm to calculate the position transformation from joint
 * space to Cartesian space of a general kinematic chain (KDL::Chain).
 *
 * @ingroup KinematicFamily
 */
class AdvancedChainFkSolverPos_recursive : public KDL::ChainFkSolverPos
{
    public:
        AdvancedChainFkSolverPos_recursive(const KDL::Chain& chain);
        ~AdvancedChainFkSolverPos_recursive();

        /**
         * @param q_in Joint states.
         * @param p_out The output frame of the given segment or end-effector.
         * @param seg_nr The max. segment nr until calculation should stop.
         * @return An error code (0 == success)
         */
        virtual int JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out, int seg_nr = -1);

        /**
         * @param seg_idx Index of the segment starting with 0.
         * @return The reference frame of the segment.
         */
        KDL::Frame getFrameAtSegment(uint16_t seg_idx) const;

        void dumpAllSegmentPostures() const;

    private:
        const KDL::Chain& chain_;
        FrameVector_t segment_pos_;

};


/**
 * Implementation of a recursive forward velocities kinematics
 * algorithm to calculate the velocity transformation from joint
 * space to Cartesian space of a general kinematic chain (KDL::Chain).
 *
 * @ingroup KinematicFamily
 */
class AdvancedChainFkSolverVel_recursive : public KDL::ChainFkSolverVel
{
    public:
        AdvancedChainFkSolverVel_recursive(const KDL::Chain& chain);
        ~AdvancedChainFkSolverVel_recursive();

        /**
         * @param q_in Joint states.
         * @param p_out The output frame of the given segment or end-effector.
         * @param seg_nr The max. segment nr until calculation should stop.
         * @return An error code (0 == success)
         */
        virtual int JntToCart(const KDL::JntArrayVel& q_in, KDL::FrameVel& out, int seg_nr = -1);
        virtual int JntToCart(const KDL::JntArrayVel& q_in, std::vector<KDL::FrameVel>& out, int segmentNr = -1);

        /**
         * @param seg_idx Index of the segment starting with 0.
         * @return The reference frame of the segment.
         */
        KDL::FrameVel getFrameVelAtSegment(uint16_t seg_idx) const;
        virtual void updateInternalDataStructures(){};

    private:
        const KDL::Chain& chain_;
        FrameVelVector_t segment_vel_;

};


#endif /* ADVANCED_CHAINFKSOLVERPOS_RECURSIVE_H_ */
