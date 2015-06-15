/*!
 *****************************************************************
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
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Header declaring an implementation for ChainFkSolverPos.
 *
 ****************************************************************/

#ifndef ADVANCED_CHAINFKSOLVERPOS_RECURSIVE_H_
#define ADVANCED_CHAINFKSOLVERPOS_RECURSIVE_H_

#include <kdl/chainfksolver.hpp>
#include <vector>
#include <stdint.h>

typedef std::vector<KDL::Frame> tFrameVector;

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
        tFrameVector segment_pos_;

};

#endif /* ADVANCED_CHAINFKSOLVERPOS_RECURSIVE_H_ */
