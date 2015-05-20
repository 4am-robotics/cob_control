/*
 * advanced_chainfksolverpos_recursive.h
 *
 *  Created on: May 20, 2015
 *      Author: fxm-mb
 */


#include "cob_twist_controller/chainfk_solvers/advanced_chainfksolverpos_recursive.h"
#include <ros/ros.h>

AdvancedChainFkSolverPos_recursive::AdvancedChainFkSolverPos_recursive(const KDL::Chain& _chain):
    chain_(_chain)
{
}

int AdvancedChainFkSolverPos_recursive::JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out, int seg_nr)
{
    unsigned int segmentNr;
    if(seg_nr < 0)
    {
        segmentNr = this->chain_.getNrOfSegments();
    }
    else
    {
        segmentNr = seg_nr;
    }

    p_out = KDL::Frame::Identity();

    if(q_in.rows() != this->chain_.getNrOfJoints())
    {
        return -1;
    }
    else if(segmentNr > this->chain_.getNrOfSegments())
    {
        return -1;
    }
    else
    {
        this->jntPos_.clear();
        int j=0;
        for(unsigned int i=0;i<segmentNr;i++)
        {
            if(this->chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
            {
                p_out = p_out * this->chain_.getSegment(i).pose(q_in(j));
                j++;
            }
            else
            {
                p_out = p_out * this->chain_.getSegment(i).pose(0.0);
            }

            this->jntPos_.push_back(KDL::Frame(p_out)); // store copies not references
        }
        return 0;
    }
}

KDL::Frame AdvancedChainFkSolverPos_recursive::getPostureAtJnt(uint16_t jntIndex) const
{
    KDL::Frame p_out = KDL::Frame::Identity();

    if (jntIndex < this->chain_.getNrOfJoints())
    {
        p_out = this->jntPos_.at(jntIndex);
    }

    return p_out;
}

void AdvancedChainFkSolverPos_recursive::dumpAllJntPostures() const
{
    uint16_t id = 0;
    ROS_INFO_STREAM("=== Dump all Jnt Postures ===");
    for(tFrameVector::const_iterator it = this->jntPos_.begin(); it != this->jntPos_.end(); ++it)
    {

        ROS_INFO_STREAM("Jnt " << id++ << ". Position: " << std::endl <<
                        it->p.x() << std::endl <<
                        it->p.y() << std::endl <<
                        it->p.z());
        ROS_INFO_STREAM("Rotation: " << std::endl <<
                        it->M.GetRot().x() << std::endl <<
                        it->M.GetRot().y() << std::endl <<
                        it->M.GetRot().z() << std::endl <<
                        "=================================");
    }
}

AdvancedChainFkSolverPos_recursive::~AdvancedChainFkSolverPos_recursive()
{
}
