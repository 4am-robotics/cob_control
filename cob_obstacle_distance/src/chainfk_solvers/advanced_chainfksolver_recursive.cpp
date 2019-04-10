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


#include <ros/ros.h>
#include "cob_obstacle_distance/chainfk_solvers/advanced_chainfksolver_recursive.hpp"

AdvancedChainFkSolverPos_recursive::AdvancedChainFkSolverPos_recursive(const KDL::Chain& _chain):
    chain_(_chain)
{
}

/**
 * Calculates the cartesion positions given to the joints array.
 * This special implementation ensures that the positions are stored in a vector so it is not necessary to call the method for each segment
 * again and again.
 */
int AdvancedChainFkSolverPos_recursive::JntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out, int seg_nr)
{
    unsigned int segmentNr;
    if (seg_nr < 0)
    {
        segmentNr = this->chain_.getNrOfSegments();
    }
    else
    {
        segmentNr = seg_nr;
    }

    p_out = KDL::Frame::Identity();

    if (q_in.rows() != this->chain_.getNrOfJoints())
    {
        return -1;
    }
    else if (segmentNr > this->chain_.getNrOfSegments())
    {
        return -1;
    }
    else
    {
        this->segment_pos_.clear();
        int j = 0;
        for (unsigned int i = 0; i < segmentNr; i++)
        {
            if (this->chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
            {
                p_out = p_out * this->chain_.getSegment(i).pose(q_in(j));
                j++;
            }
            else
            {
                p_out = p_out * this->chain_.getSegment(i).pose(0.0);
            }

            this->segment_pos_.push_back(KDL::Frame(p_out));  // store copies not references
        }
        return 0;
    }
}

/**
 * Access previously set segment positions via index.
 */
KDL::Frame AdvancedChainFkSolverPos_recursive::getFrameAtSegment(uint16_t seg_idx) const
{
    KDL::Frame p_out = KDL::Frame::Identity();

    if (seg_idx < this->chain_.getNrOfSegments())
    {
        p_out = this->segment_pos_.at(seg_idx);
    }

    return p_out;
}

/**
 * Output of all previously set segment positions.
 */
void AdvancedChainFkSolverPos_recursive::dumpAllSegmentPostures() const
{
    uint16_t id = 0;
    ROS_INFO_STREAM("=== Dump all Jnt Postures ===");
    for (FrameVector_t::const_iterator it = this->segment_pos_.begin(); it != this->segment_pos_.end(); ++it)
    {
        ROS_INFO_STREAM("Segment " << id++ << ". Position: " << std::endl <<
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


AdvancedChainFkSolverVel_recursive::AdvancedChainFkSolverVel_recursive(const KDL::Chain& _chain):
    chain_(_chain)
{
}

/**
 * Calculates the cartesion velocities given to the joints and joint velocities array.
 * This special implementation ensures that the velocities are stored in a vector so it is not necessary to call the method for each segment
 * again and again.
 */
int AdvancedChainFkSolverVel_recursive::JntToCart(const KDL::JntArrayVel& q_in, KDL::FrameVel& out, int seg_nr)
{
    unsigned int segmentNr;
    if (seg_nr < 0)
    {
        segmentNr = this->chain_.getNrOfSegments();
    }
    else
    {
        segmentNr = seg_nr;
    }

    out = KDL::FrameVel::Identity();

    if (!(q_in.q.rows() == this->chain_.getNrOfJoints() && q_in.qdot.rows() == this->chain_.getNrOfJoints()))
    {
        ROS_ERROR("Rows do not match!");
        return -1;
    }
    else if (segmentNr > this->chain_.getNrOfSegments())
    {
        return -2;
    }
    else
    {
        this->segment_vel_.clear();
        int j = 0;
        for (unsigned int i = 0; i < segmentNr; ++i)
        {
            // Calculate new Frame_base_ee
            if (this->chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
            {
                out = out * KDL::FrameVel(this->chain_.getSegment(i).pose(q_in.q(j)),
                                          this->chain_.getSegment(i).twist(q_in.q(j), q_in.qdot(j)));
                j++;  // Only increase jointnr if the segment has a joint
            }
            else
            {
                out = out * KDL::FrameVel(this->chain_.getSegment(i).pose(0.0),
                                 this->chain_.getSegment(i).twist(0.0, 0.0));
            }

            this->segment_vel_.push_back(KDL::FrameVel(out));
        }

        return 0;
    }
}

int AdvancedChainFkSolverVel_recursive::JntToCart(const KDL::JntArrayVel& in,
                                                  std::vector<KDL::FrameVel>& out,
                                                  int seg_nr)
{
    unsigned int segmentNr;
    if (seg_nr < 0)
        segmentNr = chain_.getNrOfSegments();
    else
        segmentNr = seg_nr;

    if (!(in.q.rows() == chain_.getNrOfJoints() && in.qdot.rows() == chain_.getNrOfJoints()))
        return -1;
    else if (segmentNr > chain_.getNrOfSegments())
        return -1;
    else if (out.size() != segmentNr)
        return -1;
    else if (segmentNr == 0)
        return -1;
    else
    {
        int j = 0;
        // Initialization
        if (chain_.getSegment(0).getJoint().getType() != KDL::Joint::None)
        {
            out[0] = KDL::FrameVel(chain_.getSegment(0).pose(in.q(0)), chain_.getSegment(0).twist(in.q(0), in.qdot(0)));
            j++;
        }
        else
            out[0] = KDL::FrameVel(chain_.getSegment(0).pose(0.0), chain_.getSegment(0).twist(0.0, 0.0));

        for (unsigned int i = 1; i < segmentNr; i++)
        {
            // Calculate new Frame_base_ee
            if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
            {
                out[i] =
                  out[i - 1] * KDL::FrameVel(chain_.getSegment(i).pose(in.q(j)), chain_.getSegment(i).twist(in.q(j), in.qdot(j)));
                j++;    // Only increase jointnr if the segment has a joint
            }
            else
            {
                out[i] = out[i - 1] * KDL::FrameVel(chain_.getSegment(i).pose(0.0), chain_.getSegment(i).twist(0.0, 0.0));
            }
        }
        return 0;
    }
}

KDL::FrameVel AdvancedChainFkSolverVel_recursive::getFrameVelAtSegment(uint16_t seg_idx) const
{
    KDL::FrameVel vel_out = KDL::FrameVel::Identity();

    if (seg_idx < this->chain_.getNrOfSegments())
    {
        vel_out = this->segment_vel_.at(seg_idx);
    }

    return vel_out;
}


AdvancedChainFkSolverVel_recursive::~AdvancedChainFkSolverVel_recursive()
{
}
