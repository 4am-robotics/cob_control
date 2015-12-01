/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Felix Messmer, email: felix.messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2015
 *
 * \brief
 *   This header contains the interface description for extending the
 *   kinematic chain with additional degrees of freedom, e.g. base_active or lookat
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_LOOKAT_H
#define COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_LOOKAT_H

#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

#include "cob_twist_controller/kinematic_extensions/kinematic_extension_base.h"
#include "cob_twist_controller/utils/simpson_integrator.h"

/* BEGIN KinematicExtensionLookat ****************************************************************************************/
/// Class to be used for Cartesian KinematicExtensions for Lookat.
class KinematicExtensionLookat : public KinematicExtensionBase
{
    public:
        explicit KinematicExtensionLookat(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {
            if (!initExtension())
            {
                ROS_ERROR("Initialization failed");
            }
        }

        ~KinematicExtensionLookat() {}

        bool initExtension();
        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        virtual JointStates adjustJointStates(const JointStates& joint_states);
        virtual LimiterParams adjustLimiterParams(const LimiterParams& limiter_params);
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik);

    private:
        unsigned int ext_dof_;
        KDL::Chain chain_ext_;
        KDL::Chain chain_full_;
        JointStates joint_states_ext_;
        JointStates joint_states_full_;
        std::vector<double> limits_ext_max_;
        std::vector<double> limits_ext_min_;
        std::vector<double> limits_ext_vel_;
        std::vector<double> limits_ext_acc_;

        boost::shared_ptr<KDL::ChainJntToJacSolver> jnt2jac_;
        boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_ext_;

        boost::shared_ptr<SimpsonIntegrator> integrator_;

        boost::mutex mutex_;
        ros::Timer timer_;
        tf::TransformBroadcaster br_;
        void broadcastFocusFrame(const ros::TimerEvent& event);
};
/* END KinematicExtensionLookat **********************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_LOOKAT_H
