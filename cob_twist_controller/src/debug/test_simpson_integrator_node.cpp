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


#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <kdl/jntarray.hpp>
#include <cob_twist_controller/utils/simpson_integrator.h>


class SimpsonIntegratorTester
{
public:
    SimpsonIntegratorTester()
    {
        unsigned int dof = 1;
        q_.resize(dof);
        KDL::SetToZero(q_);
        q_dot_.resize(dof);
        KDL::SetToZero(q_dot_);

        integrator_.reset(new SimpsonIntegrator(dof));
        output_q_pub_ = nh_.advertise<std_msgs::Float64>("output_q", 1);
        output_q_dot_pub_ = nh_.advertise<std_msgs::Float64>("output_q_dot", 1);
        input_sub_ = nh_.subscribe("input", 1, &SimpsonIntegratorTester::input_cb, this);
    }

    ~SimpsonIntegratorTester()
    {}

    void input_cb(const std_msgs::Float64::ConstPtr& input)
    {
        q_dot_(0) = input->data;

        std::vector<double> next_q;
        std::vector<double> next_q_dot;

        if (integrator_->updateIntegration(q_dot_, q_, next_q, next_q_dot))
        {
            for (unsigned int i = 0; i < next_q.size(); i++)
            {
                q_(i) = next_q[i];
                q_dot_(i) = next_q_dot[i];
            }

            std_msgs::Float64 output_q;
            output_q.data = q_(0);
            std_msgs::Float64 output_q_dot;
            output_q_dot.data = q_dot_(0);

            output_q_pub_.publish(output_q);
            output_q_dot_pub_.publish(output_q_dot);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_q_pub_;
    ros::Publisher output_q_dot_pub_;

    KDL::JntArray q_;
    KDL::JntArray q_dot_;

    boost::shared_ptr<SimpsonIntegrator> integrator_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_simpson_integrator_node");

    SimpsonIntegratorTester sit;

    ros::spin();
    return 0;
}
