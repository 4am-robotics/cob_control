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
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cob_twist_controller/utils/moving_average.h>


class MovingAverageTester
{
public:
    MovingAverageTester(std::string out, MovingAvgBase_double_t* ma)
    {
        ma_.reset(ma);
        output_pub_ = nh_.advertise<std_msgs::Float64>(out, 1);
        input_sub_ = nh_.subscribe("input", 1, &MovingAverageTester::input_cb, this);
    }

    ~MovingAverageTester()
    {}

    void input_cb(const std_msgs::Float64::ConstPtr& input)
    {
        std_msgs::Float64 output;

        ma_->addElement(input->data);
        if (ma_->calcMovingAverage(output.data))
        {
            output_pub_.publish(output);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;

    boost::shared_ptr< MovingAvgBase_double_t > ma_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moving_average_node");

    MovingAvgSimple_double_t* ma_3 = new MovingAvgSimple_double_t(3);
    MovingAverageTester mat("output_ma", ma_3);
    MovingAvgWeighted_double_t* maw_3 = new MovingAvgWeighted_double_t(3);
    MovingAverageTester maw_3t("output_maw", maw_3);
    MovingAvgExponential_double_t* mae_03 = new MovingAvgExponential_double_t(0.3);
    MovingAverageTester mae_03t("output_mae", mae_03);
    MovingAvgExponential_double_t* mae_05 = new MovingAvgExponential_double_t(0.5);
    MovingAverageTester mae_05t("output_mae05", mae_05);

    ros::spin();
    return 0;
}
