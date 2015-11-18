#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cob_twist_controller/utils/moving_average.h>


class MovingAverageTester
{
public:
    MovingAverageTester(std::string out, uint16_t ma_size, bool ma_auto_weighting)
    {
        ma_.reset(new MovingAvg_double_t(ma_size, ma_auto_weighting));
        output_pub_ = nh_.advertise<std_msgs::Float64>(out, 1);
        input_sub_ = nh_.subscribe("input", 1, &MovingAverageTester::input_cb, this);
    }
    
    ~MovingAverageTester()
    {}
    
    void input_cb(const std_msgs::Float64::ConstPtr& input)
    {
        std_msgs::Float64 output;
        
        ma_->addElement(input->data);
        ma_->calcWeightedMovingAverage(output.data);
        
        output_pub_.publish(output);
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    
    boost::shared_ptr<MovingAvg_double_t> ma_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moving_average_node");
    MovingAverageTester mat_1_true("output_ma_1_true", 1, true);
    MovingAverageTester mat_3_true("output_ma_3_true", 3, true);
    MovingAverageTester mat_10_true("output_ma_10_true", 10, true);
    MovingAverageTester mat_100_true("output_ma_100_true", 100, true);

    ros::spin();
    return 0;
}
