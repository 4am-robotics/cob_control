#include <ros/ros.h>
#include <cob_twist_controller/cob_twist_controller.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_twist_controller_node");
	CobTwistController *cob_twist_controller = new CobTwistController();
	
	if(cob_twist_controller->initialize())
	{
		cob_twist_controller->run();
	}
	else
		ROS_ERROR("Failed to initialize TwistController");
	
	return 0;
}
