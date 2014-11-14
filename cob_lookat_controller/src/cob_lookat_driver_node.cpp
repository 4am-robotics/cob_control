#include <ros/ros.h>
#include <cob_lookat_controller/cob_lookat_driver.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_lookat_driver_node");
	CobLookatDriver *cld = new CobLookatDriver();
	
	if(cld->initialize())
	{
		cld->run();
	}
	else
		ROS_ERROR("Failed to initialize LookatDriver");
	
	return 0;
}
