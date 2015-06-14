#include <ros/ros.h>
#include <cob_cartesian_controller/cartesian_controller.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cartesian_controller_node");
	CartesianController *cc = new CartesianController();
	
	bool success = cc->initialize();
	if(success)
	{
		ROS_INFO("...initialized!");
		cc->load();
	}else
	{
		ROS_ERROR("Initialization failed");
	}
	return 0;
}
