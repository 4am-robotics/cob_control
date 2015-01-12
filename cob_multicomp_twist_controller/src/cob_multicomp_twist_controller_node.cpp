#include <ros/ros.h>
#include <cob_multicomp_twist_controller/cob_multicomp_twist_controller.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_multicomp_twist_controller_node");
	CobMultiCompTwistController *cob_multicomp_twist_controller = new CobMultiCompTwistController();
	
	if(cob_multicomp_twist_controller->initialize())
	{
		cob_multicomp_twist_controller->run();
	}
	else
		ROS_ERROR("Failed to initialize MultiCompTwistController");
	
	return 0;
}
