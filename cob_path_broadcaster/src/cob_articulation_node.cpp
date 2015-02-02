#include <ros/ros.h>
#include <cob_path_broadcaster/cob_articulation.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_articulation");
	CobArticulation *pb = new CobArticulation();
	
	bool success = pb->initialize();
	if(success){
		ROS_INFO("...initialized!");
		pb->load();
	}else{
		ROS_ERROR("Initialization failed");
	}
	return 0;
}
