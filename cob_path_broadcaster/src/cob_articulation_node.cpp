#include <ros/ros.h>
#include <cob_path_broadcaster/cob_articulation.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_articulation");
	CobArticulation *pb = new CobArticulation();
	
	pb->initialize();
	pb->run();
	
	return 0;
}
