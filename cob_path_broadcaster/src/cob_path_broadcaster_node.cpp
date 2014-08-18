#include <ros/ros.h>
#include <cob_path_broadcaster/cob_path_broadcaster.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_path_node");
	CobPathBroadcaster *pb = new CobPathBroadcaster();
	
	pb->initialize();
	pb->run();
	
	return 0;
}
