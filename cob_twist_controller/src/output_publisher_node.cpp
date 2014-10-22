#include <ros/ros.h>
#include <cob_twist_controller/output_publisher.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "output_publisher");
	OutputPublisher *outrec = new OutputPublisher();
	
	outrec->initialize();
	outrec->run();
	
	return 0;
}
