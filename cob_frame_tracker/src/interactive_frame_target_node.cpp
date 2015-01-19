#include <ros/ros.h>
#include <cob_frame_tracker/interactive_frame_target.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "interactive_frame_target");
	InteractiveFrameTarget *ift = new InteractiveFrameTarget();
	ift->initialize();
	
	ros::spin();
	delete ift;
	return 0;
}
