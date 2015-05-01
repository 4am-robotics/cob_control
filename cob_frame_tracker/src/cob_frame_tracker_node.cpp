#include <ros/ros.h>
#include <cob_frame_tracker/cob_frame_tracker.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_frame_tracker_node");
	CobFrameTracker *cft = new CobFrameTracker();
	
	if(!cft->initialize())
	{
		ROS_ERROR("Failed to initialize FrameTracker");
		return -1;
	}
	
	ros::spin();
	delete cft;
	return 0;
}
