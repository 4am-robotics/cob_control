#include <ros/ros.h>
#include <cob_model_identifier/output_recorder.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "output_recorder");
	OutputRecorder *output_recorder = new OutputRecorder();
	
	if(output_recorder->initialize())
	{
		output_recorder->run();
	}
	else
		ROS_ERROR("Failed to initialize OutputRecorder");
	
	return 0;
}
