#include <ros/ros.h>
#include <cob_model_identifier/output_recorder.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "output_recorder");
	OutputRecorder *outrec = new OutputRecorder();
	
	outrec->initialize();
	outrec->run();
	
	return 0;
}
