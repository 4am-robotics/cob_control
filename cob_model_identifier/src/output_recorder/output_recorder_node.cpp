#include <ros/ros.h>
#include <output_recorder/output_recorder.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "output_recorder");
	OutputRecorder *outrec = new OutputRecorder();
	
	outrec->initialize();
	outrec->run();
	
	return 0;
}
