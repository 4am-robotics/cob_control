#include <ros/ros.h>
#include <cob_model_identifier/input_generator.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "input_generator");
	InputGenerator *ig = new InputGenerator();
	
	ig->initialize();
	ig->run();
	
	return 0;
}
