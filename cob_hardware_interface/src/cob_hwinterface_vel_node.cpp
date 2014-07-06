#include <cob_hardware_interface/cob_hwinterface_vel.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cob_hwinterface_vel_node");

    CobHWInterfaceVel robot;
    controller_manager::ControllerManager cm(&robot);

    ros::Time time = ros::Time::now();
    ros::Time last_update_time = time;
    ros::Duration period = time - last_update_time;
  
    double update_rate = 68; //[hz]
    ros::Rate r(update_rate);
  
    while (ros::ok())
    {
        time = ros::Time::now();
        period = time - last_update_time;
        
        robot.read();
        cm.update(time, period);
        robot.write();
        
        last_update_time = time;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

