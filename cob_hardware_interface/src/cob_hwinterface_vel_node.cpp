#include <cob_hardware_interface/cob_hwinterface_vel.h>
#include <controller_manager/controller_manager.h>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>



class CobControllerManager
{
    public:
        ~CobControllerManager()
        {
            start_thread.join();
        }
        
        void initialize()
        {
            nh = ros::NodeHandle("~");
            ROS_INFO("Namespace %s", nh.getNamespace().c_str());
            if (nh.hasParam("load_controller"))
            {
                nh.getParam("load_controller", load_controller);
                ROS_INFO("Loading %s", load_controller.c_str());
            }
            else
            {
                ROS_ERROR("No parameter 'load_controller' specified. Aborting.");
                return;
            }

            cm = new controller_manager::ControllerManager(&robot);
            //cmd_vel_sub = nh.subscribe("command_vel",1, &CobHWInterfaceVel::command_vel_callback,this);
            
            start_thread = boost::thread(boost::bind(&CobControllerManager::start_controller, this, load_controller));
            
            update_rate = 100;  //[hz]
            control_period = ros::Duration(0.01);    
        }
        
        void run()
        {
            ros::Rate r(update_rate);
            ros::Time time = ros::Time::now();
            ros::Time last_update_time = time;
            ros::Duration period = time - last_update_time;
            
            while (ros::ok())
            {
                time = ros::Time::now();
                period = time - last_update_time;
                
                if(period >= control_period)
                {
                    last_update_time = time;
                    
                    robot.read();
                    cm->update(time, period);
                }
                
                robot.write();
                
                ros::spinOnce();
                r.sleep();
            }
        }
        
    private:
        ros::NodeHandle nh;
        std::string load_controller;
        
        //ros::Subscriber cmd_vel_sub;
        
        CobHWInterfaceVel robot;
        controller_manager::ControllerManager* cm;
        
        double update_rate;
        ros::Duration control_period;
        
        boost::thread start_thread;
        
        void start_controller(std::string load_controller)
        {
            bool success = false;
            std::vector<std::string> start_controllers;
            std::vector<std::string> stop_controllers;
            
            start_controllers.push_back(load_controller);
            
            ROS_INFO("Load controller...");
            success = cm->loadController(load_controller);
            ROS_INFO("...done");
            
            if(success)
            {
                ROS_INFO("Switch controller...");
                success = cm->switchController(start_controllers, stop_controllers, 2);
                ROS_INFO("...done");
            }
            
            if(success)
                ROS_INFO("Controller should be running!");
            else
                ROS_ERROR("Failed to start controller!");
        }
     
};





int main(int argc, char** argv)
{
    ros::init(argc, argv, "cob_hwinterface_vel_node");
    
    CobControllerManager* ccm = new CobControllerManager();
    ccm->initialize();
    ccm->run();
    
    return 0;
}




