#include <cob_hardware_interface/cob_hwinterface_topics.h>
#include <controller_manager/controller_manager.h>

#include <boost/thread.hpp>


class CobControllerManager
{
  public:
    ~CobControllerManager()
    {
      //start_thread.join();
    }
    
    void initialize()
    {
      nh_p = ros::NodeHandle("~");
      //ROS_INFO("Namespace private %s", nh_p.getNamespace().c_str());
      //if (nh_p.hasParam("load_controller"))
      //{
        //nh_p.getParam("load_controller", load_controller);
        //ROS_INFO("Loading %s", load_controller.c_str());
      //}
      //else
      //{
        //ROS_ERROR("No parameter 'load_controller' specified. Aborting.");
        //return;
      //}

      cm = new controller_manager::ControllerManager(&robot);
      
      //start_thread = boost::thread(boost::bind(&CobControllerManager::start_controller, this, load_controller));
      
      update_rate = 100;  //[hz]
      timer = nh_p.createTimer(ros::Duration(1/update_rate), &CobControllerManager::update, this);
    }
    
    void update(const ros::TimerEvent& event)
    {
      ros::Duration period = event.current_real - event.last_real;
      
      robot.read();
      cm->update(event.current_real, period);
      
      robot.write();
    }
    
  private:
    ros::NodeHandle nh_p;
    
    CobHWInterfaceTopics robot;
    controller_manager::ControllerManager* cm;
    //std::string load_controller;
    
    ros::Timer timer;
    double update_rate;
    
    //boost::thread start_thread;
    
    //void start_controller(std::string load_controller)
    //{
      //bool success = false;
      //std::vector<std::string> start_controllers;
      //std::vector<std::string> stop_controllers;
      
      //start_controllers.push_back(load_controller);
      
      //success = cm->loadController(load_controller);
      
      //if(success)
      //{
        //success = cm->switchController(start_controllers, stop_controllers, 2);
      //}
      
      //if(success)
      //{
        //ROS_INFO("Controller should be running!");
      //}
      //else
        //ROS_ERROR("Failed to start controller!");
    //}
   
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_controller_manager_node");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  CobControllerManager* ccm = new CobControllerManager();
  ccm->initialize();

  ros::waitForShutdown();  
  return 0;
}




