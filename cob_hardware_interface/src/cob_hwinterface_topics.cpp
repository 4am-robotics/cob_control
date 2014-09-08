#include <cob_hardware_interface/cob_hwinterface_topics.h>

CobHWInterfaceTopics::CobHWInterfaceTopics()
{
  //Get joint names from parameter server  
  std::string param="joint_names";
  XmlRpc::XmlRpcValue jointNames_XMLRPC;
  if (nh.hasParam(param))
  {
    nh.getParam(param, jointNames_XMLRPC);
  }
  else
  {
    ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
    nh.shutdown();
  }

  for (int i=0; i<jointNames_XMLRPC.size(); i++)
  {
    joint_names.push_back(static_cast<std::string>(jointNames_XMLRPC[i]));  
    ROS_INFO("JointNames: %s", joint_names[i].c_str());
  }
   
  pos.resize(joint_names.size());
  vel.resize(joint_names.size());
  eff.resize(joint_names.size());
  cmd.resize(joint_names.size());
  
  //initialize the according subscriber and publisher here   
  cmd_vel_pub = nh.advertise<brics_actuator::JointVelocities>("command_vel_direct",1);
  jointstates_sub = nh.subscribe("/joint_states",1, &CobHWInterfaceTopics::jointstates_callback,this);
   
  for(unsigned int i=0; i<joint_names.size(); i++)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);
    
    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
    jnt_vel_interface.registerHandle(vel_handle);
  }

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);
}

void CobHWInterfaceTopics::jointstates_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //boost::lock_guard<boost::mutex> guard(mtx_);
  for(unsigned int i=0; i<msg->name.size(); i++) 
  {
    for(unsigned int j=0; j<joint_names.size(); j++)
    {
      if(msg->name[i] == joint_names[j])
      {
        pos[j]=msg->position[i];
        vel[j]=msg->velocity[i];
        eff[j]=msg->effort[i];
        break;
      }
    }
  }
}

void CobHWInterfaceTopics::read()
{
  //ROS_INFO("Reading Interface");
  //pos, vel, eff are updated!
  //boost::lock_guard<boost::mutex> guard(mtx_);
}

void CobHWInterfaceTopics::write()
{
  //ROS_INFO("Writing Interface");
  //for(unsigned int k=0; k<cmd.size(); k++)
  //{
    //ROS_INFO("CMD[%d]: %f", k, cmd[k]);
  //}
  //here the current cmd values need to be propagated to the actual hardware
  brics_actuator::JointVelocities command;
  ros::Time update_time = ros::Time::now();
  
  //mtx_.lock();
  for (unsigned int i=0; i<joint_names.size(); i++) 
  {    
    brics_actuator::JointValue value;
    value.timeStamp = update_time;
    value.joint_uri = joint_names[i];
    value.unit = "rad";
    value.value=cmd[i];
    command.velocities.push_back(value);
  }
  //mtx_.unlock();
     
  cmd_vel_pub.publish(command);
  
  return;
}
