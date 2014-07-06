#include <cob_hardware_interface/cob_hwinterface_vel.h>

CobHWInterfaceVel::CobHWInterfaceVel()
{ 
   //get joint_names
   joint_names.push_back("arm_1_joint");
   joint_names.push_back("arm_2_joint");
   joint_names.push_back("arm_3_joint");
   joint_names.push_back("arm_4_joint");
   joint_names.push_back("arm_5_joint");
   joint_names.push_back("arm_6_joint");
   joint_names.push_back("arm_7_joint");
   
   pos.resize(joint_names.size());
   vel.resize(joint_names.size());
   eff.resize(joint_names.size());
   cmd.resize(joint_names.size());
   
   
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
   
   
   //also initialize the accoring subscriber and publisher here 
}


void CobHWInterfaceVel::read()
{
   //here pos, vel, eff need to be updated!
   //this should be done by safely (access-control!) writing the current values that were received through a callback function for the subscription to /joint_states or the respective hw-driver directly 
   
   return;  
}


void CobHWInterfaceVel::write()
{
   //here the current cmd values need to be propagated to the actual hardware
   //cmd contains the desired velocity in this case
   //this should be done by publishing them to the accortind command_vel (or command_direct) topic of the hw-driver
   
   return;
}
