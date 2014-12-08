#!/usr/bin/env python
import roslib;
roslib.load_manifest('cob_lookat_controller')
import rospy
from std_msgs.msg import Float64MultiArray

def start_node():
    rospy.init_node('test_cmd_vel_publisher', anonymous=True)
    #advertise topic
    pub = rospy.Publisher('/driver/command_vel', Float64MultiArray, queue_size=1)
    rospy.sleep(1.0)
    
    msg = Float64MultiArray()
    #'lookat_lin_joint'
    msg.data.append(0.1)
    #'lookat_x_joint'
    msg.data.append(0.0)
    #'lookat_y_joint'
    msg.data.append(0.0)
    #'lookat_z_joint'
    msg.data.append(0.0)

    #print msg
    rospy.loginfo("publishing now!")
    pub.publish(msg)
    rospy.sleep(1.0)


if __name__ == '__main__':
    start_node()

