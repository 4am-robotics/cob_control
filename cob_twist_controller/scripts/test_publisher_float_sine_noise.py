#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
import math
import random
from std_msgs.msg import Float64

def inputPub():
  rospy.init_node("test_publisher_float_sine_noise", anonymous=True)
  #for testing behavior of moving_average and simpson_integrator

  input_pub = rospy.Publisher("input", Float64, queue_size=1)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)

  a = 0.05#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = 0.0#-math.pi/2.0
  d = 0.0#a
  i = 0.0

  input_msg = Float64()

  while not rospy.is_shutdown():
    input_msg.data = a*math.sin(b*i+c) + d
    
    noise = random.uniform(-a, a)
    #noise = random.uniform(-0.01, 0.01)
    input_msg.data += noise
    
    input_pub.publish(input_msg)
    
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      inputPub()
  except rospy.ROSInterruptException: pass
