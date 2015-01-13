#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_test_action_behaviour')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
from smach_ros import ActionServerWrapper
from cob_test_action.msg import TriggerAction, TriggerGoal


# protected region customHeaders on begin #
# protected region customHeaders end #



class cob_action_behaviour_impl:
	
	def	__init__(self):
		self.TriggerTracker_goal = TriggerGoal()
		genpy.message.fill_message_args(self.TriggerTracker_goal, [rospy.get_param('TriggerTracker')])
	
		# protected region initCode on begin #
		# protected region initCode end #
		pass
	
	def	configure(self):
		sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['action_feedback'], output_keys = ['action_feedback'])
		sis = smach_ros.IntrospectionServer('cob_action_behaviour', sm0, '/cob_action_behaviour_sm')
		sis.start()
		with sm0:
			smach.StateMachine.add('TriggerTracker', smach_ros.SimpleActionState('TriggerTracker', TriggerAction, self.TriggerTracker_goal), {
				"succeeded":"succeeded",
			})
	

	
		sm0.set_initial_state(['TriggerTracker'])

		# Execute

		#sm0.set_initial_state()
		outcome = sm0.execute()
	
		# protected region configureCode on begin #
		# protected region configureCode end #
		pass
	
	def	update(self):
		# protected region updateCode on begin #
		# protected region updateCode end #
		pass
		
	

class cob_action_behaviour:
	def __init__(self):
		self.impl = cob_action_behaviour_impl()

	
		
	def run(self):
		self.impl.update()

if __name__ == "__main__":
	try:
		rospy.init_node('cob_action_behaviour')
		r = rospy.Rate(10)
		n = cob_action_behaviour()
		n.impl.configure()
		while not rospy.is_shutdown():
			n.run()
			r.sleep()
			
	except rospy.ROSInterruptException:
		print "Exit"



