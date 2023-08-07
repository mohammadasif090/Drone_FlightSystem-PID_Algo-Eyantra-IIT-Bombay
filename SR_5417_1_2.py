#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries
import time
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control',anonymous= True)# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [5,5.3,16.3] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 00
		self.cmd.rcAUX2 = 00
		self.cmd.rcAUX3 = 00
		self.cmd.rcAUX4 = 00


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [23.58,23.4,105]
		self.Ki = [0.016,0.0064,-0.008]
		self.Kd = [103.5,108,123]


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.error = [0,0,0]
		self.prev_error = [0.0,0.0,0.0] 
		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]
		self.error_change = [0.0,0.0,0.0]
		self.error_sum = [0,0,0]
		self.path_points = [( 0.2, 0, 23), (3.5, -3.3, 23), ( -4.4, -4.6, 20.7) , ( -6.0, 6.3, 18.3), (5.0, 5.3, 16.3)]
		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]	
		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds
		self.sample_time = 0.060






		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.alt_error = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error = rospy.Publisher('/roll_error', Float64, queue_size=1)





		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * (-0.0008)
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = alt.Ki * 0.0008
		self.Kd[1] = alt.Kd * 0.3

	def roll_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = alt.Ki * 0.0008
		self.Kd[0] = alt.Kd * 0.3
















	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

	#------------------------------------------------------------------------------------------------------------------------
		# error_sum = [0,0,0]
		self.error=[self.setpoint[0]-self.drone_position[0],  self.setpoint[1]-self.drone_position[1],  self.drone_position[2]-self.setpoint[2]]
		self.error_change = [(self.error[0]-self.prev_error[0])/self.sample_time,(self.error[1]-self.prev_error[1])/self.sample_time, (self.error[2]-self.prev_error[2])/self.sample_time]

		self.error_sum = [self.error_sum[0]+ self.error[0]*self.sample_time, self.error_sum[1]+ self.error[1]*self.sample_time, self.error_sum[2]+ self.error[2]*self.sample_time] 


		self.out_throttle = (self.Kp[2] * self.error[2]) + (self.Kd[2]*self.error_change[2]) + (self.Ki[2]*self.error_sum[2])#here 0.6 is self.kp[2] THROTTLE  Z
		self.out_pitch    = (self.Kp[1] * self.error[1]) + (self.Kd[1]*self.error_change[1]) + (self.Ki[1]*self.error_sum[1])# PITCH Y
		self.out_roll     = (self.Kp[0] * self.error[0]) + (self.Kd[0]*self.error_change[0]) + (self.Ki[0]*self.error_sum[0])#ROLL X

		self.cmd.rcThrottle = 1500 + self.out_throttle
		self.cmd.rcPitch    = 1500 - self.out_pitch
		self.cmd.rcRoll     = 1500 + self.out_roll

		self.cmd.rcThrottle= self.max_values[2] if self.cmd.rcThrottle > self.max_values[2] else self.cmd.rcThrottle
		self.cmd.rcThrottle= self.min_values[2] if self.cmd.rcThrottle < self.min_values[2] else self.cmd.rcThrottle
		
		self.cmd.rcRoll= self.max_values[0] if self.cmd.rcRoll > self.max_values[0] else self.cmd.rcRoll
		self.cmd.rcRoll= self.min_values[0] if self.cmd.rcRoll < self.min_values[0] else self.cmd.rcRoll
		
		self.cmd.rcPitch = self.max_values[1] if self.cmd.rcPitch > self.max_values[1] else self.cmd.rcPitch
		self.cmd.rcPitch = self.min_values[1] if self.cmd.rcPitch < self.min_values[1] else self.cmd.rcPitch


		self.prev_error=self.error


		# if self.cmd.rcThrottle>self.max_values[2]:
		# 	self.cmd.rcThrottle = self.max_values[2]
		# if self.cmd.rcThrottle< self.min_values[2]:
		# 	self.cmd.rcThrottle = self.min_values[2] 
		# print("-----------------------------------------------------")
		# print("current_error: ",self.error)
		# print("prev_error: ",self.prev_error)
		# print(self.error)
		self.alt_error.publish(self.error[2])
		self.pitch_error.publish(self.error[1])
		self.roll_error.publish(self.error[0])
		self.command_pub.publish(self.cmd)
		
		

		
		# print(self.error)
		# print(self.prev_error)
		# print(self.error_change)
		# print(self.out_roll,self.out_pitch,self.out_throttle)






if __name__ == '__main__':
	i=0
	e_drone = Edrone()
	e_drone.setpoint = e_drone.path_points[i]
	r = rospy.Rate(60) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():

		if  (
			((e_drone.path_points[i][0]-0.2) <= e_drone.drone_position[0]<= (e_drone.path_points[i][0]+0.2)) and
			((e_drone.path_points[i][1]-0.2) <= e_drone.drone_position[1]<= (e_drone.path_points[i][1]+0.2)) and 
			((e_drone.path_points[i][2]-0.4) <= e_drone.drone_position[2]<= (e_drone.path_points[i][2]+0.3))
			):
			# time.sleep(2)
			if i==4:
				print("Path Completed")
			else:
				print("Reached Level " + str(i+1))

			# print(e_drone.drone_position)
			
			if i < (len(e_drone.path_points)-1):
				i=i+1
			e_drone.setpoint= e_drone.path_points[i]

		e_drone.pid()
		r.sleep()
