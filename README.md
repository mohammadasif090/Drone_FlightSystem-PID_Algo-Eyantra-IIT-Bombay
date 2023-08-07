# Eyantra-IIT-Bombay 
# Please Have a look at Video Demonstration File of the project --> task1.2.mp4

Thanks for visiting here...

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll


It is a simulation of autonomous flight of the drone. It surveys the 3d sample coordinates using overhead camera input and WhyCon Markers present in 3d space. After identification, it flies using the autonomous PID algorithm flight system and traverses each identified coordinates.

Thanks!!
Asif
