from core_nodes import Arm_System
from core_nodes.motor import Spirob_Motors
from core_nodes.vision import Vision_System
from core_nodes.load_cell import Load_Cell
import time
import numpy as np
import cv2
# Initiling the System

PUL = [17, 25, 6] 
DIR = [27, 24, 5] 
MOTOR_ORIENTATION = [True, True, False]

motors = Spirob_Motors(
	PUL, 
	DIR, 
	MOTOR_ORIENTATION, 
	steps_per_rev=1600, 
	max_rpm=30, 
	pulse_width=10 * 1e-6,
	debug=False, 
	speed = 1)

vision = Vision_System()

#load = Load_Cell()

system = Arm_System("Hysteresis_Experiment_test",motors,vision)

# Define Experiment Specific Tasks 

def lift_low(direction,mag):
	print("I did this")
	pos = [0,0,0]
	pos[direction] = -mag
	
	system.motors.move_to_steps(*pos)
	system.motors.move_to_steps(0,0,0)


if __name__ == "__main__":
	try:
		system.start()
		print("Experiment Starts")
		
		motors.speed = 1


		lift_low(2,3000)
		
		#lift_low(1,3000)

		#lift_low(2,3000)
		
		
		experiment_data = system.callback() 
	finally:
		system.stop()
		system.plot_post_processing()
		
		
		

	
	

