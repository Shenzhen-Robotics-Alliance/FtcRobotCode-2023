# delay in reacting to driver operation since program changed to single core

## program description
the program reacted much slowlier ever since the program switched from multi-thread processing to single thread processing
and cannot respond to the two pilots' operation at the same time

## posible cause
my guess is that the arm controlling module is stucking the run loop because it's periodic function waits until the lift of thr arm to complete

## solution
	- first try: create a varible that repersents the status of the robot
	jump out of "periodic" instantly after setting the motors to run to position
	still do the slow-down process, but do it in function "periodic" during latter calls from main loop
	(TODO)