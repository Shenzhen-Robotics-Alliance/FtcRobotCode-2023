# manual stage encoder auto correction

## problem description
the robot's chassis is always rotating randomly during manual stage movements, especially those with higher speed

## my solution
maybe we can add encoder correction to the manual stage program, their are two ways to do this:
	1. the joystick controls the targeted rotation of the robot, during every period, the program updates the targeted rotation according to the time elasped, the preseted sensitivity and the value of the stick. Then the robot stick to the targeted rotation using the algorizm from auto-stage program
	2. store the robot's rotation during the previous period using a variable, if the robot isn't asked to move, it sticks to the previous rotation using that same algorism and leave the variable unchanged; however, if the robot is asked to move, it rotates as before and updates that variable