# arm not working


## Problem discription
at first, one of the two motors that runs the arm is experiencing an encoder failure, so I would have use one of the encoder to guide both

## solution
1. maybe I can write a PID control algorism myself and power both motors
2. I can put two motors together as one and use one encoder to run, by laying them parrelly in one motor port and make the program control that motor port, just reverse the positive and negative side of the left motor as it spins reversively with the right motor when the arm is moving in one direction


## failure
the controller hub broke down completedly, I tested it by tring to use controller hub to power motors orginally connected to expansion hub, or the reversed way, it turns out that neighter a controller hub's hardware works on the expanssion hub, nor does the other way round, so WTF is going on?
oh it seems that the expanssion hub does give a votage on the motor, but the motor didn't turn, LOL

Ok we know what happenend, the fucking line caused all of it, the fucking line was in bad condition
I measured the votage around the motor port and it turns out to be that the port is functioning well
So now I would have to write a PID or put the motors together

