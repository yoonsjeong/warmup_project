# warmup_project


## Driving in a Square
For this program, the problem was to have the robot drive in a square. My approach was the simpler approach, moving straight for a fixed amount of time, turning, and repeating.
My code runs a while loop that performs three different actions: it drives forward, it turns 90 degrees, and it stops. Between each of these commands it performs the sleep operation, to ensure that the acceleration is 0 before the next acceleration.
The following is a gif of the robot:

![driving in a square][square]

[square]: ./square.gif