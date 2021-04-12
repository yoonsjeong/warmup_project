# warmup_project


## Driving in a Square
For this program, the problem was to have the robot drive in a square. My approach was the simpler approach, moving straight for a fixed amount of time, turning, and repeating. My code runs a while loop that performs three different actions: it drives forward, it turns 90 degrees, and it stops. Between each of these commands it performs the sleep operation, to ensure that the acceleration is 0 before the next acceleration.
In my run function, I have three different movement options, called `straight`, `stop`, and `turn_90`. I execute the `straight` for 5 seconds, then stop for 2 seconds, then `turn_90` by rotating by pi/6 rads per second for 3 seconds, stop for 2 seconds, and repeat.
The following is a gif of the robot:
![driving in a square][square]
[square]: ./gifs/drive_square.gif

## Person Follower
For this program, the problem was to approach a figure that is within the robot's vision. I decided to have the robot rotate while standing still if it did not see a person in front of it, and if it does see a person, it will drive forward. I also added some code to have it back up a little bit if it gets too close to the person.
My run function just runs allows `/scan` to call its callback `process_scan` repeatedly. Within my `process_scan` function I first check to see if the distance from the nearest thing ahead is infinity (the way to do this in Python is `float("inf)`), in which case I spin around at a rate of 1 rad per second. Then it checks to see if the distance is greater than 1.2, in which case it will move forward. If it gets too close somehow, it will start moving backwards. If the distance is between 0.8 and 1.2, the robot will stop.
The following is a gif of the robot:
![person follower][person]
[person]: ./gifs/person_follower.gif

## Wall Follower
For this program, the problem was to approach a wall and travel alongside it. My approach for this was to use trigonometry. I use the LIDAR's angles to figure out when I was aligned parallel to the wall -- if the 45 degree distance is `sqrt(2)` times the 90 degree distance, that means it is a right isoceles triangle, making the robot parallel to the wall.
In my code, I first check to see if the robot is too close to the wall -- if the robot is too close in the front or the 45 degree distance is too close, the robot will rotate to change this distance. Once these distances are made further, the robot will either be in a position that satisfies the above trigonometric identity (`diag_ok` and `left_ok` are satisfied), or it will move forward until it hits a wall where the condition can be attempted.
The following is a gif of the robot:
![wall follower][wall]
[wall]: ./gifs/wall_follower.gif
<!-- 
ffmpeg -i person_follower.mp4 -vf "eq=brightness=0.1,fps=30,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 person_follower.gif
 -->