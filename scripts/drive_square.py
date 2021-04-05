#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

PI = 3.14159265

class DriveSquare(object):
    def __init__(self):
        # initialize the ROS node
        rospy.init_node('spin_circles')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        straight = Twist(
            linear=Vector3(0.1, 0, 0),
        )
        stop = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
        )
        turn_90 = Twist(
            angular=Vector3(0, 0, PI/6)
        )

        rospy.sleep(1)
        while True:
            # go forward
            self.robot_movement_pub.publish(straight)
            rospy.sleep(5)
            # stop
            self.robot_movement_pub.publish(stop)
            rospy.sleep(2)
            # turn
            self.robot_movement_pub.publish(turn_90)
            rospy.sleep(3)
            # stop
            self.robot_movement_pub.publish(stop)
            rospy.sleep(2)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()