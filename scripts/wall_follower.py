#!/usr/bin/env python3

# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is
import rospy
# msg needed for /scan.
from sensor_msgs.msg import LaserScan
# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

PI = 3.14159265
distance = 1
class WallFollower(object):
    """ This node walks the robot to wall and stops """
    def __init__(self):
        # Start rospy node.
        rospy.init_node("walk_to_wall")
        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def within_range(self, val1, val2):
        return abs(val1 - val2) < 0.2

    def process_scan(self, data):

        diag_ok = self.within_range(1.4, data.ranges[45])
        left_ok = self.within_range(1, data.ranges[90])
        front_too_close = data.ranges[0] < 0.5
        diag_too_close = data.ranges[45] < 0.5
        
        if front_too_close or diag_too_close:
            self.twist.angular.z = -0.5
            self.twist.linear.x = 0
        elif diag_ok and left_ok:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.2
        else:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()