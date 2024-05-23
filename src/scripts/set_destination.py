#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from math import pi, floor, atan2, sqrt, sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import actionlib
from math import sin, cos, atan2, pi
from dyplom.msg import goalAction
from dyplom.msg import goalResult
from dyplom.msg import goalFeedback
from dyplom.msg import goalGoal


class GoalServer:
    def __init__(self):

        # self.scan_subscriber = rospy.Subscriber('/vacuum_sensors', ranges, self.subscriber_callback)
        # self.n = rospy.get_param("/sectors")
        # self.destination = rospy.get_param("/destination")
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        # self.goal_action_publisher = actionlib.SimpleActionClient('destination', goalAction)
        self.publish()

    def publish(self, x=1, y=1):

        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # self.goal_publisher.publish(goal)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.goal_publisher.publish(goal)
            rate.sleep()

        # goal.header.frame_id = "map"
        # goal.header.stamp = rospy.Time.now()

        # goal = goalGoal()
        # goal.x = float(x)
        # goal.y = float(y)
        # goal.z = 0
        # goal.ox = 0
        # goal.oy = 0
        # goal.oz = 0
        # goal.ow = 1
        # print(goal.x, goal.y)
        # print("Waiting for action server...")
        # self.goal_action_publisher.wait_for_server()
        # self.goal_action_publisher.send_goal(goal)
        # wait = self.goal_action_publisher.wait_for_result()

        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return self.goal_action_publisher.get_result()


  
if __name__ == '__main__':
    try:
        rospy.init_node('goal_server', anonymous=True)
        rate = rospy.Rate(10)
        goal_server = GoalServer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass