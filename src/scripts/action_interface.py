#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi, floor, atan2, sqrt, sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback, CompletePlan

from rosplan_action_interface.ServiceActionInterface import ServiceActionInterface
from rosplan_action_interface.ActionInterfaceManager import ActionInterfaceManager
from rosplan_action_interface.ActionlibActionInterface import ActionlibActionInterface


class ActionManager:
    #  inicjalizacja action managera, ktory odpala kolejne action interface
    #  subskrybuje tematy action dispatch
    #  zapewnia komunikacje z baza wiedzy
    #  aktualizuje baze wiedzy
    #  zwraca action feedback
    act_manager = ActionInterfaceManager()
    

class ScanServer:
    def __init__(self):

        self.dispatch_subscriber = rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', ActionDispatch, self.dispatch_callback)
        self.dispatch_subscriber = rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, self.feedback_callback)
        self.dispatch_subscriber = rospy.Subscriber('/rosplan_parsing_interface/complete_plan', CompletePlan, self.plan_callback)

        # self.ranges_publisher = rospy.Publisher('/vacuum_sensors', ranges, queue_size=10)
        self.move_tiago()

    def subscriber_callback(self, data):

        action_id = data.action_id
        name = data.name
        plan_id = data.plan_id

        if name == 'go':
            self.go()
        
        elif name == 'undock':
            self.undock()
        
        elif name == 'dock':
            self.dock()
    
    def go(self):
        pass

    def undock(self):
        pass

    def dock(self):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('scan_server', anonymous=True)
        scan_server = ScanServer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass