#!/usr/bin/env python

import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from std_srvs.srv import Empty
from BaseActionInterface import BaseActionInterface
from ActionlibActionInterface import ActionlibActionInterface
from ServiceActionInterface import ServiceActionInterface
from FSMActionInterface import FSMActionInterface
from RPKnowledgeBaseLink import RPKnowledgeBaseLink
from tiago_rosplan_sim.srv import RescueService

# This class defines the action interface manager node. The node:
# - initialises a set of action interfaces according to config file;
# - subscribes to the PDDL action dispatch topic;
# - runs action execution through interfaces;
# - updates knowledge base with action effects;
# - publishes to PDDL action feedback.
class ActionInterfaceManager(object):

    # interfaces to manage
    _action_interfaces = {}

    def __init__(self):
        # knowledge base link
        self._kb_link = RPKnowledgeBaseLink()
        self.debug = False

        # load action interfaces from configuration file
        found_config = False
        if rospy.has_param('/action_interface_manager/actions'):
            self.cfg_actions = rospy.get_param('/action_interface_manager/actions')
            found_config = True
            self.action_params = self.cfg_actions['actions']
            if self.debug == True:
                print("Action params")
                print(self.action_params)

        if not found_config:
            rospy.logerr('KCL: ({}) Error: configuration file was not laoded.'.format(rospy.get_name()))
            rospy.signal_shutdown('Config not found')
            return

        # feedback
        # nie ma czegos takiego jak action_feedback_topic
        # wiec wysyla na default_topic
        # do zmiany !!!

        aft = rospy.get_param('~action_feedback_topic', 'default_feedback_topic')
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self._action_feedback_pub = rospy.Publisher(aft, ActionFeedback, queue_size=10)

        # subscribe to action dispatch
        # adt = rospy.get_param('~action_dispatch_topic', 'default_dispatch_topic')
        # change of action dispatch topic to one advertised bu ROSPlan dispatcher
        adt = "/rosplan_plan_dispatcher/action_dispatch"
        rospy.Subscriber(adt, ActionDispatch, self.dispatch_callback, queue_size=10)

        # cdt = "/rosplan_plan_dispatcher/action_dispatch"
        # rospy.Subscriber(cdt, ActionDispatch, self.dispatch_callback, queue_size=10)

        self.parse_config()

        rospy.loginfo('KCL: ({}) Ready to receive'.format(rospy.get_name()))
        self.run()

    #======================#
    # PDDL action messages #
    #======================#

    # PDDL action dispatch callback
    def dispatch_callback(self, pddl_action_msg):
        self.action_params = self.cfg_actions['actions']
        if self.debug == True:
            print(pddl_action_msg)
        self.action_params = self.get_pddl_action_params(pddl_action_msg)
        if self.debug == True:
            pass
            # print("Action Interface Manager subscripted to plan dispatcher.")
            # print("Action params: \n")
            # print(self.action_params)

        if not pddl_action_msg.name in self._action_interfaces:
            # manager does not handle this PDDL action
            if self.debug == True:
                print("Action name unknown. Add it to config file.")
            return

        # Publish feedback: action enabled
        self.publish_feedback(pddl_action_msg.plan_id, pddl_action_msg.action_id, ActionFeedback.ACTION_DISPATCHED_TO_GOAL_STATE)

        # Set the start effects
        self._kb_link.kb_apply_action_effects(pddl_action_msg, RPKnowledgeBaseLink.AT_START)

        # find and run action interface
        current_interface = self._action_interfaces[pddl_action_msg.name]
        current_interface.run(pddl_action_msg)

    # PDDL action feedback
    def publish_feedback(self, plan_id, action_id, status):
        fb = ActionFeedback()
        fb.action_id = action_id
        fb.plan_id = plan_id
        fb.status = status
        self._action_feedback_pub.publish(fb)
        # if self.debug:
        print("Publishing Action Feedback")
        print("Action feedback: \n")
        print(fb)


    #==================#
    # interface status #
    #==================#

    def run(self):
        rate = rospy.Rate(10)
        if self.debug:
            print("Action Interface Manager Running")
        while not rospy.is_shutdown():        
            # iterate through interfaces and send feedback
            for interface in self._action_interfaces.values():
                for act in interface._action_status.keys():
                    if interface._action_status[act] == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE:
                        # Apply the end effects
                        self._kb_link.kb_apply_action_effects(interface._action_dispatch_msg[act], RPKnowledgeBaseLink.AT_END)
                    
                    if interface._action_status[act] == ActionFeedback.ACTION_FAILED:
                        for i in range(10):
                            print("ACTION FAILED")

                    # action completed (achieved or failed)
                    if interface._action_status[act] == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE or interface._action_status[act] == ActionFeedback.ACTION_FAILED:
                        rospy.loginfo('KCL: ({}) Reporting action complete: {} {}'.format(rospy.get_name(), act, interface._action_name))
                        self.publish_feedback(act[0], act[1], interface._action_status[act])
                        # remove completed action data from interface
                        interface.clean_action(act)

            rate.sleep()

    #==============#
    # YAML parsing #
    #==============#

    # parse YAML config and create action interfaces
    def parse_config(self):
        for action in self.action_params:
            if self.debug == True:
                print("Action:")
                print(action)
            if action["interface_type"] == "actionlib":
                self._action_interfaces[action["name"]] = ActionlibActionInterface(action)
            if action["interface_type"] == "service":
                self._action_interfaces[action["name"]] = ServiceActionInterface(action)
            if action["interface_type"] == "fsm":
                self._action_interfaces[action["name"]] = FSMActionInterface(action)

    def get_pddl_action_params(self, dispatched_action):
        action_name = dispatched_action.name
        action_id = dispatched_action.action_id
        plan_id = dispatched_action.plan_id
        action_params = dispatched_action.parameters
        start_location = action_params[1].value
        end_location = action_params[2].value
        if self.debug:
            print("Action name: " + str(action_name) + "\n")
            print("Action id: " + str(action_id) + "\n")
            print("Action params: " + str(action_params) + "\n")
            print("Plan id: " + str(plan_id) + "\n")

        if action_name == "goto_waypoint":
            if self.debug == True:
                print("goto_waypoint action received")
            start_point = action_params[1].value
            end_point = action_params[2].value
            self.end_point = end_point
            return(action_name, action_id, plan_id)

        elif action_name == "move":
            end_point = None
            locations_params = self.cfg_actions['locations']
            for location in locations_params:
                if location['name'] == end_location:
                    end_point = location['goal']
                elif location['name'] == start_location:
                    start_point = location['goal']

            self.end_point = end_point
            self.start_point = start_point
            if self.debug == True:
                print("Move action start point:")
                print(self.start_point)
                print("Move action end point:")
                print(self.end_point)

            # switch of default goal to goal (location) defined in config file
            i = 0
            for action in self.action_params:
                if self.debug == True:
                    pass
                    # print("ACTION")
                    # print(action)
                    # print("TYPE OF ACTION")
                    # print(type(action))
                if action['name'] == 'move':
                    self.action_params[i]['default_actionlib_goal'] = self.end_point
                i += 1

        elif action_name == "rescue":
            # switch of default goal to goal (location) defined in config file
            i = 0
            for action in self.action_params:
                if self.debug == True:
                    pass
                    # print("ACTION")
                    # print(action)
                    # print("TYPE OF ACTION")
                    # print(type(action))
        
        elif action_name == "event-end-move":
            # switch of default goal to goal (location) defined in config file
            i = 0
            for action in self.action_params:
                if self.debug == True:
                    pass
                    # print("ACTION")
                    # print(action)
                    # print("TYPE OF ACTION")
                    # print(type(action))
        
        elif action_name == "event-end-rescue":
            # switch of default goal to goal (location) defined in config file
            i = 0
            for action in self.action_params:
                if self.debug == True:
                    pass
                    # print("ACTION")
                    # print(action)
                    # print("TYPE OF ACTION")
                    # print(type(action))

            # return(action_name, action_id, plan_id, start_point, end_point)
        return(action_name, action_id, plan_id)

if __name__ == '__main__':
    rospy.init_node('RPActionInterfaceManager')
    aim = ActionInterfaceManager()
    rospy.spin()
