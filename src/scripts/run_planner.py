#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from dyplom.srv import problem_gen
from rosplan_dispatch_msgs.srv import ProblemService, ParsingService, PlanningService
from rosplan_knowledge_msgs.srv import GenerateProblemService


def rosplan_client():
    plan_ok = True

    (status, info) = call_problem_generator()
    if status is True and plan_ok is True:
        print(info)
        print("Problem was generated")
    else:
        print("Error while generating problem")
        plan_ok = False

    (status, info) = call_planner()
    if status is True and plan_ok is True:
        print(info)
    else:
        plan_ok = False

    (status, info) = call_problem_generator()
    if status is True and plan_ok is True:
        print(info)
    else:
        plan_ok = False

    (status, info) = call_problem_generator()
    if status is True and plan_ok is True:
        print(info)
    else:
        plan_ok = False
    

def call_problem_generator():
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    try:
        pg_server = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', problem_gen)
        problem_gen_response = pg_server()
        return (True, problem_gen_response)
    except rospy.ServiceException as e:
        print("Problem generation service call failed: %s"%e)
        return(False, "")


def call_planner():
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    try:
        pl_server = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', problem_gen)
        planner_response = pl_server()
        return (True, planner_response)
    except rospy.ServiceException as e:
        print("Planner service call failed: %s"%e)
        return(False, "")


def call_parser():
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    try:
        par_server = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', problem_gen)
        parser_response = par_server()
        return (True, parser_response)
    except rospy.ServiceException as e:
        print("Problem parser service call failed: %s"%e)
        return (False, "")


def call_dispatcher():
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    try:
        disp_server = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', problem_gen)
        dispatcher_response = disp_server()
        return (True, dispatcher_response)
    except rospy.ServiceException as e:
        print("Problem dispatcher service call failed: %s"%e)
        return (False, "")


if __name__ == "__main__":
    rosplan_client()
