#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <dyplom/goalAction.h>
#include <dyplom/goalFeedback.h>
#include <dyplom/goalResult.h>
#include <dyplom/goalGoal.h>
#include "costmap_2d/costmap_2d_ros.h"
#include "global_planner/planner_core.h"
#include <tf2_ros/transform_listener.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include "dwa_local_planner/dwa_planner_ros.h"
#include <dwa_local_planner/DWAPlannerConfig.h>
#include "rotate_recovery/rotate_recovery.h"
#include <actionlib/server/simple_action_server.h>
#include <vector>
#include <cmath>
#include <string>

std::vector <geometry_msgs::PoseStamped> globalPlan;
global_planner::GlobalPlanner globalPlanner;

geometry_msgs::PoseStamped goalPose;
geometry_msgs::PoseStamped currentPose;
nav_msgs::Odometry pose;
geometry_msgs::Twist twistVelocity;
dyplom::goalAction action;
dyplom::goalFeedback actionFeedback;
dyplom::goalGoal actionGoal;
dyplom::goalResult actionResult;

typedef actionlib::SimpleActionServer<dyplom::goalAction> Server;

int globalPathElements = 0;
float poseX;
float poseY;

bool setGlobalTrajectory = false;
bool goalSet = false;

float goalPoseX;
float goalPoseY;
float currentPoseX;
float currentPoseY;

void goalCallback (const geometry_msgs::PoseStamped::ConstPtr& msg){

    geometry_msgs::PoseStamped tempGoalPose = *msg;
    if (tempGoalPose.pose.position.x != goalPose.pose.position.x || tempGoalPose.pose.position.y != goalPose.pose.position.y){
      goalPose = *msg;
      goalPose.header.frame_id = "map";
      setGlobalTrajectory = true;
      goalSet = true;

      goalPoseX = goalPose.pose.position.x;
      goalPoseY = goalPose.pose.position.y;

      ROS_INFO("Goal x: %f, Goal y: %f", goalPoseX, goalPoseY);
      ROS_INFO("New global trajectory set.");
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

  currentPose.pose =  msg->pose.pose;
  currentPose.header = msg->header;
  currentPose.header.frame_id = "map";

  currentPoseX = currentPose.pose.position.x;
  currentPoseY = currentPose.pose.position.y;

  ROS_INFO("Path elements: %f",globalPathElements);
  ROS_INFO("Current x: %f, Current y: %f", currentPoseX, currentPoseY);

  float dist_to_goal=sqrt(pow(goalPoseX-currentPoseX, 2)+pow(goalPoseY-currentPoseY, 2));
  ROS_INFO("Distance to goal %f", dist_to_goal);
}

// void actionCallback(const dyplom::goalGoalConstPtr& goal, Server* as)
// {
//   actionResult.status="received";
//   as->setSucceeded(actionResult);

//   // goalPose.header.frame_id = "map";
//   // goalPose.header.stamp = rospy.Time.now()
//   // goalPose.pose.position.x = goal->x;
//   // goalPose.pose.position.y = goal->y;
//   // goalPose.pose.position.z = goal->z;
//   // goalPose.pose.orientation.x = goal->ox;
//   // goalPose.pose.orientation.y = goal->oy;
//   // goalPose.pose.orientation.z = goal->oz;
//   // goalPose.pose.orientation.w = goal->ow;

//   goalPose.header.frame_id = "map";
//   // goalPose.header.stamp = rospy.Time.now();
//   goalPose.pose.position.x = 1;
//   goalPose.pose.position.y = 1;
//   goalPose.pose.position.z = 0;
//   goalPose.pose.orientation.x = 0;
//   goalPose.pose.orientation.y = 0;
//   goalPose.pose.orientation.z = 0;
//   goalPose.pose.orientation.w = 1;

//   goalPoseX = goal->x;
//   goalPoseY = goal->y;
//   std::cout<<"goal"<<std::endl;
//   std::cout<<goalPoseX<<std::endl;
//   std::cout<<goalPoseY<<std::endl;
//   setGlobalTrajectory = true;
//   goalSet = true;
// }


int main(int argc, char **argv)
{

  // goalPose.header.frame_id = "map";
  // // goalPose.header.stamp = rospy.Time.now();
  // goalPose.pose.position.x = 1;
  // goalPose.pose.position.y = 1;
  // goalPose.pose.position.z = 0;
  // goalPose.pose.orientation.x = 0;
  // goalPose.pose.orientation.y = 0;
  // goalPose.pose.orientation.z = 0;
  // goalPose.pose.orientation.w = 1;

  ros::init(argc, argv, "planner");
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  ros::Subscriber globalGoalSub = node.subscribe("/move_base_simple/goal", 1000, goalCallback);
  ros::Subscriber currentPoseSub = node.subscribe("/mobile_base_controller/odom", 1000, odomCallback);
  // Server server(node, "destination", boost::bind(&actionCallback, _1, &server), false);
  // server.start();
  ros::Publisher twistVelocityPublisher = node.advertise <geometry_msgs::Twist> ("/key_vel", 1000);

  tf2_ros::Buffer buffer (ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS localCostmap("local_costmap", buffer);
  costmap_2d::Costmap2DROS globalCostmap("global_costmap", buffer);
  
  global_planner::GlobalPlanner globalPlanner("global_planner", globalCostmap.getCostmap(), "map");
  
  base_local_planner::TrajectoryPlannerROS localPlanner;
  localPlanner.initialize("local_planner", &buffer, &localCostmap);

  rotate_recovery::RotateRecovery recoveryBehaviourRotate;
  recoveryBehaviourRotate.initialize("recovery", &buffer, &localCostmap, &localCostmap);

  while (ros::ok()){
    std::cout<<"Global goal";
    std::cout<<goalPoseX<<" "<<goalPoseY;
    if (setGlobalTrajectory == true){
    
      // currentPose = 
      if (globalPlanner.makePlan(currentPose, goalPose, globalPlan) == true){
        localPlanner.setPlan(globalPlan);

        for (int elt = 0; elt<globalPlan.size(); elt++){
          geometry_msgs::PoseStamped pose = globalPlan[elt];

          poseX = pose.pose.position.x;
          poseY = pose.pose.position.y;

          ROS_INFO("Pozycja x: %f",poseX);
          ROS_INFO("Pozycja y: %f",poseY);

          globalPathElements++;
        }

        ROS_INFO("Liczba elementow sciezki: %f",globalPathElements);

        setGlobalTrajectory = false;
        ROS_INFO("Global plan created successfully.");
        ROS_INFO("Local plan set successfully.");
      }
      else{
        ROS_INFO("Global plan not created.");
        ROS_INFO("Local plan not set.");
      }
    }

    if (goalSet == true){
      if (localPlanner.computeVelocityCommands(twistVelocity)){
        twistVelocityPublisher.publish(twistVelocity);
        ROS_INFO("Linear x: %f, Linear y: %f, Angular z: %f", twistVelocity.linear.x, twistVelocity.linear.y, twistVelocity.angular.z);

        if (localPlanner.isGoalReached()){
          goalSet = false;
          ROS_INFO("Goal reached.");
        }
      }
      else{
        twistVelocity.linear.x = 0.1;
        twistVelocity.angular.z = 0.3;
        twistVelocityPublisher.publish(twistVelocity);
        ROS_INFO("Linear x: %f, Linear y: %f, Angular z: %f", twistVelocity.linear.x, twistVelocity.linear.y, twistVelocity.angular.z);
        ROS_INFO("Recovery behavior");
        // costmap_2d.reset();

        //recovery behavior nie dziala w sposob pozwalajacy na wyjscie z zakleszczenia
        //recoveryBehaviourRotate.runBehavior();
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

