#include <string.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher pick_pub = n.advertise<geometry_msgs::Point>("goal_command", 2);
  ros::Publisher pick_state_pub = n.advertise<std_msgs::Int32>("pick", 2);

  // create goal state msg
  std_msgs::Int32 gst;
 

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // define goal point and publish
  geometry_msgs::Point goal_coord;
  goal_coord.x = 4;
  goal_coord.y = -2;
  goal_coord.z = 0;
  // publish coordinates
  gst.data = 0;
  pick_pub.publish(goal_coord);
  // create initial virtual object with drop command
  pick_state_pub.publish(gst);


  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = goal_coord.x;
  goal.target_pose.pose.position.y = goal_coord.y;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick up point");

  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot reached its goal!");
    // simulate picking object
    ros::Duration(5).sleep();
    gst.data = 1;
    pick_state_pub.publish(gst);
  } 
  else
  {
    ROS_INFO("The base failed to reach its goal!");
  }
    
  // wait for 5 seconds
  ros::Duration(5).sleep();

  // define goal point and publish
  goal_coord.x = -5.0;
  goal_coord.y = -1.0;
  pick_pub.publish(goal_coord);

  // Define a second position and orientation for the robot to reach for drop off
  goal.target_pose.pose.position.x = goal_coord.x;
  goal.target_pose.pose.position.y = goal_coord.y;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off point");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot dropped the object!");
    ros::Duration(5).sleep();
    gst.data = 0;
    pick_state_pub.publish(gst);
  }
  else  
  {
    ROS_INFO("The robot failed to reach the goal!");
  }
   
  return 0;
}