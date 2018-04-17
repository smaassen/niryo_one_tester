/*
 * robot_tester_node.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: steve
 */


#include "ros/ros.h"


#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>

#include <sstream>

using namespace std;

double x_goal = 0.2;
double y_goal = 0.1;
double z_goal = 0.25;

double yaw_goal = 0.5;
double pitch_goal = 0.6;
double roll_goal = 0.8;

double quat_x = 0;
double quat_y = 0;
double quat_z = -0.644;
double quat_w =  0.765;


int toolID = 12; // change this depenidning on the tool mounted


typedef actionlib::SimpleActionServer<niryo_one_msgs::RobotMoveAction> NiryoServer;
typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction> NiryoClient;
typedef pair<geometry_msgs::Point,niryo_one_msgs::RPY> NiryoPose;



int main(int argc, char **argv)
{

  ros::init(argc, argv, "niryo_one_tester");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Connecting to the robot ===========================================
  ROS_INFO("Connecting to robot  ========================");
  NiryoClient ac("/niryo_one/commander/robot_action/",true);
  // wait for the action server to start
  bool connection_success = false;
  while (!connection_success) {
    connection_success = ac.waitForServer(ros::Duration(3.0));
    if (connection_success) {
      ROS_INFO("  Robot Connection established");
    } else {
      ROS_WARN("  Error connecting to Robot. Trying again");
    }
  }

  // Setting Gripper
  // Setting gripper
  ROS_INFO("Setting gripper");
  ros::ServiceClient changeToolClient_;
  changeToolClient_ = n.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/change_tool/");
  niryo_one_msgs::SetInt srv;
  srv.request.value = toolID;
  while (!changeToolClient_.call(srv)){
    ROS_WARN("  Could not set the tool type. Trying again in one second");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("  Success");

  string is;
  ROS_INFO("Send position command (rpy) ========================");
  ROS_INFO("  Press enter to send ...");
  getline(std::cin,is);



  niryo_one_msgs::ToolCommand tcmd;
  niryo_one_msgs::RobotMoveActionGoal action;
  niryo_one_msgs::RobotMoveCommand cmd;
  geometry_msgs::Point p;
  niryo_one_msgs::RPY rot;

  // ========================================================
  // Sending a position command =============================
  // ========================================================

  rot.roll = roll_goal;
  rot.pitch = pitch_goal;
  rot.yaw = yaw_goal;
  p.x = x_goal;
  p.y = y_goal;
  p.z = z_goal;
  NiryoPose pose1(p,rot);





  cmd.cmd_type = 2;
  cmd.position = pose1.first;
  cmd.rpy = pose1.second;
  ROS_INFO("  Sending command :" );
  ROS_INFO("    position: %f, %f, %f", cmd.position.x, cmd.position.y, cmd.position.z);
  ROS_INFO("    roll, pitch, yaw:  %f, %f, %f", cmd.rpy.roll, cmd.rpy.pitch, cmd.rpy.yaw);
  action.goal.cmd = cmd;
  ac.sendGoal(action.goal);
  bool success = ac.waitForResult(ros::Duration(5.0));


  ROS_INFO("Send position command (quaternion) ========================");
  ROS_INFO("  Press enter to send ...");
  getline(std::cin,is);
  geometry_msgs::Pose pose_quat;


  pose_quat.position.x = x_goal;
  pose_quat.position.y = -y_goal; // for better demonstration purposes
  pose_quat.position.z = z_goal;
  pose_quat.orientation.x = quat_x;
  pose_quat.orientation.y = quat_y;
  pose_quat.orientation.z = quat_z;
  pose_quat.orientation.w = quat_w;


  cmd.cmd_type = 8;
  cmd.pose_quat = pose_quat;
  ROS_INFO("  Sending command :" );
  ROS_INFO("    position: %f, %f, %f", cmd.pose_quat.position.x, cmd.pose_quat.position.y, cmd.pose_quat.position.z);
  ROS_INFO("    orientation (x,y,z,w):  %f, %f, %f, %f", cmd.pose_quat.orientation.x, cmd.pose_quat.orientation.y,
           cmd.pose_quat.orientation.z, cmd.pose_quat.orientation.w);
  action.goal.cmd = cmd;
  ac.sendGoal(action.goal);
  success = ac.waitForResult(ros::Duration(5.0));



  ROS_INFO("Send gripper command (open) ========================");
  ROS_INFO("  Press enter to send ...");
  getline(std::cin,is);
  tcmd.cmd_type = 1;
  tcmd.gripper_open_speed = 100;
  tcmd.tool_id = 12;
  action.goal.cmd.cmd_type = 6;
  action.goal.cmd.tool_cmd = tcmd;
  ac.sendGoal(action.goal);
  ac.waitForResult(ros::Duration(10.0));


  ROS_INFO("Send gripper command (close) ========================");
  ROS_INFO("  Press enter to send ...");
  getline(std::cin,is);
  tcmd.cmd_type = 2;
  tcmd.gripper_open_speed = 100;
  tcmd.tool_id = 12;
  action.goal.cmd.cmd_type = 6;
  action.goal.cmd.tool_cmd = tcmd;
  ac.sendGoal(action.goal);
  ac.waitForResult(ros::Duration(10.0));


  return 0;
}
