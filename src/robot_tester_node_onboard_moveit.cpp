/*
 * robot_tester_node.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: steve
 */


#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <niryo_one_msgs/RobotMoveAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>

#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/move_group/capability_names.h>

#include <sstream>

using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
typedef actionlib::SimpleActionServer<niryo_one_msgs::RobotMoveAction> Server;

std::vector<float> currentJointStates;

const std::string NS = "";

void jointStatesCallback(const sensor_msgs::JointStateConstPtr & msg){
  if (currentJointStates.empty()){
    for (size_t i = 0; i < msg->position.size(); i++){
      currentJointStates.push_back(msg->position[i]);
    }
  } else {
    for (size_t i = 0; i < msg->position.size(); i++){
      currentJointStates[i]=msg->position[i];
    }
  }
  //ROS_INFO("New JOint states received");
	return;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "robotTeseter");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  string planning_group_name_ = "arm";


  string is;
  getline(std::cin,is);
  ROS_INFO("Starting robot tester");
  ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(1);
    sleep_t.sleep();
    ROS_INFO("No Subcriber found to topic %s planning_scene", NS.c_str());
  }

  moveit::planning_interface::MoveGroupInterface::Options loadOptions("arm");
  ROS_INFO("Befor changing descriptopn: %s", loadOptions.robot_description_.c_str());
  //loadOptions.robot_description_=NS + loadOptions.robot_description_;
  ROS_INFO("after changing descriptopn: %s", loadOptions.robot_description_.c_str());
  loadOptions.topic_namespace_=NS;

 // getline(std::cin,is);

  //move_group::MOVE_ACTION = "planningNS" + move_group::MOVE_ACTION;
  moveit::planning_interface::MoveGroupInterface move_group(loadOptions);
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Model loaded!!!");

  getline(std::cin,is);
  ROS_INFO("Getting current state");
  //robot_state::RobotStatePtr pointer= move_group.getCurrentState();
  //getline(std::cin,is);


  ROS_INFO("Planning move");

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.2;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.2;
  move_group.setPoseTarget(target_pose1);
  getline(std::cin,is);
  ROS_INFO("Target set");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Success? %s", success ? "yes" : "no");

  getline(std::cin,is);
  //robot_state::RobotStatePtr pointer2= move_group.getCurrentState();
//  ros::Subscriber jointStateSub = n.subscribe("/joint_states", 10, jointStatesCallback);
//
//
//
//  string is;
//  getline(std::cin,is);
//
//  moveit::planning_interface::MoveGroupInterface::Options loadOptions("arm");
//  ROS_INFO("Befor changing descriptopn: %s", loadOptions.robot_description_.c_str());
//  //loadOptions.robot_description_="planningNS/" + loadOptions.robot_description_;
//  ROS_INFO("after changing descriptopn: %s", loadOptions.robot_description_.c_str());
//
//
//  //getline(std::cin,is);
//
//  moveit::planning_interface::MoveGroupInterface move_group(loadOptions);
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//
//  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
//  ROS_INFO("target frame: %s", move_group.getEndEffectorLink().c_str());
//
//  ros::spinOnce();
//
//
//  std::vector<float> newJointStates = currentJointStates;
//  newJointStates[0] = 0;


//  group_variable_values = move_group.getCurrentJointValues();
//  geometry_msgs::Pose target_pose1;
//  target_pose1.orientation.w = 1.0;
//  target_pose1.position.x = 0.2;
//  target_pose1.position.y = -0.2;
//  target_pose1.position.z = 0.2;
//  move_group.setPoseTarget(target_pose1);
//  ROS_INFO("TargetPose Set");
////  //getline(std::cin,is);
//
//  ROS_INFO("Get joint states");
//
//  std::vector<double> group_variable_values;
//  robot_state::RobotStatePtr robotstate;
//  //move_group.getCurrentState(robotstate,10);
//  group_variable_values = move_group.getCurrentJointValues();
//  //move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName());
//  //ROS_INFO("HAs joint modelGrou %s", hasMGroup ? "YES" : "NO");
//
//
////
//  ROS_INFO("Waiting for action server to start.");

    actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction> ac("/niryo_one/commander/robot_action/",true);
    //ros::Publisher as = n.advertise<std_msgs::String>("/dummy",1000);
  //
  //    std_msgs::String ss;
  //    ss.data = "Helo";
  //    as.publish(ss);

//
//////  // wait for the action server to start
//  //ac.waitForActionServerToStart(ros::Duration(10.0)); //will wait for infinite time
    ac.waitForServer(ros::Duration(10.0));



    if (success) {
      ROS_INFO("Sending command");
      niryo_one_msgs::TrajectoryPlan plan;
      niryo_one_msgs::RobotMoveCommand cmd;
      cmd.cmd_type=7;
      plan.group_name = "arm";
      plan.trajectory_start = my_plan.start_state_;
      plan.trajectory = my_plan.trajectory_;
      cmd.Trajectory = plan;

      niryo_one_msgs::RobotMoveActionGoal action;
      action.goal.cmd = cmd;
      ac.sendGoal(action.goal);
      ac.waitForResult(ros::Duration(30.0));
    }


//////
//  ROS_INFO("Action server started, sending goal.");
//////  // send a goal to the action
//  niryo_one_msgs::RobotMoveActionGoal action;
//  niryo_one_msgs::RobotMoveCommand command;
//  geometry_msgs::Point point;
//  point.x = 0.2;
//  point.y = 0.0;
//  point.z = 0.2;
//  command.cmd_type = 2;
//  command.position = point;
//  action.goal.cmd = command;
//  //ac.sendGoal(action.goal);

  //ac.waitForResult(ros::Duration(30.0));
//
//  string is;
//  getline(std::cin,is);
//
//
//  string planning_group_name_ = "arm";
//  move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
//  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface);
////
//  ROS_INFO("loaded move group");
//  string is;
//  getline(std::cin,is);
//
//  //bool success =  false;
////  // Raw pointers are frequently used to refer to the planning group for improved performance.
////  try{
////
////  }catch(){
////
////  }
//  const robot_state::JointModelGroup *joint_model_group =
//      move_group_->getCurrentState()->getJointModelGroup(planning_group_name_);
//
//  ROS_INFO("Joint model group fetched");
//  getline(std::cin,is);
//
//  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
//
//  std::vector<double> joint_group_positions;
//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//
//  ROS_INFO("State of axis 1 read %f", joint_group_positions[0]);
//  joint_group_positions[0] = 0;
//
//  getline(std::cin,is);
//
//  move_group_->setJointValueTarget(joint_group_positions);


//  // Getting Basic Information
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // We can print the name of the reference frame for this robot.
//  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group_->getPlanningFrame().c_str());
//
//  // We can also print the name of the end-effector link for this group.
//  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_->getEndEffectorLink().c_str());
//
//  // Planning to a Pose goal
//  // ^^^^^^^^^^^^^^^^^^^^^^^
//  // We can plan a motion for this group to a desired pose for the
//  // end-effector.
//  geometry_msgs::Pose target_pose1;
//  target_pose1.orientation.w = 1;
//  target_pose1.position.x = 0.2;
//  target_pose1.position.y = 0.2;
//  target_pose1.position.z = 0.2;
//  move_group_->setPoseTarget(target_pose1);

//  ROS_INFO("Targetpose set");
//  getline(std::cin,is);
//
//  // Now, we call the planner to compute the plan and visualize it.
//  // Note that we are just planning, not asking move_group
//  // to actually move the robot.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//
//  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//
//  ROS_INFO("Planed trajectory, exectue?");
//  getline(std::cin,is);
//
//  if (success) move_group_->execute(my_plan);
//
//  ROS_INFO("loaded move group");
//  getline(std::cin,is);
  int sign = -1;
  int count = 0;
  while (ros::ok())
  {

//    point.x = 0.2;
//    point.y = sign * 0.2;
//    point.z = 0.2;
//    command.cmd_type = 2;
//    command.position = point;
//    action.goal.cmd = command;
//    ac.sendGoal(action.goal);
//    ac.waitForResult(ros::Duration(30.0));

//    ss.data = "Hello World" + std::to_string(count);
//    as.publish(ss);


//    ROS_INFO("publishiung %s",ss.data.c_str());

	ROS_INFO("Spinned Once");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    sign = sign * (-1);
  }


  return 0;
}
