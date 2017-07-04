

#include "vrep_ros_robotis_plugin/RobotisApp.h"
#include "v_repLib.h"

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <boost/lexical_cast.hpp>

RobotisApp::RobotisApp():
  cnt_(1),
  control_cycle_sec_(0.008)
{

}

RobotisApp::~RobotisApp()
{

}

void RobotisApp::initialize()
{
  queue_thread_ = boost::thread(boost::bind(&RobotisApp::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/present_joint_state", 1);


  joint_controller_ = new VrepJointController();
  joint_controller_->Initialize();
}

void RobotisApp::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber set_mode_sub_ = ros_node.subscribe("/robotis/set_mode", 5, &RobotisApp::setModeCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void RobotisApp::stop()
{

}

void RobotisApp::process()
{
  ros::Time now = ros::Time::now();

  if (now - last_publish_ < ros::Duration(0.01))
    return;

  joint_controller_->RecvState();

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = now;

  joint_state_msg.name.push_back("ur5_joint1");
  joint_state_msg.name.push_back("ur5_joint2");
  joint_state_msg.name.push_back("ur5_joint3");
  joint_state_msg.name.push_back("ur5_joint4");
  joint_state_msg.name.push_back("ur5_joint5");
  joint_state_msg.name.push_back("ur5_joint6");

  for (int id=0; id<6; id++)
  {
    joint_state_msg.position.push_back(joint_controller_->present_joint_position_[id]);
    joint_state_msg.velocity.push_back(joint_controller_->present_joint_velocity_[id]);
    joint_state_msg.effort.push_back(joint_controller_->present_joint_torque_[id]);
  }

  joint_state_pub_.publish(joint_state_msg);

  last_publish_ = now;
}

void RobotisApp::setModeCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string mode = msg->data;

  ROS_INFO("Mode : %s", mode.c_str());

  return;
}
