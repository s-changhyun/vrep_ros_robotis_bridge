
#ifndef __JOINT_SPACE_CONTROLLER_H__
#define __JOINT_SPACE_CONTROLLER_H__

#include "v_repLib.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

#include <vrep_ros_robotis_plugin/VrepJointController.h>

class RobotisApp
{
public:
  RobotisApp();
  virtual ~RobotisApp();

  void initialize();
  void stop();
  void process();

  void setModeCallback(const std_msgs::String::ConstPtr& msg);

private:
  void queueThread();
  boost::thread queue_thread_;

  ros::Publisher joint_state_pub_;

  int cnt_;
  double control_cycle_sec_;

  ros::Time last_publish_;

protected:
  VrepJointController *joint_controller_;

};

#endif //ndef __JOINT_SPACE_CONTROLLER_H__
