
#include <stdio.h>
#include "vrep_ros_robotis_plugin/VrepJointController.h"

VrepJointController::VrepJointController() :
  joint_num_(6)
{
  joints_handle_.resize(joint_num_);

  goal_joint_toruqe_.resize(joint_num_);
  present_joint_torque_.resize(joint_num_);
  goal_joint_velocity_.resize(joint_num_);
  present_joint_velocity_.resize(joint_num_);
  goal_joint_position_.resize(joint_num_);
  present_joint_position_.resize(joint_num_);
}

VrepJointController::~VrepJointController()
{

}

int VrepJointController::Initialize(void)
{
  return VrepInit();
}

int VrepJointController::VrepInit(void)
{
  VrepDeinit();

  std::string joint_name;
  simInt handle;

  joint_name = "UR5_joint1";
  handle = simGetObjectHandle(joint_name.c_str());
  joints_handle_.push_back(handle);
  joint_name = "UR5_joint2";
  handle = simGetObjectHandle(joint_name.c_str());
  joints_handle_.push_back(handle);
  joint_name = "UR5_joint3";
  handle = simGetObjectHandle(joint_name.c_str());
  joints_handle_.push_back(handle);
  joint_name = "UR5_joint4";
  handle = simGetObjectHandle(joint_name.c_str());
  joints_handle_.push_back(handle);
  joint_name = "UR5_joint5";
  handle = simGetObjectHandle(joint_name.c_str());
  joints_handle_.push_back(handle);
  joint_name = "UR5_joint6";
  handle = simGetObjectHandle(joint_name.c_str());
  joints_handle_.push_back(handle);

  return 0;
}

int VrepJointController::VrepDeinit(void)
{
  joints_handle_.clear();

  return 0;
}

int VrepJointController::RecvState(void)
{
  if(joints_handle_.size() != joint_num_) {
    return -1;
  }

  for(size_t i = 0; i < joint_num_; i++) {
    if(VrepGetTorque(joints_handle_[i], present_joint_torque_[i]) < 0) {
      return -1;
    }
    if(VrepGetVelocity(joints_handle_[i], present_joint_velocity_[i]) < 0) {
      return -1;
    }
    if(VrepGetPosition(joints_handle_[i], present_joint_position_[i]) < 0) {
      return -1;
    }
  }

  return 0;
}

int VrepJointController::VrepGetTorque(simInt handle, simFloat &torque)
{
  return simGetJointForce(handle, &torque);
}

int VrepJointController::VrepSetVelocity(simInt handle, simFloat velocity)
{
  return simSetJointTargetVelocity(handle, velocity);
}

int VrepJointController::VrepGetVelocity(simInt handle, simFloat &velocity)
{
  return simGetObjectFloatParameter(handle, 2012, &velocity);
}

int VrepJointController::VrepSetPosition(simInt handle, simFloat position)
{
  return simSetJointTargetPosition(handle, position);
}

int VrepJointController::VrepGetPosition(simInt handle, simFloat &position)
{
  return simGetJointPosition(handle, &position);
}
