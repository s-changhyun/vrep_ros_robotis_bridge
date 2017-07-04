
#pragma once

#include "v_repLib.h"

#include <std_msgs/String.h>

class VrepJointController
{
public:
  VrepJointController();
  virtual ~VrepJointController();

public:
  int SetControlMode(std::string controlMode);
  int Initialize(void);
  int Finalize(void);
  int Enable(void);
  int Disable(void);
  int ClearFault(void);
  int Homing(void);
  int Stop(void);
  int Enable(size_t index);
  int Disalbe(size_t index);
  int ClearFault(size_t index);
  int Homing(size_t index);
  int Stop(size_t index);
  int GetStatus(size_t index, uint32_t &status);
  int SetTorque(size_t index, double_t torque);
  int GetTorque(size_t index, double_t &torque);
  int SetVelocity(size_t index, double_t velocity);
  int GetVelocity(size_t index, double_t &velocity);
  int SetPosition(size_t index, double_t position);
  int GetPosition(size_t index, double_t &position);
  int GetStatus(std::vector<uint32_t> &status);
  int SetTorque(std::vector<double_t> &torque);
  int GetTorque(std::vector<double_t> &torque);
  int SetVelocity(std::vector<double_t> &velocity);
  int GetVelocity(std::vector<double_t> &velocity);
  int SetPosition(std::vector<double_t> &position);
  int GetPosition(std::vector<double_t> &position);
  int SendCommand(void);
  int RecvState(void);

  size_t joint_num_;
  std::vector<simInt> joints_handle_;

  std::vector<simFloat> goal_joint_toruqe_;
  std::vector<simFloat> present_joint_torque_;
  std::vector<simFloat> goal_joint_velocity_;
  std::vector<simFloat> present_joint_velocity_;
  std::vector<simFloat> goal_joint_position_;
  std::vector<simFloat> present_joint_position_;

protected:
  int VrepInit(void);
  int VrepDeinit(void);

  int VrepEnable(simInt handle);
  int VrepDisable(simInt handle);
  int VrepSetOperationMode(simInt handle, simInt operationMode);
  int VrepGetOperationMode(simInt handle, simInt &operationMode);
  int VrepSetTorque(simInt handle, simFloat torque, simFloat maximumVelocity);
  int VrepGetTorque(simInt handle, simFloat &torque);
  int VrepSetVelocity(simInt handle, simFloat velocity);
  int VrepGetVelocity(simInt handle, simFloat &velocity);
  int VrepSetPosition(simInt handle, simFloat position);
  int VrepGetPosition(simInt handle, simFloat &position);
};
