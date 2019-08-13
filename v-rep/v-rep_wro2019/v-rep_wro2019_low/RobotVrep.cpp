#include "Robot.h"
RobotVrep::RobotVrep(std::shared_ptr<b0RemoteApi> cl) : cl_(cl) {}
int GetObjectHandle(std::shared_ptr<b0RemoteApi> cl, std::string name) {
  auto msg = cl->simxGetObjectHandle(name.c_str(), cl->simxServiceCall());
  return b0RemoteApi::readInt(msg, 1);
}

std::shared_ptr<b0RemoteApi> RobotVrep::GetClient() { return cl_; }

void RobotVrep::Init() {
  std::shared_ptr<MotorVREP> motor_front(
      new MotorVREP(cl_, GetObjectHandle(cl_, "OmniWheel3"), false));
  std::shared_ptr<MotorVREP> motor_left(
      new MotorVREP(cl_, GetObjectHandle(cl_, "OmniWheel2"), false));
  std::shared_ptr<MotorVREP> motor_back(
      new MotorVREP(cl_, GetObjectHandle(cl_, "OmniWheel4"), false));
  std::shared_ptr<MotorVREP> motor_right(
      new MotorVREP(cl_, GetObjectHandle(cl_, "OmniWheel1"), false));
  omni_ = std::shared_ptr<OmniWheels4Squre>(new OmniWheels4Squre(
      50, 131, motor_left, motor_front, motor_right, motor_back));

  cl_->simxAddStatusbarMessage("Init robot done!", cl_->simxDefaultPublisher());
  cl_->simxStartSimulation(cl_->simxDefaultPublisher());
}

void RobotVrep::GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) {}

RobotVrep::~RobotVrep() {}
