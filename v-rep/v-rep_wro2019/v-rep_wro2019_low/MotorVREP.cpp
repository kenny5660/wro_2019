#include "Motor.h"
#include <thread>
MotorVREP::MotorVREP(std::shared_ptr<b0RemoteApi> cl, int handle, bool inverted,
                     int counts_per_deg)
    : cl_(cl),
      handle_(handle),
      inverted_coef_(inverted ? -1 : 1),
      counts_per_deg_(counts_per_deg) {}

double MotorVREP::GetCurEnc() const {
 auto msg =  cl_->simxGetJointPosition(handle_, cl_->simxDefaultPublisher());
  return (b0RemoteApi::readFloat(msg, 1) * 180.0 / M_PI * inverted_coef_) -
         real_pos_;
}

double MotorVREP::GetCurEncDeg() const {
  auto msg = cl_->simxGetJointPosition(handle_, cl_->simxDefaultPublisher());
  return (b0RemoteApi::readFloat(msg, 1) * 180.0 / M_PI * inverted_coef_) -
         real_pos_;
}

double MotorVREP::GetCurEncRot() const { return GetCurEncDeg()/360.0; }

void MotorVREP::Stop() { MoveContinue(0); }

void MotorVREP::MoveContinue(double speed) {
  //cl_->simxSetObjectIntParameter(handle_,kJointintparam_ctrl_enabled, 0, cl_->simxDefaultPublisher());
  cl_->simxSetJointTargetVelocity(handle_, (float)speed/180.0*M_PI * inverted_coef_, cl_->simxDefaultPublisher());
  //cl_->simxSpinOnce();
  cl_->simxSleep(1);
}

void MotorVREP::MoveTime(double speed, int msec) { 
	MoveContinue(speed);  
	std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

void MotorVREP::MoveIncEncCounts(double speed, double counts, bool wait) {
  MoveToEncCounts(speed, GetCurEncDeg()+counts,wait);
}

void MotorVREP::MoveIncDeg(double speed, double deg, bool wait) {
  MoveIncEncCounts(speed, deg * counts_per_deg_, wait);
}

void MotorVREP::MoveIncRot(double speed, double rots, bool wait) {
  MoveIncDeg(speed, rots * 360, wait);
}

void MotorVREP::MoveToEncCounts(double speed, double counts, bool wait) {
  target_pos_ = counts;
  cl_->simxSetObjectIntParameter(handle_, kJointintparam_ctrl_enabled, 1,cl_->simxDefaultPublisher());
  // joint upper velocity limit.
  // May only have an effect after simulation restart, or after resetting the
  // joint with sim.resetDynamicObject
  cl_->simxSetObjectFloatParameter(handle_, kJointfloatparam_upper_limit, speed,
                                 cl_->simxDefaultPublisher());

  cl_->simxSetJointTargetPosition(
      handle_, (counts + real_pos_) / 180.0 * M_PI * inverted_coef_,
                                  cl_->simxDefaultPublisher());
  if (wait)
  {
    while (IsReady());
  }

}

void MotorVREP::MoveToDeg(double speed, double deg, bool wait) {
  MoveToEncCounts(speed, deg * counts_per_deg_, wait);
}

void MotorVREP::MoveToRot(double speed, double rots, bool wait) {
  MoveToDeg(speed, rots * 360, wait);
}

bool MotorVREP::IsReady() { return abs(target_pos_ - GetCurEncDeg()) < 1; }

void MotorVREP::ResetEnc() { target_pos_ = 0;
  auto msg = cl_->simxGetJointPosition(handle_, cl_->simxDefaultPublisher());
  real_pos_ = b0RemoteApi::readFloat(msg, 1) * 180.0 / M_PI * inverted_coef_;
  
}
