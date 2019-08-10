#include "Robot.h"
RobotVrep::RobotVrep(std::shared_ptr<b0RemoteApi> cl) : cl_(cl) {}

std::shared_ptr<b0RemoteApi> RobotVrep::GetClient() {
  return cl_; }

void RobotVrep::Init() {

	cl_->simxAddStatusbarMessage("Init robot done!", cl_->simxDefaultPublisher());
	cl_->simxStartSimulation(cl_->simxDefaultPublisher()); 
}

void RobotVrep::GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) {


}

RobotVrep::~RobotVrep() {


}
