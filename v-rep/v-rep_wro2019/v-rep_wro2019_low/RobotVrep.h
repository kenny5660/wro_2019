#pragma once
#include "b0RemoteApi.h"
class RobotVrep : public RobotGardener {
 public:
  RobotVrep(std::shared_ptr<b0RemoteApi> cl);
  std::shared_ptr<b0RemoteApi> GetClient();
  void Init() override;
  void GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) override;
  ~RobotVrep();

 private:
 std::shared_ptr<b0RemoteApi> cl_;
};
