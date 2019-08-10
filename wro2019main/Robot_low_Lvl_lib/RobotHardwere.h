#pragma once
class RobotGardenerHardwere : public RobotGardener
{
public:
	RobotGardenerHardwere();
	void Init() override;
	void GetLidarPolarPoints(std::vector<PolarPoint>& polar_points) override;


	~RobotGardenerHardwere();

};