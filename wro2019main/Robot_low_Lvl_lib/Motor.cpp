#include "Motor.h"
MotorKangaroo::MotorKangaroo(std::shared_ptr<KangarooDriver> kangarooDrv, 
	uint8_t chnl,
	bool inverted,
	int counts_per_deg)
	: kangaroo_drv_(kangarooDrv)
	, chnl_(chnl)
	, inverted_coef_(inverted ? -1:1)
	, counts_per_deg_(counts_per_deg) {
		kangaroo_drv_->CmdStart(chnl_);
		ResetEnc();
	}
void MotorKangaroo::MoveTime(int speed, int msec)
{
	throw std::runtime_error("Not implemented");
}
int MotorKangaroo::GetCurEnc() const
{
	auto params =	kangaroo_drv_->CmdGet(chnl_, KangarooDriver::kGetPos);
	return params.first*inverted_coef_;
}

int  MotorKangaroo::GetCurEncDeg() const
{
	return GetCurEnc() / counts_per_deg_;
}

int  MotorKangaroo::GetCurEncRot() const
{
	return GetCurEnc() / counts_per_deg_ / 360;
}

bool  MotorKangaroo::IsReady()
{
	auto params = kangaroo_drv_->CmdGet(chnl_, KangarooDriver::kGetPos);
	return (params.second & 2) == 0;
}

void MotorKangaroo::ResetEnc()
{
	kangaroo_drv_->CmdHome(chnl_);
}

void MotorKangaroo::Stop()
{
	kangaroo_drv_->CmdMoveToSpeed(chnl_, 0);
}

void  MotorKangaroo::MoveContinue(int speed)
{
	speed *= inverted_coef_*counts_per_deg_;
	speed = speed > 1500 ? 1500 : speed;
	speed = speed < -1500 ? -1500 : speed;
	kangaroo_drv_->CmdMoveToSpeed(chnl_, speed);
}

void MotorKangaroo::MoveIncEncCounts(int speed, int counts, bool wait)
{
	speed *= inverted_coef_*counts_per_deg_;
	if (speed != 0) {
		kangaroo_drv_->CmdMoveIncPos(chnl_, speed > 0 ? counts : -counts, speed > 0 ? speed : -speed);
	}
	while (!IsReady() && wait) ;
}

void MotorKangaroo::MoveIncDeg(int speed, int deg, bool wait)
{
	MoveIncEncCounts(speed, deg*counts_per_deg_, wait);
}

void MotorKangaroo::MoveIncRot(int speed, int rot, bool wait)
{
	MoveIncDeg(speed, rot * 360, wait);
}

void MotorKangaroo::MoveToEncCounts(int speed, int counts, bool wait)
{
	speed *= inverted_coef_*counts_per_deg_;
	if (speed != 0) {
		kangaroo_drv_->CmdMoveToPos(chnl_, speed > 0 ? counts : -counts, speed > 0 ? speed : -speed);
	}
	while (!IsReady() && wait) ;
}

void MotorKangaroo::MoveToDeg(int speed, int deg, bool wait)
{
	MoveToEncCounts(speed, deg*counts_per_deg_, wait);
}

void MotorKangaroo::MoveToRot(int speed, int rot, bool wait)
{
	MoveToDeg(speed, rot * 360, wait);
}

