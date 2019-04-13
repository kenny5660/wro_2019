#include "Motor.h"
KangarooMotor::KangarooMotor(std::shared_ptr<KangarooDriver> kangarooDrv, 
	uint8_t chnl,
	int counts_per_deg)
	: kangaroo_drv_(kangarooDrv)
	, chnl_(chnl)
	, counts_per_deg_(counts_per_deg) {}

int KangarooMotor::GetCurEnc() const
{
	auto params =	kangaroo_drv_->CmdGet(chnl_, KangarooDriver::kGetPos);
	return params.first;
}

int  KangarooMotor::GetCurEncDeg() const
{
	return GetCurEnc() / counts_per_deg_;
}

int  KangarooMotor::GetCurEncRot() const
{
	return GetCurEnc() / counts_per_deg_ / 360;
}

bool  KangarooMotor::IsReady()
{
	auto params = kangaroo_drv_->CmdGet(chnl_, KangarooDriver::kGetPos);
	return (params.second & 2) == 0;
}

void KangarooMotor::ResetEnc()
{
	kangaroo_drv_->CmdHome(chnl_);
}

void KangarooMotor::Stop()
{
	kangaroo_drv_->CmdMoveToSpeed(chnl_, 0);
}

void  KangarooMotor::MoveContinue(int speed)
{
	kangaroo_drv_->CmdMoveToSpeed(chnl_, speed);
}

void KangarooMotor::MoveIncEncCounts(int speed, int counts, bool wait)
{
	kangaroo_drv_->CmdMoveIncPos(chnl_, counts, speed);
	while (!IsReady() && wait) ;
}

void KangarooMotor::MoveIncDeg(int speed, int deg, bool wait)
{
	MoveIncEncCounts(speed, deg*counts_per_deg_, wait);
}

void KangarooMotor::MoveIncRot(int speed, int rot, bool wait)
{
	MoveIncDeg(speed, rot * 360, wait);
}

void KangarooMotor::MoveToEncCounts(int speed, int counts, bool wait)
{
	kangaroo_drv_->CmdMoveToPos(chnl_, counts, speed);
	while (!IsReady() && wait) ;
}

void KangarooMotor::MoveToDeg(int speed, int deg, bool wait)
{
	MoveToEncCounts(speed, deg*counts_per_deg_, wait);
}

void KangarooMotor::MoveToRot(int speed, int rot, bool wait)
{
	MoveToDeg(speed, rot * 360, wait);
}

