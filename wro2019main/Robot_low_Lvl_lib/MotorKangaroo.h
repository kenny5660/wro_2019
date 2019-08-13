#pragma once
#include "Kangaroo.h"
class MotorKangaroo : public Motor {
public:
	MotorKangaroo(std::shared_ptr<KangarooDriver> kangarooDrv, uint8_t chnl, bool inverted = false, double counts_per_deg = 1);
	//Getting Current Encoder value
	 double GetCurEnc() const override;
	
	//Getting Current Encoder value in degrees
	 double GetCurEncDeg() const override;
	
	//Getting Current Encoder value in rotations
	 double GetCurEncRot() const override;
	
	//Stop motor
	 void Stop() override;
	
	// Move motor without stop
     void MoveContinue(double speed) override;
	
	// Move motor and wait 'msec' milliseconds
	 void MoveTime(double speed, int msec) override;
	
	void MoveIncEncCounts(double speed, double counts, bool wait = false) override;
	
	// Move motor  from GetCurEncDeg to GetCurEncDeg+'deg' (increment), when 'wait' is true, function wait flag 'IsReady'
	 void MoveIncDeg(double speed, double deg, bool wait = false) override;
	
	// Move motor from GetCurEncRot to GetCurEncRot+'rots' (increment), when 'wait' is true, function wait flag 'IsReady'
	 void MoveIncRot(double speed, double rots, bool wait = false) override;
	
	// Move motor from 'CurEnc' to 'counts' (set), when 'wait' is true, function wait flag 'IsReady'
	 void MoveToEncCounts(double speed, double counts, bool wait = false) override;
	
	// Move motor from 'CurEnc' to 'deg' (set), when 'wait' is true, function wait flag 'IsReady'
	 void MoveToDeg(double speed, double deg, bool wait = false) override;
	
	// Move motor from 'CurEnc' to 'rots' (set), when 'wait' is true, function wait flag 'IsReady'
	 void MoveToRot(double speed, double rots, bool wait = false) override;
	
	//Return true if motor is ready for new command, false if motor is moving
	 bool IsReady() override;
	
	//Set encoder to 0
	 void ResetEnc() override;
private:
	std::shared_ptr<KangarooDriver> kangaroo_drv_;
	uint8_t chnl_;
	double counts_per_deg_;
	int inverted_coef_ = 1;
};