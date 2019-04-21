#pragma once
#include "Kangaroo.h"
class Motor {
public:
	//@brief Getting Current Encoder value
	virtual int	GetCurEnc() const {throw std::runtime_error("Not implemented");}
	
	//@brief Getting Current Encoder value in degrees
	virtual int GetCurEncDeg() const {throw std::runtime_error("Not implemented");}
	
	//@brief Getting Current Encoder value in rotations
	virtual int GetCurEncRot() const {throw std::runtime_error("Not implemented");}
	
	//@brief Stop motor
	virtual void Stop() {throw std::runtime_error("Not implemented");}
	
	//@brief Move motor without stop
	//@param speed measured in deg/sec
    virtual void MoveContinue(int speed) {throw std::runtime_error("Not implemented");}
	
	// Move motor and wait 'msec' milliseconds
	//@brief Move motor and wait 'msec' milliseconds
	//@param speed measured in deg/sec
	//@param msec measured in milliseconds
	virtual void MoveTime(int speed, int msec) {throw std::runtime_error("Not implemented");}
	
	//@brief Move motor from 'CurEnc' to 'CurEnc'+'counts' (increment)
	//@param speed measured in deg/sec
	//@param counts measured in EncCounts
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveIncEncCounts(int speed, int counts, bool wait = false) {throw std::runtime_error("Not implemented");}
	
	//@brief Move motor  from GetCurEncDeg to GetCurEncDeg+'deg' (increment)
	//@param speed measured in deg/sec
	//@param deg measured in degrees
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveIncDeg(int speed, int deg, bool wait = false) {throw std::runtime_error("Not implemented");}
	
	//@brief Move motor from GetCurEncRot to GetCurEncRot+'rots' (increment)
	//@param speed measured in deg/sec
	//@param rots measured in rotations
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveIncRot(int speed, int rots, bool wait = false){throw std::runtime_error("Not implemented");}
	
	//@brief Move motor from 'CurEnc' to 'counts' (set)
	//@param speed measured in deg/sec
	//@param counts measured in EncCounts
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveToEncCounts(int speed, int counts, bool wait = false){throw std::runtime_error("Not implemented");}
	
	//@brief Move motor from 'CurEnc' to 'deg' (set)
	//@param speed measured in deg/sec
	//@param deg measured in degrees
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveToDeg(int speed, int deg, bool wait = false){throw std::runtime_error("Not implemented");}
	
	//@brief Move motor from 'CurEnc' to 'rots' (set)
	//@param speed measured in deg/sec
	//@param rots measured in degrees
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveToRot(int speed, int rots, bool wait = false){throw std::runtime_error("Not implemented");}
	
	//@brief Return true if motor is ready for new command, false if motor is moving
	virtual bool IsReady(){throw std::runtime_error("Not implemented");} 
	
	//@brief Set encoder to 0
	virtual void ResetEnc(){throw std::runtime_error("Not implemented");}
	virtual ~Motor() {}
};

class KangarooMotor : public Motor {
public:
	KangarooMotor(std::shared_ptr<KangarooDriver> kangarooDrv, uint8_t chnl, bool inverted = false,int counts_per_deg = 1);
	//Getting Current Encoder value
	 int GetCurEnc() const override;
	
	//Getting Current Encoder value in degrees
	 int GetCurEncDeg() const override;
	
	//Getting Current Encoder value in rotations
	 int GetCurEncRot() const override;
	
	//Stop motor
	 void Stop() override;
	
	// Move motor without stop
     void MoveContinue(int speed) override;
	
	// Move motor and wait 'msec' milliseconds
	 void MoveTime(int speed, int msec) override;
	
	 void MoveIncEncCounts(int speed, int counts, bool wait = false) override;
	
	// Move motor  from GetCurEncDeg to GetCurEncDeg+'deg' (increment), when 'wait' is true, function wait flag 'IsReady'
	 void MoveIncDeg(int speed, int deg, bool wait = false) override;
	
	// Move motor from GetCurEncRot to GetCurEncRot+'rots' (increment), when 'wait' is true, function wait flag 'IsReady'
	 void MoveIncRot(int speed, int rots, bool wait = false) override;
	
	// Move motor from 'CurEnc' to 'counts' (set), when 'wait' is true, function wait flag 'IsReady'
	 void MoveToEncCounts(int speed, int counts, bool wait = false) override;
	
	// Move motor from 'CurEnc' to 'deg' (set), when 'wait' is true, function wait flag 'IsReady'
	 void MoveToDeg(int speed, int deg, bool wait = false) override;
	
	// Move motor from 'CurEnc' to 'rots' (set), when 'wait' is true, function wait flag 'IsReady'
	 void MoveToRot(int speed, int rots, bool wait = false) override;
	
	//Return true if motor is ready for new command, false if motor is moving
	 bool IsReady() override;
	
	//Set encoder to 0
	 void ResetEnc() override;
 private:
	std::shared_ptr<KangarooDriver> kangaroo_drv_;
	uint8_t chnl_;
	int counts_per_deg_;
	int inverted_coef_ = 1;
};