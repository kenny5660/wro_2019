#pragma once

class Motor {
public:
	//@brief Getting Current Encoder value
	virtual int	GetCurEnc() const = 0;
	
	//@brief Getting Current Encoder value in degrees
	virtual int GetCurEncDeg() const  = 0;
	
	//@brief Getting Current Encoder value in rotations
	virtual int GetCurEncRot() const  = 0;
	
	//@brief Stop motor
	virtual void Stop()  = 0;
	
	//@brief Move motor without stop
	//@param speed measured in deg/sec
    virtual void MoveContinue(int speed) = 0;
	
	// Move motor and wait 'msec' milliseconds
	//@brief Move motor and wait 'msec' milliseconds
	//@param speed measured in deg/sec
	//@param msec measured in milliseconds
	virtual void MoveTime(int speed, int msec) = 0;
	
	//@brief Move motor from 'CurEnc' to 'CurEnc'+'counts' (increment)
	//@param speed measured in deg/sec
	//@param counts measured in EncCounts
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveIncEncCounts(int speed, int counts, bool wait = false) = 0;
	
	//@brief Move motor  from GetCurEncDeg to GetCurEncDeg+'deg' (increment)
	//@param speed measured in deg/sec
	//@param deg measured in degrees
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveIncDeg(int speed, int deg, bool wait = false) = 0;
	
	//@brief Move motor from GetCurEncRot to GetCurEncRot+'rots' (increment)
	//@param speed measured in deg/sec
	//@param rots measured in rotations
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveIncRot(int speed, int rots, bool wait = false) = 0;
	
	//@brief Move motor from 'CurEnc' to 'counts' (set)
	//@param speed measured in deg/sec
	//@param counts measured in EncCounts
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveToEncCounts(int speed, int counts, bool wait = false) = 0;
	
	//@brief Move motor from 'CurEnc' to 'deg' (set)
	//@param speed measured in deg/sec
	//@param deg measured in degrees
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveToDeg(int speed, int deg, bool wait = false) = 0;
	
	//@brief Move motor from 'CurEnc' to 'rots' (set)
	//@param speed measured in deg/sec
	//@param rots measured in degrees
	//@param wait when is true, function wait flag 'IsReady'
	virtual void MoveToRot(int speed, int rots, bool wait = false) = 0;
	
	//@brief Return true if motor is ready for new command, false if motor is moving
	virtual bool IsReady() = 0;
	
	//@brief Set encoder to 0
	virtual void ResetEnc() = 0;
	virtual ~Motor() {}
};

#ifdef HARDWERE_MODE
#include "MotorKangaroo.h"
#else
#include "RobotModel.h"
#endif