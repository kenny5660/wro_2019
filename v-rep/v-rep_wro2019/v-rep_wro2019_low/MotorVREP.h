#pragma once
#include <memory>
#include "b0RemoteApi.h"
class MotorVREP : public Motor {
 public:
  MotorVREP(std::shared_ptr<b0RemoteApi> cl, int handle, bool inverted = false,
            int counts_per_deg = 1);
  //@brief Getting Current Encoder value
  virtual double GetCurEnc() const;

  //@brief Getting Current Encoder value in degrees
  virtual double GetCurEncDeg() const;

  //@brief Getting Current Encoder value in rotations
  virtual double GetCurEncRot() const;

  //@brief Stop motor
  virtual void Stop();

  //@brief Move motor without stop
  //@param speed measured in deg/sec
  virtual void MoveContinue(double speed);

  // Move motor and wait 'msec' milliseconds
  //@brief Move motor and wait 'msec' milliseconds
  //@param speed measured in deg/sec
  //@param msec measured in milliseconds
  virtual void MoveTime(double speed, int msec);

  //@brief Move motor from 'CurEnc' to 'CurEnc'+'counts' (increment)
  //@param speed measured in deg/sec
  //@param counts measured in EncCounts
  //@param wait when is true, function wait flag 'IsReady'
  virtual void MoveIncEncCounts(double speed, double counts, bool wait = false);

  //@brief Move motor  from GetCurEncDeg to GetCurEncDeg+'deg' (increment)
  //@param speed measured in deg/sec
  //@param deg measured in degrees
  //@param wait when is true, function wait flag 'IsReady'
  virtual void MoveIncDeg(double speed, double deg, bool wait = false);

  //@brief Move motor from GetCurEncRot to GetCurEncRot+'rots' (increment)
  //@param speed measured in deg/sec
  //@param rots measured in rotations
  //@param wait when is true, function wait flag 'IsReady'
  virtual void MoveIncRot(double speed, double rots, bool wait = false);

  //@brief Move motor from 'CurEnc' to 'counts' (set)
  //@param speed measured in deg/sec
  //@param counts measured in EncCounts
  //@param wait when is true, function wait flag 'IsReady'
  virtual void MoveToEncCounts(double speed, double counts, bool wait = false);

  //@brief Move motor from 'CurEnc' to 'deg' (set)
  //@param speed measured in deg/sec
  //@param deg measured in degrees
  //@param wait when is true, function wait flag 'IsReady'
  virtual void MoveToDeg(double speed, double deg, bool wait = false);

  //@brief Move motor from 'CurEnc' to 'rots' (set)
  //@param speed measured in deg/sec
  //@param rots measured in degrees
  //@param wait when is true, function wait flag 'IsReady'
  virtual void MoveToRot(double speed, double rots, bool wait = false);

  //@brief Return true if motor is ready for new command, false if motor is
  // moving
  virtual bool IsReady();

  //@brief Set encoder to 0
  virtual void ResetEnc();

 private:
  double target_pos_ = 0;
  double real_pos_ = 0;
  int handle_;
  std::shared_ptr<b0RemoteApi> cl_;
  int counts_per_deg_;
  int inverted_coef_ = 1;

  const int kJointintparam_ctrl_enabled = 2001;
  const int kJointfloatparam_upper_limit = 2017;
};