#pragma once

#include <memory>
#include "Uart.h"
class Servo
{
public:
	virtual void Enable() = 0;
	virtual void Disable() = 0;
	virtual void SetDegrees(double deg,bool wait = false, uint16_t time = 0) = 0;
	virtual double GetDegrees() = 0;
	virtual int GetLead()  = 0;
	virtual bool IsLead() = 0;

};

class ServoOcsBase : public Servo
{
public:	
	ServoOcsBase(uint8_t id, std::shared_ptr<Uart> uart, double offset_deg  = 0);
	virtual void Enable();
	virtual void Disable();
	virtual int GetLead();
	virtual bool IsLead();
protected:
	//@brief Send ping packet
//@return servo id 
int Ping();
	void WaitSet(bool isWhait, double deg, uint16_t time, uint8_t data[4]);
	int ReadData(uint8_t addr, uint8_t *data, size_t size);
	void WriteData(uint8_t addr, uint8_t* data, size_t size);
	std::shared_ptr<Uart> uart_;
	uint8_t id_;
	double offset_deg_;
 
	
	const double SERVO_D_LEAD_MID = 100; 
	const uint8_t SERVO_D_INSTRUCTION_PING = 0x01;
	const uint8_t SERVO_D_INSTRUCTION_READ = 0x02;
	const uint8_t SERVO_D_INSTRUCTION_WRITE = 0x03;
	const uint8_t SERVO_D_INSTRUCTION_REG_WRITE = 0x04;
	const uint8_t SERVO_D_INSTRUCTION_ACTION = 0x05;
	const uint8_t SERVO_D_INSTRUCTION_SYNC_WRITE = 0x83;
	const uint8_t SERVO_D_INSTRUCTION_RESET = 0x06;
	const uint8_t SERVO_D_PACKET_ID = 2;
	const uint8_t SERVO_D_PACKET_LENGTH = 3;
	const uint8_t SERVO_D_PACKET_INSTRUCTION = 4;
	const uint8_t SERVO_D_PACKET_PARAMS = 5;
	const uint8_t SERVO_D_PACKET_STATE = 4;
	const uint8_t SERVO_D_BROADCAST_ID = 0xFE;
	const uint8_t SERVO_D_ADDR_TORQUE_SWITCH = 0x28;
	const uint8_t SERVO_D_ADDR_GOAL_POSITION = 0x2A;  //high first
	const uint8_t SERVO_D_ADDR_OPERATION_SPEED = 0x2E;  //low first
	const uint8_t SERVO_D_ADDR_OPERATION_TIME = 0x2C;  //low first
	const uint8_t SERVO_D_ADDR_CURRENT_LEAD = 0x3C;    // low first
	const uint8_t SERVO_D_ADDR_CURRENT_TEMP = 0x3F;
	const uint8_t SERVO_D_ADDR_CURRENT_POSITION = 0x38; 
};

class ServoOcs251 : public ServoOcsBase
{
public:
	ServoOcs251(uint8_t id, std::shared_ptr<Uart> uart, double offset_deg  = 0);
	void SetDegrees(double deg, bool wait = false, uint16_t time = 0) override;
	double GetDegrees()override;
private:
	const double SERVO_D_251_DEGREE_COEF = 0.3519061583577712609970674486803;
	const int kMaxServoDeg = 1023;  
	const int kMinServoDeg = 0; 
};

class ServoOcs301 : public ServoOcsBase
{
public:
	ServoOcs301(uint8_t id, std::shared_ptr<Uart> uart, double offset_deg  = 0);
	void SetDegrees(double deg, bool wait = false, uint16_t time = 0) override;
	double GetDegrees()override;

private:
	const double SERVO_D_301_DEGREE_COEF = 0.0879120879120879;
	const int kMaxServoDeg = 4095;  
	const int kMinServoDeg = 0; 
};