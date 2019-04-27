#pragma once
#include <cstdint>
#include <memory>
#include <utility>
#include "Uart.h"
#include "MyRio.h"

class KangarooDriver {
public:
	KangarooDriver(std::shared_ptr<Uart> _uart, uint8_t _addr);
	// @brief Enable Channel  
	// @param chnl Channel number   
	void CmdStart(uint8_t chnl);
	
	// @brief Send cmd home 
	// @param chnl Channel number 
	void CmdHome(uint8_t chnl);

	// @brief Move to 'pos' with speed 'limit_speed'
	// @param chnl Channel number 
	// @param pos the position in which the motor should come, measured in Encoder Counts / 4
	// @param limit_speed maximum speed
	void CmdMoveToPos(uint8_t chnl, int pos, int limit_speed);
	
	// @brief Move increment 'pos' with speed 'limit_speed
	// @param chnl Channel number 
	// @param pos increment value, measured in Encoder Counts / 4
	// @param limit_speed maximum speed
	void CmdMoveIncPos(uint8_t chnl, int pos, int limit_speed);
	
	//Move Channel number 'chnl' with speed 'speed'
	// @brief Move with speed 'speed'
	// @param chnl Channel number 
	// @param speed measured in (Encoder Counts / 4)/sec
	void CmdMoveToSpeed(uint8_t chnl, int speed);
	
	//Move Channel number 'chnl' with speed 'speed'
	// @brief Move with Currnet speed + 'speed' (increment)
	// @param chnl Channel number 
	// @param speed measured in (Encoder Counts / 4)/sec
	void CmdMoveIncSpeed(uint8_t chnl, int speed);
	
	//Move Channel number 'chnl' with speed 'speed'
	// @brief Move with Currnet speed + 'speed' (increment)
	// @param chnl Channel number 
	// @param speed measured in (Encoder Counts / 4)/sec
	std::pair<int, uint8_t> CmdGet(uint8_t chnl, uint8_t type);
	
	static	const uint8_t kGetPos = 1;
	static	const uint8_t kGetSpeed = 2;
	static	const uint8_t kGetIncPos = 65;
	static	const uint8_t kGetIncSpeed = 66;
	static	const uint8_t kGetMinPos = 8;
	static	const uint8_t kGetMaxPos = 9;
	
private:
	void CmdMoveSpeed(uint8_t chnl, uint8_t type, int speed);
	void CmdMovePos(uint8_t chnl, uint8_t type, int pos, int limit_speed);
	std::shared_ptr<Uart> uart_;
	uint8_t addr_;
	// @brief Computes a 14-bit CRC. 
	// @param data The buffer to compute the CRC of.  
    // @param length The length of the data. 
	// @return The CRC. 
	uint16_t crc14(const uint8_t* data, std::size_t length);
	//  @brief Bit-packs a number
	//	@param buffer The buffer to write into.
	//	@param number The number to bit-pack. Should be between -(2^29-1) and 2^29-1. 
	//  @return How many bytes were written (1 to 5). 
	std::size_t bitpackNumber(uint8_t* buffer, int32_t number);
	
	static	const uint8_t kCmdStart = 32;
	static	const uint8_t kCmdHome = 34;
	static	const uint8_t kCmdMove = 36;
	static	const uint8_t kCmdGet = 35;
	static const uint8_t kCmdGetReply = 67;

	static const uint8_t kMoveTypePos = 1;
	static	const uint8_t kMoveTypeSpeed = 2;
	static	const uint8_t kMoveTypeIncPos = 65;
	static const uint8_t kMoveTypeIncSpeed = 66;

	
};