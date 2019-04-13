#pragma once
#include <cstdint>
#include <memory>
#include <utility>
#include "UART.h"
#include "MyRio.h"

class KangarooDriver {
public:
	KangarooDriver(std::shared_ptr<MyRio_Uart> _uart, uint8_t _addr);
	//Enable Channel number 'chnl'
	void CmdStart(uint8_t chnl);
	
	//Send cmd home Channel number 'chnl'
	void CmdHome(uint8_t chnl);
	
	//Move Channel number 'chnl' to 'pos' with speed 'limit_speed'
	void CmdMoveToPos(uint8_t chnl, int pos, int limit_speed);
	
	//Move Channel number 'chnl' increment 'pos' with speed 'limit_speed'
	void CmdMoveIncPos(uint8_t chnl, int pos, int limit_speed);
	
	//Move Channel number 'chnl' with speed 'speed'
	void CmdMoveToSpeed(uint8_t chnl, int speed);
	
	//Move Channel number 'chnl' with speed 'speed'
	void CmdMoveIncSpeed(uint8_t chnl, int speed);
	
	//gets parameter selected by 'type'
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
	std::shared_ptr<MyRio_Uart> uart_;
	uint8_t addr_;
	//! Computes a 14-bit CRC. 
	// \param data The buffer to compute the CRC of.  
    // \param length The length of the data. 
	// \return The CRC. 
	uint16_t crc14(const uint8_t* data, std::size_t length);
	//! Bit-packs a number
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