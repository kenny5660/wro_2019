#include "Kangaroo.h"
#include <string>
void KangarooDriver::CmdMoveToPos(uint8_t chnl, int pos, int limit_speed)
{
	CmdMovePos(chnl, kMoveTypePos, pos, limit_speed);
}
void KangarooDriver::CmdMoveIncPos(uint8_t chnl, int pos, int limit_speed)
{
	CmdMovePos(chnl, kMoveTypeIncPos, pos, limit_speed);
}

void KangarooDriver::CmdMoveToSpeed(uint8_t chnl, int speed)
{
	CmdMoveSpeed(chnl, kMoveTypeSpeed, speed);
}
void KangarooDriver::CmdMoveIncSpeed(uint8_t chnl, int speed)
{
	CmdMoveSpeed(chnl, kMoveTypeIncSpeed, speed);
}

uint16_t KangarooDriver::crc14(const uint8_t* data, std::size_t length)
{
	uint16_t crc = 0x3fff; std::size_t i, bit;
	for (i = 0; i < length; i++) {
		crc ^= data[i] & 0x7f;
		for (bit = 0; bit < 7; bit++) {
			if (crc & 1) { crc >>= 1; crc ^= 0x22f0; }
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc ^ 0x3fff;
}

std::size_t KangarooDriver::bitpackNumber(uint8_t* buffer, int32_t number)
{
	std::size_t i = 0;
	if (number < 0) { number = -number; number <<= 1; number |= 1; }
	else { number <<= 1; }
	while (i < 5) {
		buffer[i++] = (number & 0x3f) | (number >= 0x40 ? 0x40 : 0x00); 
		number >>= 6; 
		if (!number) { break; }
	} 
	return i;
}

void KangarooDriver::CmdStart(uint8_t chnl)
{
	//Channel Name (1 byte)
		//Flags(1 byte) 
		//	0 for no options.
		//	64 if a sequence code is used.
		//Sequence Code(optional, 1 byte)
    uint8_t data_packet[7];
	data_packet[0] = addr_;
	data_packet[1] = kCmdStart;
	data_packet[2] = 2; //length
	data_packet[3] = chnl;
	data_packet[4] = 0;
	uint16_t crc = crc14(data_packet, 5);
	data_packet[5] = crc & 0x7F;
	data_packet[6] = crc >> 7 & 0x7F;
	uart_->Send(data_packet, 7);	
}

void KangarooDriver::CmdHome(uint8_t chnl)
{
	//Channel Name (1 byte)
	//Flags(1 byte) 
	//	0 for no options.
	//	64 if a sequence code is used.
	//Sequence Code(optional, 1 byte)
	uint8_t data_packet[7];
	data_packet[0] = addr_;
	data_packet[1] = kCmdHome;
	data_packet[2] = 2; //length
	data_packet[3] = chnl;
	data_packet[4] = 0;
	uint16_t crc = crc14(data_packet, 5);
	data_packet[5] = crc & 0x7F;
	data_packet[6] = crc >> 7 & 0x7F;
	uart_->Send(data_packet, 7);
}
void  KangarooDriver::CmdMoveSpeed(uint8_t chnl, uint8_t type, int speed)
{
	//MOVE
	//Channel Name(1 byte)
	//Flags(1 byte) 0 for no options.
	//8 to not apply the speed limit and speed ramping source settings.By default, the speed limit comes from Kangaroo's potentiometers.
	//32 to use raw units.For analog, raw units are millivolts.For quadrature, 4 raw units equal 1 line. 64 if a sequence code is used.Add these to combine options.
	//Sequence Code(optional, 1 byte) Motion Parameter(s)
	//Use one or multiple motion parameters.
	//Type(1 byte) 1 for Position.
	//2 for Speed, if no Position parameter is included.If a Position parameter is included, it becomes a Speed Limit.
	//3 for Speed Ramping.This does not affect Position Control.Add 64 to make the command incremental(relative to the current position and speed). 10
	//Channel Name (1 byte)
	//Flags(1 byte) 
	//	0 for no options.
	//	64 if a sequence code is used.
	//Sequence Code(optional, 1 byte)
	uint8_t data_packet[13];
	uint8_t bitpack_Value[5];
	data_packet[0] = addr_;
	data_packet[1] = kCmdMove;
	data_packet[3] = chnl;
	data_packet[4] = 0; //flags
	data_packet[5] = type;   //type
	size_t lengthValue = bitpackNumber(bitpack_Value, speed);
	for (int i = 0; i < lengthValue; ++i) {
		data_packet[6 + i] = bitpack_Value[i];
	}
	data_packet[2] = 3 + lengthValue; //length
	uint16_t crc = crc14(data_packet, 6 + lengthValue);
	data_packet[6 + lengthValue] = crc & 0x7F;
	data_packet[7 + lengthValue] = crc >> 7 & 0x7F;
	uart_->Send(data_packet, 8 + lengthValue);
}


void KangarooDriver::CmdMovePos(uint8_t chnl, uint8_t type, int pos, int limit_speed)
{
	//MOVE
	//Channel Name(1 byte)
	//Flags(1 byte) 0 for no options.
	//8 to not apply the speed limit and speed ramping source settings.By default, the speed limit comes from Kangaroo's potentiometers.
	//32 to use raw units.For analog, raw units are millivolts.For quadrature, 4 raw units equal 1 line. 64 if a sequence code is used.Add these to combine options.
	//Sequence Code(optional, 1 byte) Motion Parameter(s)
	//Use one or multiple motion parameters.
	//Type(1 byte) 1 for Position.
	//2 for Speed, if no Position parameter is included.If a Position parameter is included, it becomes a Speed Limit.
	//3 for Speed Ramping.This does not affect Position Control.Add 64 to make the command incremental(relative to the current position and speed). 10
	//Channel Name (1 byte)
	//Flags(1 byte) 
	//	0 for no options.
	//	64 if a sequence code is used.
	//Sequence Code(optional, 1 byte)
	uint8_t data_packet[20];
	uint8_t bitpack_Value[5];
	data_packet[0] = addr_;
	data_packet[1] = kCmdMove;
	data_packet[3] = chnl;
	data_packet[4] = 0; //flags
	data_packet[5] = type; //type
	size_t lengthValue = bitpackNumber(bitpack_Value, pos);
	for (int i = 0; i < lengthValue; ++i) {
		data_packet[6 + i] = bitpack_Value[i];
	}
	size_t lengthSpeed = 0;
	if (limit_speed > 0) {
		data_packet[6 + lengthValue] = kMoveTypeSpeed; //type
		lengthSpeed = bitpackNumber(bitpack_Value, limit_speed);
		for (int i = 0; i < lengthSpeed; ++i) {
			data_packet[7 + lengthValue + i] = bitpack_Value[i];
		}
		lengthSpeed++;
	}
	data_packet[2] = 3 + lengthValue + lengthSpeed; //length
	uint16_t crc = crc14(data_packet, 6 + lengthValue + lengthSpeed);
	data_packet[6 + lengthValue + lengthSpeed] = crc & 0x7F;
	data_packet[7 + lengthValue + lengthSpeed] = crc >> 7 & 0x7F;
	uart_->Send(data_packet, 8 + lengthValue + lengthSpeed);
}

std::pair<int, uint8_t> KangarooDriver::CmdGet(uint8_t chnl, uint8_t type)
{
	//Channel Name(1 byte)
	//	Flags(1 byte) 0 for no options.
	//	16 if an echo code is used.It will be sent with the reply.
	//	32 to use raw units.For analog, raw units are millivolts.For quadrature, 4 raw units equal 1 line. 64 for a sequence code to be sent with the reply.Add these to combine options.
	//	Echo Code(optional, 1 byte)
	//	Parameter(1 byte) 1 to get the current position. 2 to get the current speed.
	//	Add 64 to get an incremental position or speed(relative to the last Move you commanded).
	//	8 to get the minimum position. 9 to get the maximum position.
	uint8_t flags;
	uint8_t data_packet[8];
	data_packet[0] = addr_;
	data_packet[1] = kCmdGet;
	data_packet[2] = 3;
	data_packet[3] = chnl;
	data_packet[4] = 0; //flags
	data_packet[5] = type; //type

	uint16_t crc = crc14(data_packet, 6);
	data_packet[6] = crc & 0x7F;
	data_packet[7] = crc >> 7 & 0x7F;
	uart_->Clear();
	uart_->Send(data_packet, 8);


	uint8_t data_packet_reply[15];
	uint8_t packet_header[3];
	uint8_t data_header[3];
	uint8_t bitpack_Value[5];  // Зачем это?
	uint8_t crc14[2];
	int read_status = uart_->Get(packet_header, 3) ;
	if (packet_header[0] != addr_ || packet_header[1] != kCmdGetReply || read_status < VI_SUCCESS)
	{
		throw std::runtime_error(std::string("Read error, kangaroo driver! id  = ")+std::to_string(addr_));
	}
	if (packet_header[2] > 8) {
		flags = -1;
		throw std::runtime_error(std::string("Read error, kangaroo driver! id  = ") + std::to_string(addr_));
		return std::make_pair(0,flags);
	}
	uart_->Get(data_header, 3);
	uart_->Get(bitpack_Value, packet_header[2] - 3);

	uart_->Get(crc14, 2);
	flags = data_header[1];
	int value = 0;
	int sign = 1;
	if ((bitpack_Value[0] & 0x1) > 0) {
		sign = -1;
	}
	for (int i = 0; i < packet_header[2] - 3; ++i) {
		value |= (int)((bitpack_Value[i] & 0x7E) >> 1) << (i * 6);
	}
	value *= sign;
	return std::make_pair(value, flags); 
}

KangarooDriver::KangarooDriver(std::shared_ptr<Uart> _uart, uint8_t _addr)
	: uart_(_uart)
	, addr_(_addr)
{
	CmdGet('1', kGetPos);
}
