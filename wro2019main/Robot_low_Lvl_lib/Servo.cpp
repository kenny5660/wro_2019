#include "Servo.h"
#include  <string>
#include <cmath>
void Servo_ocs251::SetDegrees(double deg,bool wait, uint16_t time)
{
	uint8_t data[4];
	int servo_deg = (deg + offset_deg_) / SERVO_D_251_DEGREE_COEF;
	if (servo_deg > kMaxServoDeg)
	{
		throw std::runtime_error("Error, servo angle more than it can!");
	}
	if (servo_deg < kMinServoDeg)
	{
		throw std::runtime_error("Error, servo angle less than it can!");
	}
	data[0] = (servo_deg >> 8) & 0xFF;
	data[1] = servo_deg & 0xFF;

	data[2] = time & 0xFF;
	data[3] = (time >> 8) & 0xFF;
	WriteData(SERVO_D_ADDR_GOAL_POSITION, data, 4);
	if (wait)
	{
		while (1)
		{
			int a = abs(deg - GetDegrees());
			if (a < 4)
			{
				break;
			}
		}
	
	}
	//Delay_servo(100);	
}

int Servo_ocs251::GetDegrees()
{
	uint8_t data[2];
	ReadData(SERVO_D_ADDR_CURRENT_POSITION, data, 2);
	int deg = (int)data[0] << 8;
	deg |= (int)data[1];
	deg *= SERVO_D_251_DEGREE_COEF;
	return deg;
}

int Servo_ocs251::GetLead()
{
	uint8_t data[2];
	ReadData(SERVO_D_ADDR_CURRENT_LEAD, data, 2);
	int32_t lead = data[0];
	lead <<= 8;
	lead |= data[1];
	return lead;
}


bool Servo_ocs251::IsLead()
{
	int lead = GetLead(); 
	//std::cout << "Servo current lead: " << lead << std::endl;                  
	return lead > SERVO_D_LEAD_MID;
}

void Servo_ocs251::Disable()
{
	uint8_t data = 0;
	WriteData(SERVO_D_ADDR_TORQUE_SWITCH, &data, 1);
}


void Servo_ocs251::Enable()
{
	uint8_t data = 1;
	WriteData(SERVO_D_ADDR_TORQUE_SWITCH, &data, 1);
}



int Servo_ocs251::ReadData(uint8_t addr, uint8_t *data, size_t size)
{
	uint8_t check_sum = id_ + SERVO_D_INSTRUCTION_READ + addr + size + 4;
	check_sum = ~check_sum;
	uint8_t data_packet[8] = {
		0xFF,
		0xFF,
		id_,
		4,
		SERVO_D_INSTRUCTION_READ,
		addr,
		(uint8_t)size,
		check_sum 
	};

	uart_->Send(data_packet, 8);
	//uint8_t* data_packet_return = (uint8_t*)malloc(6 + size);
	 uint8_t data_packet_return[10];

	//uart_->Clear();
	int status  = uart_->Get(data_packet_return, 6 + size);
	if (data_packet_return[2] != id_ || data_packet_return[0] != 0xFF || data_packet_return[1] != 0xFF ||  uart_->isError()) 
	{
		throw std::runtime_error(std::string("Read error, Servo! id  = ") + std::to_string(id_));
	}
	uint8_t check_sum_return = data_packet_return[SERVO_D_PACKET_ID] +
	                           data_packet_return[SERVO_D_PACKET_LENGTH];
	for (int i = 0; i < size; ++i) {
		data[i] = data_packet_return[SERVO_D_PACKET_PARAMS + i];
		check_sum_return += data_packet_return[SERVO_D_PACKET_PARAMS + i];
	}
	check_sum_return = ~check_sum_return;
	if (check_sum_return ==
	    data_packet_return[3 + data_packet_return[SERVO_D_PACKET_LENGTH]]) {
		size_t data_length = data_packet_return[SERVO_D_PACKET_LENGTH] - 2;
		//free(data_packet_return);
		return data_length;
	}
	else {
	//free(data_packet_return);
		throw std::runtime_error(std::string("Read, Cheksum error, Servo! id  = ") + std::to_string(id_));
		return 0;
	}
}
void Servo_ocs251::WriteData(uint8_t addr, uint8_t* data, size_t size)
{
	uint8_t check_sum = id_ + 3 + size + SERVO_D_INSTRUCTION_WRITE + addr;
	uint8_t* data_packet = (uint8_t*)malloc(7 + size);
	// uint8_t data_packet[15];
	for(int i = 0 ; i < size ; ++i) {
		data_packet[SERVO_D_PACKET_PARAMS + 1 + i] = data[i];
		check_sum += data[i];
	}
	check_sum = ~check_sum;
	data_packet[0] = 0xff;
	data_packet[1] = 0xff;
	data_packet[SERVO_D_PACKET_ID] = id_;
	data_packet[SERVO_D_PACKET_LENGTH] = 3 + size;
	data_packet[SERVO_D_PACKET_INSTRUCTION] = SERVO_D_INSTRUCTION_WRITE;
	data_packet[SERVO_D_PACKET_PARAMS] = addr;
	data_packet[3 + data_packet[SERVO_D_PACKET_LENGTH]] = check_sum;

	uart_->Send(
		data_packet,
		data_packet[SERVO_D_PACKET_LENGTH] + 4);

	uint8_t return_pucket[6];
	int status = uart_->Get(return_pucket, 6);
	if (return_pucket[2] != id_ || return_pucket[0] != 0xFF || return_pucket[1] != 0xFF ||  uart_->isError()) 
	{
		throw std::runtime_error(std::string("Write error, Servo! id  = ") + std::to_string(id_) 
			+ std::string("Uart_err = ") + std::to_string(uart_->isError())) ;
	}
	uint8_t state = return_pucket[SERVO_D_PACKET_STATE];
	free(data_packet);
}



Servo_ocs251::Servo_ocs251(uint8_t id, 
	std::shared_ptr<Uart> uart,
	double offset_deg_)
	: uart_(uart)
	, id_(id)
	, offset_deg_(offset_deg_)
{
	if (Ping() != id_)
	{
		throw std::runtime_error(std::string("Ping error, Servo! id  = ") + std::to_string(id_));
	}
}


int Servo_ocs251::Ping()
{
	uint8_t size = 0x2;
	uint8_t check_sum = id_ + SERVO_D_INSTRUCTION_PING + size;
	check_sum = ~check_sum;
	uint8_t data_packet[6] = {
		0xFF,
		0xFF,
		id_,
		size,
		SERVO_D_INSTRUCTION_PING,
		check_sum 
	};

	uart_->Send(data_packet, 6);
	uint8_t data_packet_return[6];
	// uint8_t data_packet_return[10];

	//uart_->Clear();
	int status = uart_->Get(data_packet_return, 6);
	if (data_packet_return[2] != id_ || data_packet_return[0] != 0xFF || data_packet_return[1] != 0xFF ||  uart_->isError()) 
	{
		throw std::runtime_error(std::string("Ping error, Servo! id  = ") + std::to_string(id_));
	}
	return data_packet_return[2];
}


