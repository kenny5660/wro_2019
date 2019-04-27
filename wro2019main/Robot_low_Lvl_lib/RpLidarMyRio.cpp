#include "sdkcommon.h"

#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "hal/assert.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/event.h"
#include "rplidar_driver_impl.h"
#include "rplidar_driver_serial.h"
#include "rplidar_driver_TCP.h"
#include "RpLidarMyRio.h"
bool rp::standalone::rplidar::ChannelDeviceMyRioSerial::bind(const char* name, uint32_t baund)
{
	uart_->SetBaundRate(baund);
	return true;
}

bool rp::standalone::rplidar::ChannelDeviceMyRioSerial::open()
{
	return true;
}

rp::standalone::rplidar::ChannelDeviceMyRioSerial::ChannelDeviceMyRioSerial(std::shared_ptr<Uart> uart, 
	std::shared_ptr<MyRio_Dio> dtr)
	: dtr_(dtr)
	, uart_(uart)
{
}


void rp::standalone::rplidar::ChannelDeviceMyRioSerial::close()
{
	
}


void rp::standalone::rplidar::ChannelDeviceMyRioSerial::flush()
{
	uart_->Clear();
}


int rp::standalone::rplidar::ChannelDeviceMyRioSerial::senddata(const _u8 * data, size_t size)
{
	return uart_->Send(data, size);
}


int rp::standalone::rplidar::ChannelDeviceMyRioSerial::recvdata(unsigned char * data, size_t size)
{
	int size_r = uart_->Get(data, size);
	if (uart_->isError())
	{
		throw std::runtime_error("Error chanel device");	
	}
	
	return size_r;
}


void rp::standalone::rplidar::ChannelDeviceMyRioSerial::setDTR()
{
	Dio_WriteBit(dtr_.get(),0);
}


void rp::standalone::rplidar::ChannelDeviceMyRioSerial::clearDTR()
{
	Dio_WriteBit(dtr_.get(), 1);
}


void rp::standalone::rplidar::ChannelDeviceMyRioSerial::ReleaseRxTx()
{
}


bool rp::standalone::rplidar::ChannelDeviceMyRioSerial::waitfordata(size_t data_count, _u32 timeout /* = -1 */, size_t * returned_size /* = NULL */)
{
	return true;
}


u_result rp::standalone::rplidar::RpLidarDriverMyRioSerial::connect(const char * port_path, _u32 baudrate, _u32 flag /* = 0 */)
{
	if (isConnected()) return RESULT_ALREADY_DONE;

	if (!_chanDev) return RESULT_INSUFFICIENT_MEMORY;

	{
		rp::hal::AutoLocker l(_lock);

		// establish the serial connection...
		if(!_chanDev->bind(port_path, baudrate)  ||  !_chanDev->open()) {
			return RESULT_INVALID_DATA;
		}
		_chanDev->flush();
	}

	_isConnected = true;

	checkMotorCtrlSupport(_isSupportingMotorCtrl);
	stopMotor();

	return RESULT_OK;
}


rp::standalone::rplidar::RpLidarDriverMyRioSerial::RpLidarDriverMyRioSerial(std::shared_ptr<Uart> uart, 
	std::shared_ptr<MyRio_Dio> dtr)
{
	_chanDev = new ChannelDeviceMyRioSerial(uart,dtr);	
}
void rp::standalone::rplidar::RpLidarDriverMyRioSerial::disconnect()
{
	if (!_isConnected) return ;
	stop();
}