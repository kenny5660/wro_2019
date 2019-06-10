#include "OpticalFlow.h"
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

ADNS3080::ADNS3080(std::shared_ptr<Spi> spi, 
	std::shared_ptr<GPIO> ss_pin)
	: spi_(spi)
	, pos_x_(0)
	, pos_y_(0)
	, ss_pin_(ss_pin)
{
	if (Ping_())
	{
		throw std::runtime_error("ADNS3080 error connect!");
	}
}


std::pair < double, double> ADNS3080::GetPos()
{
	return std::make_pair(pos_x_.load(), pos_y_.load());
}


int ADNS3080::ReadReg(uint8_t reg, uint8_t* data, size_t size)
{


	ss_pin_->Reset();

	
	spi_->Transmit(reg);   // Send register address
	std::this_thread::sleep_for(std::chrono::microseconds(75));  // Wait minimum 75 us in case writing to Motion or Motion_Burst registersst
	std::memset(data, 0, size);
	spi_->Transmit(data, data, size);
	ss_pin_->Set();
	return 0;

}

std::pair<double, double> HidMice::GetPos()
{
	double x = pos_x_.load();
	double y = pos_y_.load();//mice_to_mm_coef_;
	double ang = coord_angle_ / 180*M_PI;
	double x_new  = x*std::cos(ang) - y*std::sin(ang);
	double y_new  = x*std::sin(ang) + y*std::cos(ang);
//	return std::make_pair(x*std::cos(coord_angle_) - y*std::sin(coord_angle_), x*std::sin(coord_angle_) + y*std::cos(coord_angle_));
	return std::make_pair(x_new*mice_to_mm_coef_, y_new*mice_to_mm_coef_);
}


void HidMice::Reset()
{
	pos_x_ = 0;
	pos_y_ = 0;
}


void HidMice::ThreadReadMice_Pos_(int fd)
{
		
	int  bytes;
	unsigned char data[4];

	// Open Mouse

	int left, middle, right;
	signed char dx, dy;

	// Read Mouse
	
	while(!thread_stop)
	{
		bytes = read(fd, data, sizeof(data));
		if (bytes > 0)
		{
//			left = data[0] & 0x1;
//			right = data[0] & 0x2;
//			middle = data[0] & 0x4;

			dx = data[1];
			dy = data[2];
		    pos_x_ += dx;
			pos_y_ += dy;
			//printf("x=%d, y=%d, mmx=%f, mmy=%f, left=%d, middle=%d, right=%d\n", x, y, x * 0.017643026, y * 0.017643026, left, middle, right);
		}
	}
}


void HidMice::Start()
{
	
	fd_ = open(pDevice_.c_str(), O_RDONLY);
	if (fd_ == -1)
	{
		throw std::runtime_error (std::string("ERROR Opening ")+pDevice_.c_str());
		return;
	}
	thread_stop = false;
	thread_read_mice_ = std::make_shared<std::thread>(&HidMice::ThreadReadMice_Pos_, this,fd_);

}


HidMice::HidMice(std::string pDevice, double mice_to_mm_coef, double coord_angle)
    : pDevice_(pDevice)
	, pos_x_(0)
	, pos_y_(0)
	, mice_to_mm_coef_(mice_to_mm_coef)
	, coord_angle_(coord_angle)
{
	Start();
}


HidMice::~HidMice()
{
	thread_stop = true;
	thread_read_mice_->join();
	close(fd_);
}