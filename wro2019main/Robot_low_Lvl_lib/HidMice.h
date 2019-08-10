#pragma once
#include <utility>
#include <memory>
#include <atomic>
#include <thread>


class HidMice : public  OpticalFlow
{
public:
	HidMice(std::string pDevice, std::pair<double, double> mice_to_mm_coef, double coord_angle = 0);
	std::pair<double, double> GetPos() override;
	std::pair<double, double> GetRowPos();
	void Reset() override;
	virtual ~HidMice();
	
private:
	void Start();
	void ThreadReadMice_Pos_(int fd);
	std::shared_ptr<std::thread> thread_read_mice_;
	std::atomic_bool  thread_stop;
	std::string pDevice_;
	std::atomic_int_fast32_t pos_x_;
	std::atomic_int_fast32_t pos_y_;
	std::pair<double, double> mice_to_mm_coef_;
	int fd_;
	double coord_angle_;
};