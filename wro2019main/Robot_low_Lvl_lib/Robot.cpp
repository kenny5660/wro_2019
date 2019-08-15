#include "Robot.h"
#include "lidar_math.h"
#include  <exception>
#include <chrono>
#include "debug.h"
#include "VisionAlgs.h"
#include <algorithm>


int Sign(double a)
{
	return a < 0 ? -1 : a == 0 ? 0 : 1 ; 
}


std::shared_ptr<OmniWheels> RobotGardener::GetOmni()
{
	return omni_;
}

void Robot::Delay(int msec)
{
	//	struct timespec tw = { msec / 1000, (msec % 1000) * 1000000 };
	//	struct timespec tr;
	//	nanosleep(&tw, &tr);
		std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}




Robot::~Robot()
{
#ifdef HARDWERE_MODE
MyRio_Close();
#endif
}


std::shared_ptr<Indicator> RobotGardener::GetIndicator()
{
	return indicator_;
}

void RobotGardener::WaitStartButton()
{
	start_but_->WaitDown();
}

std::shared_ptr<CameraRotate> RobotGardener::GetCamRot()
{
	return cam_rot_;
}

std::shared_ptr<OpticalFlow> RobotGardener::GetOptFlow()
{
	return opt_flow_;
}

std::shared_ptr<DistanceSensor> RobotGardener::GetDistSensor(DistSensorEnum dist_sensor)
{
	return dist_sensors_[dist_sensor];
}


std::shared_ptr<Manipulator> RobotGardener::GetMan()
{
	return man_;
}


std::shared_ptr<Lidar> RobotGardener::GetLidar()
{
	return lidar_;
}


color_t RobotGardener::CatchCube(CatchCubeSideEnum side, bool IsTakePhoto)
{
	const int kDist = 65;
	//const int kDistAfter = 110;
	const int kOfsetAngle = -2;
	const int kSpeed = 130;
	const int kSpeedAfter = 150;
	const int kSpeedLow = 31;
	const int kCamAng = 21;
		
	CatchCubeSideEnum side_relative_cube  = AlliginByDist(kDist, kOfsetAngle);
	const int mid_dist = 170;
	int speed = side == CatchCubeSideEnum::LEFT ? kSpeed : -kSpeed;
	int speedLow = side == CatchCubeSideEnum::LEFT ? kSpeedLow : -kSpeedLow;
	std::shared_ptr<DistanceSensor> dist = GetDistSensor(RobotGardener::DIST_TOP);
	std::shared_ptr<DistanceSensor> dist_left =  side == CatchCubeSideEnum::LEFT ?  GetDistSensor(DIST_C_LEFT) : GetDistSensor(DIST_C_RIGHT);
	std::shared_ptr<DistanceSensor> dist_right =  side == CatchCubeSideEnum::LEFT ?  GetDistSensor(DIST_C_RIGHT) : GetDistSensor(DIST_C_LEFT);
	man_->Middle(true);
	Delay(100);
	std::cout << "Dist aligin before = " <<  dist->GetRealDistance() << std::endl;
	if (dist->GetRealDistance() > mid_dist)
	{
		GetOmni()->MoveWithSpeed(std::make_pair(0, speed), 0);

		while (dist->GetDistance() > mid_dist)
		{
			
			std::cout << "Dist aligin  > mid_dist  = " <<  dist->GetDistance() << std::endl;
			if (dist_left->GetDistance() > mid_dist || side_relative_cube == CatchCubeSideEnum::LEFT)
			{
				side_relative_cube =  CatchCubeSideEnum::LEFT;
				std::cout << "go right = " << std::endl;
				GetOmni()->MoveWithSpeed(std::make_pair(0, -speed), 0);
				dist->GetRealDistance();
				while (dist->GetDistance() > mid_dist) ;
				std::cout << "Dist after1 left edge = " <<  dist->GetDistance() << std::endl;
				dist->GetRealDistance();
				std::cout << "Dist after left edge = " <<  dist->GetDistance() << std::endl;
				AlliginByDist(kDist, kOfsetAngle);
				GetOmni()->MoveWithSpeed(std::make_pair(0, -speed), 0);
				Delay(100);
				while (dist->GetDistance() < mid_dist) ;
				dist->GetRealDistance();
				std::cout << "Dist back= " <<  dist->GetDistance() << std::endl;
				GetOmni()->MoveWithSpeed(std::make_pair(0, speedLow), 0);
				while (dist->GetDistance() > mid_dist) ;
				GetOmni()->Stop();
				break;
			}
		}
		
		GetOmni()->Stop();
		dist->GetRealDistance();
		dist->GetRealDistance();
		std::cout << "Dist back= " <<  dist->GetDistance() << std::endl;
		GetOmni()->MoveWithSpeed(std::make_pair(0, -speedLow), 0);
		while (dist->GetDistance() < mid_dist) ;
		
		
		std::cout << "Dist aligin after j = " <<  dist->GetDistance() << std::endl;
	}
	else
	{
		GetOmni()->MoveWithSpeed(std::make_pair(0, -speed), 0);
		while (dist->GetDistance() < mid_dist) ;
		GetOmni()->Stop();
		dist->GetRealDistance();
		std::cout << "Dist back= " <<  dist->GetDistance() << std::endl;
		GetOmni()->MoveWithSpeed(std::make_pair(0, speedLow), 0);
		while (dist->GetDistance() > mid_dist) ;
		GetOmni()->Stop();
			
		
	}
	GetOmni()->Stop();
	std::cout << "Dist aligin after = " <<  dist->GetDistance() << std::endl;

	color_t colorbox;
	if (IsTakePhoto)
	{
		man_->Home(true, 300);
		Delay(359);
		auto frame = cam_rot_->GetFrame(kCamAng);
		colorbox = VisionGetSmallBox(*frame, side);
	}
	
	
	switch (side)
	{
	case CatchCubeSideEnum::LEFT: 
		man_->CatchRight();
		Go2({ Point(0, 30) });
		//		AlliginByDist(kDist, kOfsetAngle);
				man_->Out(true);
		Go2({ Point(0, 110) });
		//Go2({ Point(5, 0) });
		//AlliginByDist(kDistAfter, kOfsetAngle);
		man_->CatchLeft(true, 300);
		man_->Out2(true);
		Go2({ Point(0, 50) });
		man_->Home(true);
		Go2({ Point(0, -89) });
		AlliginByDist(kDist, kOfsetAngle);
		Go2({ Point(-41, 0) });
		break;
	case CatchCubeSideEnum::RIGHT: 
		man_->CatchLeft();
		Go2({ Point(0, 115) });
		man_->Out(true);
		Go2({ Point(0, -112) });
		//Go2({ Point(2, 0) });
		//AlliginByDist(kDistAfter, kOfsetAngle);
		man_->CatchRight(true, 300);
		man_->Out2(true);
		Go2({ Point(0, -45) });
		man_->Home(true);
		Go2({ Point(0, 55) });
		AlliginByDist(kDist, kOfsetAngle);
		Go2({Point(-45, 0)});
		
		break;
	default:
		throw std::runtime_error("wrong CatchCubeSideEnum");
		break;
	}
	GetOmni()->Stop();
	return colorbox;
}

Robot::CatchCubeSideEnum RobotGardener::AlliginByDist(int dist, int offset_alg)
{
	std::shared_ptr<DistanceSensor> dist_left = GetDistSensor(DIST_C_LEFT);
	std::shared_ptr<DistanceSensor> dist_right = GetDistSensor(DIST_C_RIGHT);
	const int kDistDelta = 100;
	const int kSpeedPre = 250;
	CatchCubeSideEnum side_relative_cube = Robot::CatchCubeSideEnum::NONE;
	if (dist_left->GetRealDistance() - dist_right->GetRealDistance() > kDistDelta)
	{
		GetOmni()->MoveWithSpeed(std::make_pair(0, -kSpeedPre), 0);
		while (dist_left->GetDistance() - dist_right->GetDistance() > kDistDelta) ;
		side_relative_cube = Robot::CatchCubeSideEnum::LEFT;
		omni_->MoveToPosInc(std::make_pair(0, -60), kSpeedPre);
		//Delay(150);
	}
	if (dist_right->GetRealDistance() - dist_left->GetRealDistance() > kDistDelta)
	{
		GetOmni()->MoveWithSpeed(std::make_pair(0, kSpeedPre), 0);
		while (dist_right->GetDistance() - dist_left->GetDistance() > kDistDelta) ;
		side_relative_cube = Robot::CatchCubeSideEnum::RIGHT;
		omni_->MoveToPosInc(std::make_pair(0, 60), kSpeedPre);
	}
	
	using namespace std::chrono;
	const  int x_speed_max = 300;
	const milliseconds timeOut(300);
	const int err_dist_limit = 150;
	const int err_align_limit = 30;
	const double P_align = 5;
	const double D_aligin = 8;
	const double P_dist = -6;
	const double D_dist = 4;
	int alg_speed = 0;
	int x_speed = 0;
	int err_dist = INT_MAX;
	int err_dist_old = 0;
	int err_align_old = 0;
	int err_align  = INT_MAX;
	steady_clock::time_point startTime = steady_clock::now(); 
	
	while ((steady_clock::now() - startTime)  < timeOut)//(abs(err_align) > 2  || abs(err_dist) > 2)
	{
		err_align = dist_left->GetDistance() - dist_right->GetDistance() + offset_alg;
		err_align = abs(err_align) > err_align_limit ?  err_align > 0 ?  err_align_limit : -err_align_limit  : err_align;
		alg_speed = err_align*P_align + D_aligin*(err_align - err_align_old);
		
		err_dist = dist - dist_left->GetDistance();
		err_dist = abs(err_dist) > err_dist_limit ?  err_align > 0 ?  err_dist_limit : -err_dist_limit  : err_dist;
		
		x_speed = err_dist*P_dist + D_dist*(err_dist - err_dist_old);
		err_dist_old = err_dist;
		err_align_old = err_align;
		x_speed = std::abs(x_speed) >  x_speed_max ? Sign(x_speed)*x_speed_max : x_speed;
		GetOmni()->MoveWithSpeed(std::make_pair(x_speed, 0), alg_speed);
		Delay(5);
		if ((abs(err_align) > 2 || abs(err_dist) > 2))
		{
			startTime = steady_clock::now(); 
		}
		
	}
	
	GetOmni()->Stop();
	return side_relative_cube;
}



void RobotGardener::WayFromFrame()
{
	const int kDegServo = 286;
	const int kmidDist  = 200;
	AlliginByDist(48, 0);
	std::shared_ptr<DistanceSensor> dist_sensor = GetDistSensor(RobotGardener::DIST_C_LEFT);
	std::shared_ptr<DistanceSensor> dist_c_sensor = GetDistSensor(RobotGardener::DIST_C_RIGHT);
	omni_->MoveWithSpeed(std::make_pair(0, 150), 0);
	dist_sensor->GetRealDistance();
	while (dist_sensor->GetDistance() < kmidDist) ;
	omni_->Stop();
	omni_->MoveToPosInc(std::make_pair(0, 40), 150);
}

void RobotGardener::WayFromFrame(cv::Mat &frame)
{

	const int kmidDist  = 200;
	AlliginByDist(48, -1);
	omni_->MoveToPosInc(std::make_pair(0, 100), 200);
	QrGetFrame(frame);
	std::shared_ptr<DistanceSensor> dist_sensor = GetDistSensor(RobotGardener::DIST_C_LEFT);
	std::shared_ptr<DistanceSensor> dist_c_sensor = GetDistSensor(RobotGardener::DIST_C_RIGHT);
	omni_->MoveWithSpeed(std::make_pair(0, 150), 0);
	dist_sensor->GetRealDistance();
	while (dist_sensor->GetDistance() < kmidDist) ;
	omni_->Stop();

	omni_->MoveToPosInc(std::make_pair(0, 40), 150);
}


void  RobotGardener::MouseTurn(double angle, int speed)
{
	const double r_phi = 31.76 / 180*M_PI;
	const double r = 34.7;

	double L_circle = -r*angle; 
	
	
	using namespace std::chrono;
	const double P = 9;
	const double D = 0;
	
	const double kSmoothStartTime = 800;
	const double kSmoothStartStep = 5;

	
	double max_speed = 0;
	double max_speed_end = speed;
	double err;
	double err_old;
	
	opt_flow_->Reset();
	steady_clock::time_point slippage_startTime = steady_clock::now(); 
	double slippage_err_old;
	steady_clock::time_point  smooth_start_startTime = steady_clock::now(); 
	


	do
	{
		if (steady_clock::now() - smooth_start_startTime  > milliseconds((int)(kSmoothStartTime / (max_speed_end / kSmoothStartStep))) && max_speed <= max_speed_end)
		{
			smooth_start_startTime  = steady_clock::now();
			max_speed += kSmoothStartStep; 
			max_speed = max_speed  > max_speed_end ? max_speed_end : max_speed; 
		}

		std::pair<double, double> pos = GetOptFlow()->GetPos();
		std::pair<double, double> pos_new(pos.first*std::cos(r_phi) - pos.second*std::sin(r_phi), pos.first*std::sin(r_phi) + pos.second*std::cos(r_phi));
		
		err = L_circle - pos_new.second;
		double sp  = err * P + D*(err - err_old);
		sp = std::abs(sp) > max_speed ? Sign(sp)*max_speed : sp;

		err_old = err;
		omni_->MoveWithSpeed({ 0, 0 }, sp);
		Delay(1);
	} while (std::abs(err) > 0.1);
	std::pair<double, double> pos = GetOptFlow()->GetPos();
	std::pair<double, double> pos_new(pos.first*std::cos(r_phi) - pos.second*std::sin(r_phi), pos.first*std::sin(r_phi) + pos.second*std::cos(r_phi));
	std::cout  << "L_circle = "  << L_circle  << " Mouse" << "x = " << pos_new.first  << " y = "  << pos_new.second << std::endl;
}

void RobotGardener::Turn(double angle)
{
	const double pi2 = 2 * M_PI;
	const int kRobot_rot_speed = 130;
		angle = (fmod(fmod(angle, pi2) + pi2, pi2));
	angle = (angle > M_PI) ? (angle - pi2) : (angle); 
	//MouseTurn(angle, kRobot_rot_speed);
	//omni_->Stop();
	omni_->Turn(angle, kRobot_rot_speed);
}


//void RobotGardener::Go2(std::vector<Point> points)
//{
//	const int kRobot_mooving_speed = 200;
//	std::vector<std::pair<int, int>> traj;
//	for (auto it : points)
//	{
//		traj.emplace_back(it.get_x(),it.get_y());
//	}
//	omni_->MoveTrajectory(traj, kRobot_mooving_speed);
//}

void RobotGardener::Go2(std::vector<Point> points)
{
	const int kRobot_mooving_speed = 250; //250;
	for (auto it : points)
	{
		MoveByEncoder(std::make_pair(it.get_x(), it.get_y()), kRobot_mooving_speed);
		//MoveByOptFlow(std::make_pair(it.get_x(), it.get_y()), kRobot_mooving_speed);
		//GetOmni()->MoveToPosInc(std::make_pair(it.get_x(), it.get_y()), kRobot_mooving_speed);
		GetOmni()->Stop();
	}
	
	
}


void RobotGardener::MoveByEncoder(std::pair<int, int> toPos, double speed)
{
	using namespace std::chrono;
	 double P = 1.3;
	 double D = 0;
	
	const milliseconds kSlippageTime = milliseconds(200);
	const int kSlippageSpeedInc = 200;  
	const double kSmoothStartTime = 600;
	const double kSmoothStartStep = 5;
	
	if (speed < 0)
	{
		toPos.first *= -1;
		toPos.second *= -1;
		speed *= -1;
	}
	
	std::pair<double, double> max_speed = std::make_pair(0, 0);
	std::pair<double, double> max_speed_end = std::make_pair(speed, speed);
	std::pair<double, double> err;
	std::pair<double, double> err_old;
	
	if (std::abs(toPos.first) > abs(toPos.second))
	{
		if (abs(toPos.second)  > 21)
		{
			max_speed_end.second = speed*std::abs(((double)toPos.second / toPos.first));
		}
	
	}
	else
	{
		if (abs(toPos.first)  > 21)
		{
			max_speed_end.first = speed*std::abs(((double)toPos.first / toPos.second));
		}
	}
	std::pair<double, double> toPosAng(toPos.first / omni_->GetR_wheel()*180/M_PI, toPos.second / omni_->GetR_wheel() * 180 / M_PI);
	//opt_flow_->Reset();
	omni_->Reset();
	std::pair<steady_clock::time_point, steady_clock::time_point> slippage_startTime = std::make_pair(steady_clock::now(), steady_clock::now()); 
	std::pair<double, double> slippage_err_old;
	std::pair<steady_clock::time_point, steady_clock::time_point>  smooth_start_startTime = std::make_pair(steady_clock::now(), steady_clock::now()); 
	
	std::pair<double, double> cur_pos = omni_->GetPosMm(); //opt_flow_->GetPos();
	err.first = toPos.first - cur_pos.first;
	err.second = toPos.second - cur_pos.second;
	slippage_err_old = err;
	std::pair<double, double> sp;
	do
	{
		if (steady_clock::now() - smooth_start_startTime.first  > milliseconds((int)(kSmoothStartTime / (max_speed_end.first / kSmoothStartStep))) && max_speed.first <= max_speed_end.first)
		{
			smooth_start_startTime.first  = steady_clock::now();
			max_speed.first += kSmoothStartStep; 
			max_speed.first = max_speed.first  > max_speed_end.first ? max_speed_end.first : max_speed.first; 
		}
		if (steady_clock::now() - smooth_start_startTime.second  > milliseconds((int)(kSmoothStartTime / (max_speed_end.second / kSmoothStartStep))) && max_speed.second <= max_speed_end.second)
		{
			smooth_start_startTime.second  = steady_clock::now();
			max_speed.second += kSmoothStartStep; 
			max_speed.second = max_speed.second  > max_speed_end.second ? max_speed_end.second : max_speed.second;
		}
		
		std::pair<double, double> cur_pos = omni_->GetAng(); //opt_flow_->GetPos();
		err.first = toPosAng.first - cur_pos.first;
		err.second = toPosAng.second - cur_pos.second;
		
		 sp  = std::make_pair(err.first * P + D*(err.first - err_old.first), err.second * P + D*(err.second - err_old.second));
		sp.first = std::abs(sp.first) > max_speed.first ? Sign(sp.first)*max_speed.first : sp.first;
		sp.second = std::abs(sp.second) > max_speed.second ? Sign(sp.second)*max_speed.second : sp.second;
		
		
		err_old = err;
		omni_->MoveWithSpeed(sp, 0);
		//std::cout  << "err1 "<< err.first  << "err2" << err.second << std::endl;
	} while (std::abs(err.first) > 150 || std::abs(err.second) > 150);
	omni_->SetAng(toPosAng, 150);
	
//	std::pair<double, double> pos = GetOptFlow()->GetPos();
//	std::cout  << "Encoder" << "x = " << pos.first  << " y = "  << pos.second << std::endl;
}


void RobotGardener::MoveByOptFlow(std::pair<int, int> toPos, double speed)
{
	using namespace std::chrono;
	const double P = 5.3;
	const double D = 0;
	
	const milliseconds kSlippageTime = milliseconds(200);
	const int kSlippageSpeedInc = 200;  
	const double kSmoothStartTime = 600;
	const double kSmoothStartStep = 5;

	if (speed < 0)
	{
		toPos.first *= -1;
		toPos.second *= -1;
		speed *= -1;
	}
	
	std::pair<double, double> max_speed = std::make_pair(0, 0);
	std::pair<double, double> max_speed_end = std::make_pair(speed, speed);
	std::pair<double, double> err;
	std::pair<double, double> err_old;
	
	if (std::abs(toPos.first) > abs(toPos.second))
	{
		if (abs(toPos.second)  > 21)
		{
			max_speed_end.second = speed*std::abs(((double)toPos.second / toPos.first));
		}
	
	}
	else
	{
		if (abs(toPos.first)  > 21)
		{
			max_speed_end.first = speed*std::abs(((double)toPos.first / toPos.second));
		}
	}

	opt_flow_->Reset();
	std::pair<steady_clock::time_point, steady_clock::time_point> slippage_startTime = std::make_pair(steady_clock::now(), steady_clock::now()); 
	std::pair<double, double> slippage_err_old;
	std::pair<steady_clock::time_point, steady_clock::time_point>  smooth_start_startTime = std::make_pair(steady_clock::now(), steady_clock::now()); 
	
	std::pair<double, double> cur_pos = opt_flow_->GetPos();
	err.first = toPos.first - cur_pos.first;
	err.second = toPos.second - cur_pos.second;
	slippage_err_old = err;
	do
	{
		if (steady_clock::now() - smooth_start_startTime.first  > milliseconds((int)(kSmoothStartTime / (max_speed_end.first / kSmoothStartStep))) && max_speed.first <= max_speed_end.first)
		{
			smooth_start_startTime.first  = steady_clock::now();
			max_speed.first += kSmoothStartStep; 
			max_speed.first = max_speed.first  > max_speed_end.first ? max_speed_end.first : max_speed.first; 
		}
		if (steady_clock::now() - smooth_start_startTime.second  > milliseconds((int)(kSmoothStartTime / (max_speed_end.second / kSmoothStartStep))) && max_speed.second <= max_speed_end.second)
		{
			smooth_start_startTime.second  = steady_clock::now();
			max_speed.second += kSmoothStartStep; 
			max_speed.second = max_speed.second  > max_speed_end.second ? max_speed_end.second : max_speed.second;
		}
		
		std::pair<double, double> cur_pos = opt_flow_->GetPos();
		err.first = toPos.first - cur_pos.first;
		err.second = toPos.second - cur_pos.second;
		
		//		if (steady_clock::now() - smooth_start_startTime.first >  kSlippageTime && (err.first - slippage_err_old.first) >= max_speed.first*(kSlippageTime.count() / 1000.0))
		//		{
		//			omni_->MoveWithSpeed(max_speed, 0);
		//			Delay(50);
		//			//max_speed.first = max_speed_end.first + kSlippageSpeedInc;
		//			smooth_start_startTime.first = steady_clock::now();
		//			slippage_err_old.first  = err.first;
		//		}
		
				std::pair<double, double> sp  = std::make_pair(err.first * P + D*(err.first - err_old.first), err.second * P + D*(err.second - err_old.second));
		sp.first = std::abs(sp.first) > max_speed.first ? Sign(sp.first)*max_speed.first : sp.first;
		sp.second = std::abs(sp.second) > max_speed.second ? Sign(sp.second)*max_speed.second : sp.second;
		err_old = err;
		omni_->MoveWithSpeed(sp, 0);
		Delay(1);
	} while (std::abs(err.first) > 1 || std::abs(err.second) > 1);
	std::pair<double, double> pos = GetOptFlow()->GetPos();
	std::cout  << "Mouse" << "x = " << pos.first  << " y = "  << pos.second << std::endl;
}



std::vector<std::pair<int, color_t>> RobotGardener::GetColorFromAng(const std::vector<std::pair<int, PolarPoint>> &ang_pps)
{
	const double cam_ang0 = 246; ///106
	const double cam_ang_offset  = 248;
	std::vector<std::pair<int, PolarPoint>> ang_pps_ = ang_pps;
	std::vector<std::pair<int, color_t>> result;
	double cur_ang = 0;
	for (int i = 0; i < ang_pps_.size(); ++i)
	{
		ang_pps_[i].second.set_f(-ang_pps_[i].second.get_f() + (cam_ang_offset / 180*M_PI));
	}
	sort(ang_pps_.begin(), ang_pps_.end(), [](const std::pair<int, PolarPoint> & a, const std::pair<int, PolarPoint> & b) -> bool{ return a.second.get_f() < b.second.get_f(); });
	
	for (auto it : ang_pps_)
	{
		double ang = (it.second.get_f()) - cur_ang;
		Turn(ang);	
		cur_ang += ang;
		Delay(200);
		auto frame = cam_rot_->GetFrame(cam_ang0);
		color_t color  = VisionGetBigBox(*frame, it.second.get_r());
		result.push_back(std::make_pair(it.first, color));
	}
	Turn(0 - cur_ang);	
	sort(result.begin(), result.end(), [](const std::pair<int, PolarPoint> & a, const std::pair<int, PolarPoint> & b) -> bool{ return a.first < b.first; });
	return result;
}

//void turn_camera()
//{
//	const double cam_ang0 = 20;
//	const double cam_ang_from_robot_ang0 = 170;
//	const double cam_r = 140;
//	double alpha = cam_ang_from_robot_ang0 - PolarPoint::angle_norm(pp.get_f());
//	
//	const double wrong_ang_min  = 30;
//	const double wrong_ang_max  = 310;
//	
//	double offset_ang_robot_turn = 0;
//	//double norm_ang = PolarPoint::angle_norm(pp.get_f());
//	double cam_ang =; ///norm_ang + cam_ang_robot0;
//	
//	
//	cam_ang = cam_ang * 180 / M_PI;
//	if (cam_ang  > wrong_ang_max)
//	{
//		offset_ang_robot_turn =  wrong_ang_max  - cam_ang;
//		Turn(offset_ang_robot_turn*M_PI / 180);
//	}
//	if (cam_ang  < wrong_ang_min)
//	{
//		offset_ang_robot_turn =  cam_ang - wrong_ang_min;
//		Turn(offset_ang_robot_turn*M_PI / 180);
//	}
//	auto frame = cam_rot_->GetFrame(cam_ang - offset_ang_robot_turn);
//	color_t colorbox = VisionGetSmallBox(*frame);
//	return colorbox;
//}

void RobotGardener::QrGetFrame(cv::Mat &frame)
{
	const int kDegServo = 319;
	const int kDegServoAfter = 240;
	Delay(200);
	auto fram = cam_rot_->GetFrame(kDegServo);
	cam_rot_->GetServo()->SetDegrees(kDegServoAfter);
//	cv::Rect cut_rect;
//	cut_rect = cv::Rect(cv::Point(266, 320), cv::Size(350, 350));
//	cv::Mat cut_mat(*fram, cut_rect);
	frame = *fram;	
	save_debug_img("Qrcode.jpg", frame);
}
