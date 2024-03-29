#include "OmniWheels.h"
#include <cmath>
OmniWheels4Squre::OmniWheels4Squre(
	double r_wheel,
	double r_body,
	std::shared_ptr<Motor>motor_front, 
	std::shared_ptr<Motor>motor_left, 
	std::shared_ptr<Motor>motor_back, 
	std::shared_ptr<Motor>motor_right)
	: r_wheel_(r_wheel)
	, r_body_(r_body)
	, motors{motor_left, motor_front, motor_right, motor_back}
{
	
}

//formuls is from picture http://www-ist.massey.ac.nz/conferences/ICARA2004/files/Papers/Paper74_ICARA2004_425_428.pdf
void OmniWheels4Squre::MoveWithSpeed(std::pair<double, double> vSpeed, double angular_speed)
{
	angular_speed = -angular_speed / 180 * M_PI;
	int w_front = ((r_body_*angular_speed - vSpeed.first) / r_wheel_) * 180 / M_PI;    //w1 on picture 
	int w_left = ((r_body_*angular_speed  -  vSpeed.second) / r_wheel_) * 180 / M_PI;   //w2 on picture 
	int w_back = ((r_body_*angular_speed  + vSpeed.first) / r_wheel_) * 180 / M_PI;   //w3 on picture 
	int w_right = ((r_body_*angular_speed + vSpeed.second) / r_wheel_) * 180 / M_PI;   //w4 on picture 
	
	motors[(int)MotorDir::FRONT]->MoveContinue(w_front);
	motors[(int)MotorDir::LEFT]->MoveContinue(w_left);
	motors[(int)MotorDir::BACK]->MoveContinue(w_back);
	motors[(int)MotorDir::RIGHT]->MoveContinue(w_right);

}


void OmniWheels4Squre::Stop()
{
	motors[(int)MotorDir::FRONT]->Stop();
	motors[(int)MotorDir::LEFT]->Stop();
	motors[(int)MotorDir::BACK]->Stop();
	motors[(int)MotorDir::RIGHT]->Stop();
}


std::shared_ptr<Motor> OmniWheels4Squre::GetMotor(MotorDir motor_dir)
{
	return motors[(int)motor_dir];
}


void OmniWheels4Squre::MoveToPosInc(std::pair<double, double> vSpeed, double speed_limit)
{
	speed_limit = speed_limit / r_wheel_ * 180 / M_PI;
	int w_front = ((-vSpeed.first) / r_wheel_) * 180 / M_PI;     //w1 on picture 
	int w_left = ((-vSpeed.second) / r_wheel_) * 180 / M_PI;    //w2 on picture 
	int w_back = ((vSpeed.first) / r_wheel_) * 180 / M_PI;    //w3 on picture 
	int w_right = ((vSpeed.second) / r_wheel_) * 180 / M_PI;    //w4 on picture 
	
	motors[(int)MotorDir::FRONT]->MoveIncDeg(speed_limit, w_front);
	motors[(int)MotorDir::LEFT]->MoveIncDeg(speed_limit, w_left);
	motors[(int)MotorDir::BACK]->MoveIncDeg(speed_limit, w_back);
	motors[(int)MotorDir::RIGHT]->MoveIncDeg(speed_limit, w_right);
	while (!motors[(int)MotorDir::FRONT]->IsReady() ||
		   !motors[(int)MotorDir::LEFT]->IsReady() ||
		   !motors[(int)MotorDir::BACK]->IsReady() ||
		   !motors[(int)MotorDir::RIGHT]->IsReady()) ;
}


int Sign(int a)
{
	return a < 0 ? -1 : a == 0 ? 0 : 1 ; 
}
void OmniWheels4Squre::MoveTrajectory(const std::vector<std::pair<int, int>> &tr, double speed)
{
	const int  kLow_speed = 100; 
	const int  kConpen = -10;
	std::pair<int, int> speedOmni;
	int speed_ = speed;
	for (auto it = tr.begin(); it != tr.end(); ++it)
	{
		if (it == tr.begin())
		{
			speed_ = kLow_speed;
		}
		else
		{
			speed_ = speed;
		}
		
		int start_pos_front = motors[(int)MotorDir::FRONT]->GetCurEncDeg();
		int start_pos_left = motors[(int)MotorDir::LEFT]->GetCurEncDeg();
		int start_pos_back = motors[(int)MotorDir::BACK]->GetCurEncDeg();
		int start_pos_right = motors[(int)MotorDir::RIGHT]->GetCurEncDeg();

		int ang_front = ((-it->first) / r_wheel_) * 180 / M_PI;        //w1 on picture 
		int ang_left = ((-it->second) / r_wheel_) * 180 / M_PI;       //w2 on picture 
		int ang_back = ((it->first) / r_wheel_) * 180 / M_PI;       //w3 on picture 
		int ang_right = ((it->second) / r_wheel_) * 180 / M_PI; 	
		
		speedOmni.first = Sign(it->first)*speed_;
		speedOmni.second = Sign(it->second)*speed_;
		if (std::abs(ang_front) > abs(ang_right))
		{
			speedOmni.second =  Sign(it->second)*speed_*std::abs(((double)ang_right / ang_front));
		}
		else
		{
			speedOmni.first =  Sign(it->first)*speed_*std::abs(((double)ang_front / ang_right));
		}
		MoveWithSpeed(std::make_pair(speedOmni.first, speedOmni.second), 0);	
		while (
			abs(motors[(int)MotorDir::FRONT]->GetCurEncDeg() - start_pos_front) < abs(ang_front)/*+ (abs(ang_front) > -kConpen ? kConpen: 0)*/ || 
		//	abs(motors[(int)MotorDir::LEFT]->GetCurEncDeg() - start_pos_left) < abs(ang_left) ||
		//	abs(motors[(int)MotorDir::BACK]->GetCurEncDeg() - start_pos_back) < abs(ang_back) ||
			abs(motors[(int)MotorDir::RIGHT]->GetCurEncDeg() - start_pos_right) < abs(ang_right) /*+ (abs(ang_right) > -kConpen ? kConpen: 0)*/);
		Stop();
		int a  = 1;
//		MoveToPosInc(*(it), speed);
	}
	Stop();
	//MoveToPosInc(*(tr.end() - 1), speed);
}


void OmniWheels4Squre::Turn(double angl,int speed)
{
	int speed_limit = speed / r_wheel_ * 180 / M_PI;
	double w_front = r_body_*angl;      //w1 on picture 
	double w_left = r_body_*angl;       //w2 on picture 
	double w_back = r_body_*angl;      //w3 on picture 
	double w_right = r_body_*angl;        //w4 on picture 
	
	motors[(int)MotorDir::FRONT]->MoveIncDeg(speed_limit, w_front);
	motors[(int)MotorDir::LEFT]->MoveIncDeg(speed_limit, w_left);
	motors[(int)MotorDir::BACK]->MoveIncDeg(speed_limit, w_back);
	motors[(int)MotorDir::RIGHT]->MoveIncDeg(speed_limit, w_right);
	while (!motors[(int)MotorDir::FRONT]->IsReady() ||
		   !motors[(int)MotorDir::LEFT]->IsReady() ||
		   !motors[(int)MotorDir::BACK]->IsReady() ||
		   !motors[(int)MotorDir::RIGHT]->IsReady()) ;
}


std::pair<double, double> OmniWheels4Squre::GetPosMm()
{
	double degX = motors[(int)MotorDir::FRONT]->GetCurEncDeg() / 180.0 * M_PI;
	double degY = motors[(int)MotorDir::LEFT]->GetCurEncDeg() / 180.0 * M_PI;
	return std::make_pair(-degX*r_wheel_,-degY*r_wheel_);
}
std::pair<double, double> OmniWheels4Squre::GetAng()
{
	double degX = motors[(int)MotorDir::FRONT]->GetCurEncDeg();
	double degY = motors[(int)MotorDir::LEFT]->GetCurEncDeg();
	return std::make_pair(-degX, -degY);
}

void OmniWheels4Squre::Reset()
{
	motors[(int)MotorDir::FRONT]->ResetEnc();
	motors[(int)MotorDir::LEFT]->ResetEnc();
	motors[(int)MotorDir::BACK]->ResetEnc();
	motors[(int)MotorDir::RIGHT]->ResetEnc();
}


double OmniWheels4Squre::GetR_wheel()
{
	return r_wheel_;
}


void OmniWheels4Squre::SetAng(std::pair<double, double> angs, double speeed)
{
	
	motors[(int)MotorDir::FRONT]->MoveToDeg(speeed, -angs.first);
	motors[(int)MotorDir::LEFT]->MoveToDeg(speeed, -angs.second);
	motors[(int)MotorDir::BACK]->MoveToDeg(speeed, angs.first);
	motors[(int)MotorDir::RIGHT]->MoveToDeg(speeed, angs.second);
	while (!motors[(int)MotorDir::FRONT]->IsReady() ||
		   !motors[(int)MotorDir::LEFT]->IsReady() ||
		   !motors[(int)MotorDir::BACK]->IsReady() ||
		   !motors[(int)MotorDir::RIGHT]->IsReady()) ;
	
}
