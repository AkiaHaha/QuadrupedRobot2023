#include<cmath>
#include<cstdlib>
#include<vector>
#include<iostream>
#include "aris.hpp"
#include "tools/operator.h"
#include "model/model.h"
#include "control/plan.h"
using namespace std;

auto Tcurve::getCurve(int count)->double
{
	int t = count % (this->circleCount);
	double s = 0;

	if (2 * ta_ == Tc_)//三角形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	
	else//梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	return s;
}
auto Tcurve::getCurveParam()->void{
	if (v_ * v_ / a_ <= 1)
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		this->Tc_ = 2.0 / v_;  
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
	this->circleCount=int(Tc_ * 1000);
}
auto Tcurve::getClassifyNumber(int count)->double{
	this->cmlCircle = count / (this->circleCount) ;
	if(this->cmlCircle % 2 ==1 )
	{
		this->classifyNumber=1;
	}
	else
	{
		this->classifyNumber=-1;
	}	
	
	return this->classifyNumber;
}

auto EllipseTrajectory7::trajectoryInitialize() ->void
{
	double moveParam_[] = {moveX_, moveY_, moveZ_};

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			centerPoint_[3 * i + j] = startModelPE_[16 + 3 * i + j] + moveParam_[j] / 2;
		}
	}

	majorLength_ = 0.5 * sqrt(moveX_ * moveX_ + moveY_ * moveY_ + moveZ_ * moveZ_);
	
	if (majorLength_ == 0)
	{
		throw std::runtime_error("Major length caculation failed ~");
	}

	for (int i = 0; i < 3; i++)
	{
		majorUnitAxis_[i] = ( centerPoint_[i] - startModelPE_[16 + i] ) / majorLength_;
	}

	double unitVectorY[] = {0, 1, 0};
	double verticalVector[3]{};

	crossProduct(majorUnitAxis_, unitVectorY, verticalVector);
	crossProduct(verticalVector, majorUnitAxis_, minorUnitAxis_);
}
auto EllipseTrajectory7::getMoveModelPE(double theta, double moveModelPE[28]) -> void
{
	theta_ = theta;

	for (int i = 0; i < 28; ++i) 
	{
		moveModelPE[i] = startModelPE_[i];
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			moveModelPE[16 + 3 * i + j] = centerPoint_[3 * i + j] + majorLength_ * majorUnitAxis_[j] * std::cos( theta_ ) + Height_ * minorUnitAxis_[j] * std::sin( theta_ );
		}
	}

}
auto EllipseTrajectory7::crossProduct(const double vector1[3], const double vector2[3], double result[3]) -> void
{
	result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
	result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
}

auto EllipseMovePlan::planInit()->void {
	lambda = t_.getCurve(time_);
	theta = pi * (1 - lambda);
	delta_p[0] = vel_x_ * (1 + std::cos(theta)) * 0.5 * kDefaultMajorLength;
	delta_p[1] = vel_h_ * std::sin(theta) * kDefaultHeight;
	delta_p[2] = vel_z_ * (1 + std::cos(theta)) * 0.5 * kDefaultMinorLength;

	std::cout << "plan init test => ";
	std::cout << "tn: " << time_ << std::setw(4)  << "\t";
	std::cout << "lambda: " << lambda << std::setw(4)  << "\t";
	std::cout << "theta: " << theta << std::setw(4) << std::endl;
	show(1, 3, delta_p);
}

auto EllipseMovePlan::legPlan() -> void{
	if (switch_number){
		for (int8_t i = 0; i < 3; i++){
			move_pee_[3 + i] = delta_p[i] + init_pee_[3 + i];
			move_pee_[9 + i] = delta_p[i] + init_pee_[9 + i];
		}
	}
	else{
		for (int8_t i = 0; i < 3; i++) {
			move_pee_[0 + i] = delta_p[i] + init_pee_[0 + i];
			move_pee_[6 + i] = delta_p[i] + init_pee_[6 + i];
		}
	}
	//std::cout << "---leg plan test---" << std::endl;
	//show(4, 3, init_pee_);
	//show(4, 3, move_pee_);
}

auto EllipseMovePlan::bodyPlan() -> void{
	move_mb_[3] = delta_p[0] + init_mb_[3];
	move_mb_[11] = delta_p[2] + init_mb_[11];
	//std::cout << "---body plan test---" << std::endl;
	//show(4, 4, init_mb_);
	//show(4, 4, move_mb_);
}

auto EllipseMovePlan::getCurrentM28(int t)->double* {
	time_ = t;
	planInit();
	legPlan();
	bodyPlan();
	std::copy(move_mb_, move_mb_ + 16, move_m28_);
	std::copy(move_pee_, move_pee_ + 12, move_m28_ + 16);
	std::cout << kBars40 << "plan test ends" << time_  << std::endl << std::endl;
	return move_m28_;
}