#include"tplan.h"
#include"operator.h"
#include<cmath>
#include<cstdlib>
#include<vector>
#include<iostream>
#include "operator.h"
using namespace std;


/////////////////////////////////////梯形曲线///////////////////////////////////////////////
//生成梯形曲线0->1,用户可输入参数-速度和加速度，并自动判断生成梯形曲线还是三角形曲线
//输入：时间，每毫秒计数一次，注意单位转换
//输出：当前时刻s的值

//                                            _________
//     /\                                    /         \ 
//    /  \                                  /           \
//   /    \                                /             \ 
//  /      \                              /               \ 
// /        \                            /                 \ 
// -----------                          ---------------------
// Case1:Triangle                         Case2：Trapezoid
//--------------------------------------------------------------------------------------------

                            
//==================================获取梯形曲线当下的值===================================//
auto tCurve::getCurve(int count)->double
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

//==================================计算梯形曲线的参数===================================//
                   //由成员函数初始化，对应输入参数由构造函数初始化//
auto tCurve::getCurveParam()->void
{
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

auto tCurve::getClassifyNumber(int count)->double
{
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

//==================================计算椭圆轨迹的相关参数 E1===================================//
auto ellipticalTrajectory::trajectoryInitialization() -> void
{
	trunkPoint[0]=endPoint[0];
	trunkPoint[1]=startPoint[1];
	trunkPoint[2]=endPoint[2];
	variableToControlHeight1=1.5;
	variableToControlHeight2=0.4;
	errorForBisectionMethod=1e-6;
	errorToJudgeMotionType=1e-6;
	initializationStatus=false;

	if( abs(startPoint[1]-endPoint[1] ) <= errorToJudgeMotionType )
	{
		Height=variableToControlHeight2 * abs(startPoint[0]-endPoint[0]);
	}
	else
	{
		Height=variableToControlHeight1 * abs(startPoint[1]-endPoint[1]);
	}

	//to get ST//
	for(int i=0; i<3; ++i)	
	{
		ST[i] = trunkPoint[i] - startPoint[i];
	}

	double lengthOfST = sqrt(std::pow(ST[0] , 2) + std::pow(ST[1] , 2) + std::pow(ST[2] , 2));

	//to get unitVector of ST//
	for(int i=0; i<3; ++i)	
	{
		unitVectorST[i] = ST[i] / lengthOfST;
	}

	// to get the coordinate of centerPoint with bisection method ; whose judgement is to verify the locationShip between endPoint and the ellipse//
	lambda = 0.5;
	for(int i=0; i<3; ++i)
	{
		centerPoint[i] = startPoint[i] + lambda * ST[i];
	}

	while ( std::abs( ( std::pow( (centerPoint[0] - trunkPoint[0]) , 2) + std::pow( (centerPoint[1] - trunkPoint[1]) , 2) +std::pow( (centerPoint[2] - trunkPoint[2]) , 2)
 			) / ( std::pow( (centerPoint[0] - startPoint[0]) , 2) + std::pow( (centerPoint[1] - startPoint[1]) , 2) +std::pow( (centerPoint[2] - startPoint[2]) , 2)
			 ) + std::pow( (endPoint[1] - startPoint[1]) , 2 ) / (std::pow( Height , 2)) - 1 )> errorForBisectionMethod )
	{
		lambda = lambda * 0.5;
		initializationCount = initializationCount + 1;

		direction = ( ( std::pow( (centerPoint[0] - trunkPoint[0]) , 2) + std::pow( (centerPoint[1] - trunkPoint[1]) , 2) +std::pow( (centerPoint[2] - trunkPoint[2]) , 2)
 				) / ( std::pow( (centerPoint[0] - startPoint[0]) , 2) + std::pow( (centerPoint[1] - startPoint[1]) , 2) +std::pow( (centerPoint[2] - startPoint[2]) , 2)
 					) + std::pow( (endPoint[1] - startPoint[1]) , 2 ) / (std::pow( Height , 2)) - 1 > 0 ) ? 1 : -1 ;

		for(int i=0; i<3; ++i)
		{
			centerPoint[i] = centerPoint[i] + direction * lambda * ST[i];
		}

		if ( std::abs( ( std::pow( (centerPoint[0] - trunkPoint[0]) , 2) + std::pow( (centerPoint[1] - trunkPoint[1]) , 2) +std::pow( (centerPoint[2] - trunkPoint[2]) , 2)
 			) / ( std::pow( (centerPoint[0] - startPoint[0]) , 2) + std::pow( (centerPoint[1] - startPoint[1]) , 2) +std::pow( (centerPoint[2] - startPoint[2]) , 2)
			 ) + std::pow( (endPoint[1] - startPoint[1]) , 2 ) / (std::pow( Height , 2)) - 1 ) <= errorForBisectionMethod )
		{
			initializationStatus = true;
		}
	}
	
	for(int i=0; i<3; ++i)	//to get SC & TC//
	{
		SC[i] = centerPoint[i] - startPoint[i];
		TC[i] = centerPoint[i] - trunkPoint[i];
	}

	Width = sqrt(std::pow(SC[0] , 2) + std::pow(SC[1] , 2) + std::pow(SC[2] , 2));
	double lengthOfTC = sqrt(std::pow(TC[0] , 2) + std::pow(TC[1] , 2) + std::pow(TC[2] , 2));

	polar = (endPoint[1] >= startPoint[1]) ? 1: -1;

	phi = polar * std::acos( lengthOfTC / Width );
}



//==================================计算椭圆轨迹的相关参数 E2===================================//
auto ellipticalTrajectory2::trajectoryInitialization() -> void
{
	unitVector[0] = cos(theta);
	unitVector[1] = 0;
	unitVector[2] = sin(theta);

	for(int i=0; i<3; i++ )
	{
		centerPoint[i] = startPoint[i] + Width * unitVector[i] ;
	}
}

//==================================计算椭圆轨迹的相关参数 E3===================================//
auto ellipticalTrajectory3::trajectoryInitialization() -> void
{
	double SE[3]{};
	for (int i = 0; i < 3; i++)
	{
		SE[i] = endPoint[i] - startPoint[i];
	}
	
	Length = std::sqrt( std::pow( SE[0] , 2 ) + std::pow( SE[1] , 2 ) + std::pow( SE[2] , 2 ) );
	Width = Length * 0.5;  

	for (int i = 0; i < 3; i++)
	{
		majorAxisUnitVector[i] = SE[i] / Length ;
	}

	for (int i = 0; i < 3; i++)
	{
		centerPoint[i] = startPoint[i] + Width * majorAxisUnitVector[i] ;
	}

	unitVectorY[0]=0;
	unitVectorY[1]=1;
	unitVectorY[2]=0;

	double vertivalVector[3]{};

    crossProduct(majorAxisUnitVector, unitVectorY, vertivalVector);
	crossProduct(vertivalVector, majorAxisUnitVector, minorAxisUnitVector);
}

auto ellipticalTrajectory3::crossProduct(const double vector1[3], const double vector2[3], double result[3]) -> void
{
	result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
	result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];
}


//================================ calculate ellipse parameters E4 =================================//
auto ellipticalTrajectory4::trajectoryInitialization() -> void
{
	double SE[3]{};
	SE[0] = moveX;
	SE[1] = startPoint[1];
	SE[2] = moveZ;
	Height = moveY;
	
	Length = std::sqrt( std::pow( SE[0] , 2 ) + std::pow( SE[1] , 2 ) + std::pow( SE[2] , 2 ) );
	Width = Length * 0.5;  

	for (int i = 0; i < 3; i++)
	{
		majorAxisUnitVector[i] = SE[i] / Length ;
	}

	for (int i = 0; i < 3; i++)
	{
		centerPoint[i] = startPoint[i] + Width * majorAxisUnitVector[i] ;
	}

	minorAxisUnitVector[0]=0;
	minorAxisUnitVector[1]=1;
	minorAxisUnitVector[2]=0;

}

//============================= calculate ellipse parameter E5 ========================================//
auto EllipseTrajectory5::trajectoryInitialize() ->void
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			centerPoint_(j, i) = startModelPE_(j, i + 4);
		}
	}
	
	majorLength_ = 0.5 * sqrt(moveX_ * moveX_ + moveY_ * moveY_ + moveZ_ * moveZ_);

	for (int i = 0; i < 3; i++)
	{
		majorUnitAxis_(0, i) = (centerPoint_(0, i) - startModelPE_(0, i + 4)) / majorLength_;
	}

	Matrix<double> unitVectorY(1,3);
	Matrix<double> verticalVector(1,3);
	unitVectorY(1,1) = 1;

	verticalVector(0, 0) = majorUnitAxis_(0, 1) * unitVectorY(0, 2) - majorUnitAxis_(0, 2) * unitVectorY(0, 1);
	verticalVector(0, 1) = majorUnitAxis_(0, 2) * unitVectorY(0, 0) - majorUnitAxis_(0, 0) * unitVectorY(0, 2);
	verticalVector(0, 2) = majorUnitAxis_(0, 0) * unitVectorY(0, 1) - majorUnitAxis_(0, 1) * unitVectorY(0, 0);

	minorUnitAxis_(0, 0) = verticalVector(0, 1) * majorUnitAxis_(0, 2) - verticalVector(0, 2) * majorUnitAxis_(0, 1);
	minorUnitAxis_(0, 1) = verticalVector(0, 2) * majorUnitAxis_(0, 0) - verticalVector(0, 0) * majorUnitAxis_(0, 2);
	minorUnitAxis_(0, 2) = verticalVector(0, 0) * majorUnitAxis_(0, 1) - verticalVector(0, 1) * majorUnitAxis_(0, 0);



}

auto EllipseTrajectory5::getMoveModelPE(double theta) -> Matrix<double>
{
	theta_ = theta;
	for (int i = 0; i < 3; i++)
	{
	for (int j = 0; j < 4; j++)
		{
			moveModelPE_(j, i+4) = centerPoint_(j, i) + majorLength_ * majorUnitAxis_(1,i) * std::cos( theta_ ) + Height_ * minorUnitAxis_(1, i) * std::sin( theta_ );
		}
	}

	return moveModelPE_;
}