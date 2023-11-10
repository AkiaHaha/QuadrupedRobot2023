#ifndef TPLAN_H_
#define TPLAN_H_
#include <vector>
#include "operator.h"

///功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
//
//   ##参数定义##
//  Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
//   v:速度，由用户输入，构造函数初始化
//   a:加速度，由用户输入，构造函数初始化
//  ta:加速段所需的时间，由输入的速度和加速度计算得到
class tCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;
	int circleCount;
	int cmlCircle;
	int classifyNumber;//通过判断circleCount的奇偶性给出不同返回值，奇数返回1,偶数返回-1;用于决定每个周期梯形曲线运动的方向

public:
	auto getCurve(int count)->double;  //计算当前的位置
	auto getCurveParam()->void;         //计算梯形曲线的参数
	auto getTc()->double { return Tc_; };  //运行一次T型曲线所需要的时间
	auto getClassifyNumber(int count)->double ;
	tCurve(double a, double v) { a_ = a; v_ = v; }  //构造函数
	~tCurve() {} //析构函数
};


class ellipticalTrajectory
{
private:
	double errorToJudgeMotionType;
	double errorForBisectionMethod;
	double variableToControlHeight1;
	double variableToControlHeight2;
	double bisectionDirection;
	double lambda;
	double polar;
	double Height;
	double Width;
	double phi;
	double startPoint[3]{};
	double endPoint[3]{};
	double centerPoint[3]{};
	double eePoint[3]{};
	double trunkPoint[3]{};
	double unitVectorST[3]{};
	double unitVectorY[3]{};
	int direction{};
	double SC[3]{};
	double ST[3]{};
	double TC[3]{};
	bool initializationStatus;
	int initializationCount{};

public:
	auto trajectoryInitialization() ->void;  
	auto getCenterPoint() -> double* { return centerPoint; } 
	auto getUnitVector() -> double* { return unitVectorST; }
	auto getPhi() -> double { return phi; }
	auto getWidth()-> double { return Width; }
	auto getHeight()->double { return Height; }
	auto getInitializationStatus()->bool  { return initializationStatus; }
	auto getInitializationCount()->double { return initializationCount; }

	// ellipticalTrajectory(double* start[3], double* end[3]) 
	// {
	// 	startPoint = start;
	// 	endPoint = end;
	// }    error message: incompatible types in assignment of ‘double**’ to ‘double [3]
	ellipticalTrajectory(double start[3], double end[3])
	{
   		for (int i = 0; i < 3; i++) 
			{
       	 	startPoint[i] = start[i];
        	endPoint[i] = end[i];
    		}
		trajectoryInitialization();
	}
	~ellipticalTrajectory() {} 
};

class ellipticalTrajectory2
{
private:
	double Width;
	double theta;
	double startPoint[3]{};
	double centerPoint[3]{};
	double unitVector[3]{};

public:
	auto trajectoryInitialization() ->void;  
	auto getCenterPoint() -> double* { return centerPoint; } 
	auto getUnitVector() -> double* { return unitVector; }

	ellipticalTrajectory2(double start[3], double L, double angle)
	{
   		for (int i = 0; i < 3; i++) 
		{
       	 	startPoint[i] = start[i];
		}
		Width = L / 2;
		theta = angle;
		trajectoryInitialization();
	}
	~ellipticalTrajectory2() {} 
};

class ellipticalTrajectory3
{
private:
	double Width;
	double Length;
	double Height;
	double theta;
	double startPoint[3]{};
	double centerPoint[3]{};
	double endPoint[3]{};
	double unitVectorY[3]{};
	double majorAxisUnitVector[3]{};
	double minorAxisUnitVector[3]{};


public:
	auto trajectoryInitialization() ->void;  
	auto getCenterPoint() -> double* { return centerPoint; } 
	auto getMajorAxisUnitVector() -> double* { return majorAxisUnitVector; }
	auto getMinorAxisUnitVector() -> double* {return minorAxisUnitVector;}
	auto getWidth()-> double { return Width; }
	auto crossProduct(const double vector1[3], const double vector2[3], double result[3]) ->void;

	auto init( double start[3], double end[3] )->void
	{
   		for (int i = 0; i < 3; i++) 
		{
       	 	startPoint[i] = start[i];
			endPoint[i] = end[i];
		}
		trajectoryInitialization();
	}
	~ellipticalTrajectory3() {} 
};


class ellipticalTrajectory4
{
private:
	double moveX{};
	double moveY{};
	double moveZ{};
	double Width{};
	double Length{};
	double Height{};
	double startPoint[3]{};
	double centerPoint[3]{};
	double majorAxisUnitVector[3]{};
	double minorAxisUnitVector[3]{};


public:
	auto trajectoryInitialization() ->void;  
	auto getCenterPoint() -> double* { return centerPoint; } 
	auto getMajorAxisUnitVector() -> double* { return majorAxisUnitVector; }
	auto getMinorAxisUnitVector() -> double* {return minorAxisUnitVector;}
	auto getWidth()-> double { return Width; }

	auto init( double x, double y, double z, double start[3] )->void
	{
   		for (int i = 0; i < 3; i++) 
		{
       	 	startPoint[i] = start[i];
		}
		moveX = x;
		moveY = y;
		moveZ = z;
		trajectoryInitialization();
	}
	~ellipticalTrajectory4() {} 
};


class EllipseTrajectory5
{
private:
	double moveX_{};
	double moveY_{};
	double moveZ_{};
	double Height_{};
	double theta_{};
	double majorLength_{};

	Matrix<double> startModelPE_;
	Matrix<double> moveModelPE_;

	Matrix<double> centerPoint_;
	Matrix<double> majorUnitAxis_;
	Matrix<double> minorUnitAxis_;

public:
	auto trajectoryInitialize() -> void;
	auto getMoveModelPE(double theta) -> Matrix<double>;
	EllipseTrajectory5(Matrix<double> startModelPE, double x, double y, double z, double h)
		: startModelPE_(Matrix<double>(4, 7)),
		  moveModelPE_(Matrix<double>(4, 7)),
		  centerPoint_(Matrix<double>(4, 3)),
		  majorUnitAxis_(Matrix<double>(1, 3)),
		  minorUnitAxis_(Matrix<double>(1, 3)),

		  moveX_(x),
		  moveY_(y),
		  moveZ_(z),
		  Height_(h)
	{
		startModelPE_ = std::move(startModelPE);
		trajectoryInitialize();
	}

	~EllipseTrajectory5() {}
};

#endif