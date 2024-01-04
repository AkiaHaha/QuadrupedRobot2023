#ifndef PLAN_H_
#define PLAN_H_
#include <vector>
#include <iostream>
#include <array>
#include <aris.hpp>
#include "tools/operator.h"

using namespace aris::dynamic;

/// <summary>
/// const values
/// </summary>
constexpr double pi = aris::PI;
constexpr double kDefaultMajorLength = 0.1;
constexpr double kDefaultMinorLength = 0.01;
constexpr double kDefaultHeight = 0.1;
constexpr int kTcurvePeriodCount = 900;
constexpr int kFactorThousnad = 0.001;
constexpr char kBars10[] = "----------";
constexpr char kBars20[] = "--------------------";
constexpr char kBars40[] = "----------------------------------------";
constexpr char kBars50[] = "--------------------------------------------------";

/// <summary>
/// 功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
/// Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
/// v:速度，由用户输入，构造函数初始化
/// a:加速度，由用户输入，构造函数初始化
/// ta:加速段所需的时间，由输入的速度和加速度计算得到
/// 输入：时间，每毫秒计数一次，注意单位转换
/// 输出：当前时刻s的值
///                                            _________
///     /\                                    /         \ 
///    /  \                                  /           \
///   /    \                                /             \ 
///  /      \                              /               \ 
/// /        \                            /                 \ 
/// -----------                          ---------------------
/// Case1:Triangle                         Case2：Trapezoid
/// </summary>
class Tcurve{
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
	Tcurve(double a, double v) { a_ = a; v_ = v; getCurveParam(); }  //构造函数
	~Tcurve() {} //析构函数
};

/// <summary>
/// ellipse trajectory generator
/// </summary>
class EllipseTrajectory7{
private:
    double moveX_{};
    double moveY_{};
    double moveZ_{};
    double Height_{};
    double theta_{};
    double majorLength_{};
    double startModelPE_[28]{};
    double centerPoint_[12]{};
    double majorUnitAxis_[3]{};
    double minorUnitAxis_[3]{};

public:
    auto trajectoryInitialize() -> void;
    auto getMoveModelPE(double theta, double moveModelPE[28]) -> void;
		auto getCenterPoint()->double* {return centerPoint_;}
		auto getMajorUnitAxis()->double* {return majorUnitAxis_;}
		auto getMinorUnitAxis()->double* {return minorUnitAxis_;}
		auto getStartModelPE()->double* {return startModelPE_;}
		auto getMajorLength()->double {return majorLength_;} 
		auto crossProduct(const double vector1[3], const double vector2[3], double result[3]) -> void;

    EllipseTrajectory7(double* startModelPE, double x, double y, double z, double h): moveX_(x), moveY_(y), moveZ_(z), Height_(h){
			for (std::size_t i = 0; i < 28; ++i){
				startModelPE_[i] = startModelPE[i];
			}
        trajectoryInitialize();
    }
    ~EllipseTrajectory7() {}
};


class EllipseMovePlan {
	private: 
		double vel_x_{};
		double vel_z_{};
		double vel_h_{};

		double* init_m28_;
		double init_mb_[16]{};
		double init_pee_[12]{};

		double move_m28_[28]{};
		double move_mb_[16]{};
		double move_pee_[12]{};

		bool switch_number{};

		Tcurve t_;
		int time_{};
		double lambda{};
		double theta{};
		double delta_p[3]{};



	public:
		auto getCurrentM28(int t)->double*;
		auto planInit() -> void;
		auto legPlan() -> void;
		auto bodyPlan() -> void;
		auto getM28() -> double* { return move_m28_; };
		auto getM16() -> double* { return move_mb_; };
		auto getM12() -> double* { return move_pee_;};

		EllipseMovePlan(double x, double z, double h, bool s, double* initM28)
		: vel_x_(x), vel_z_(z), vel_h_(h), switch_number(s), init_m28_(initM28), t_(5, 2) {
			std::copy(init_m28_, init_m28_ + 16, init_mb_);
			std::copy(init_m28_ + 16, init_m28_ + 28, init_pee_);

			std::copy(init_m28_, init_m28_ + 28, move_m28_);
			std::copy(init_m28_, init_m28_ + 16, move_mb_);
			std::copy(init_m28_ + 16, init_m28_ + 28, move_pee_);
		}
		~EllipseMovePlan() {}
};
#endif