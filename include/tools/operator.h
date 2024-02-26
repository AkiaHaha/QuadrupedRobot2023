#ifndef OPERATOR_H
#define OPERATOR_H
#include <iostream>
#include <vector>
#include <aris.hpp>
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
using namespace aris::dynamic;
using namespace aris::plan;

/// <summary>
/// const values
/// </summary>
constexpr double pi = aris::PI;
constexpr double kDefaultMajorLength = 0.1;
constexpr double kDefaultMinorLength = 0.01;
constexpr double kDefaultHeight = 0.1;
constexpr int kTcurvePeriodCount = 900;
constexpr int kFactorThousnad = 0.001;
constexpr int kErr3 = 0.001;
constexpr int kErr4 = 0.0001;
constexpr int kErr5 = 0.00001;
constexpr double ktoqFactor = 1000 / 17.93771626666666; //equal about 55.5555555;
constexpr double kRatedToq = 17.93771626666666;
constexpr char kBars10[] = "----------";
constexpr char kBars20[] = "--------------------";
constexpr char kBars40[] = "----------------------------------------";
constexpr char kBars50[] = "--------------------------------------------------";
//enum class OperationMode : uint8_t {
//  kPositionMode = 8,
//  kVelocityMode = 9,
//  kTorqueMode = 10,
//};

enum OperationMode : std::uint8_t {
  kPositionMode = 8,
  kVelocityMode = 9,
  kTorqueMode = 10,
};


/// <summary>
/// classify if 2 number is Quotient Odd
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool inline isQuotientOdd(int a, int b);

/// <summary>
/// splitMatrix28 to 2 mttrix of body pose and leg point
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <param name="c"></param>
void splitMatrix28(double a[28], double b[16], double c[12]);

/// <summary>
/// print an array in matrix type  of row * col automatically
/// </summary>
/// <param name="m"></param>
/// <param name="n"></param>
/// <param name="a"></param>
void show(int8_t m, int8_t n, double* a);
#endif
/*
  if(kStop == 0){
    cm.setModeOfOperation(8);
    cm.setTargetPos(cm.actualPos());
    aris::server::ControlServer::instance().errorChecker().storeServerData();
  }

    vel_ = cm.actualVel();
    pos_ = cm.actualPos();
    toq_ = cm.actualToq();
    cur_ = cm.actualCur();

    if(count() % show_period_ == 0){
      mout() << "T---> " << count() << "\t"
        << "actual vel: " << vel_ << "\t"
        << "vel errorr: " << vel_err << "\t"
        << "cmd toq: " << toq_cmd_ << "\t"
        << "actual toq: " << toq_ << "\t"
        << "actual cur: " << cur_ << "\t"
        << "actual pos: " << pos_ << std::endl;
    }


    auto& cm = controller()->motorPool()[motorId_];
    if(count()==1){
      flag_mode_set_ = 1;
      PdPosController.setPdParam(kp_, kd_);
      pos_des_ = pos_move_ + cm.actualPos();
      mout() << "pd pos test start ---->" <<std::endl;

      init_pos_ = cm.actualPos();
    }
    
    Tcurve tcurve(0.5, 0.2);
    double pos_target = init_pos_ + pos_move_ * tcurve.getCurve(count());

    if(flag_mode_set_ == 1){
      if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
        mout() << "set mode success" << std::endl;
        flag_mode_set_ = 0;
      }else
      return 1;
    }

    vel_ = cm.actualVel();
    pos_ = cm.actualPos();

    toq_cmd_ = PdPosController.getTargetOut(toq_des_, pos_target, pos_, vel_des_, vel_);
    cm.setTargetToq(toq_cmd_);
    cm.setTargetPos(pos_);

    double pos_err = abs(pos_des_ - pos_);
    int kStop = (pos_err <= error_set_) ? 0 : 1;


  if(kStop == 0){
    cm.setModeOfOperation(8);
    cm.setTargetPos(cm.actualPos());
    aris::server::ControlServer::instance().errorChecker().storeServerData();
  }


    std::cout << "init motor pool" << std::endl;
    aris::dynamic::dsp(4, 3, init_mPos_);
    
    //use init motor pos for init pp//

    modelBase()->setInputPos(init_mPos_);
    if (modelBase()->forwardKinematics()) {THROW_FILE_LINE("Forward kinematics failed");}
    modelBase()->getOutputPos(init_BodyPPFootEE_);

    std::copy(init_BodyPPFootEE_, init_BodyPPFootEE_ + 28, move_BodyPPFootEE_);
    std::cout << "set init pos" << std::endl;


    modelBase()->setOutputPos(move_m28);
    if (modelBase()->inverseKinematics()) {
      throw std::runtime_error("Move Status Inverse kinematic position failed wawawaw");
    }
    modelBase()->getInputPos(move_motor_pos);

    if (count() == count_stop) {
    double m16[16]{}, m12[12]{};
    std::copy(init_m28, init_m28 + 16, m16);
    std::copy(init_m28 + 16, init_m28 + 28, m12);
    std::cout << "init motor_pos,pose, pee" << std::endl;
    show(4, 3, init_motor_pos);
    show(4, 4, m16);
  }

	theta = pi * (1 - lambda);
	delta_p[0] = vel_x_ * (1 + std::cos(theta)) * 0.5 * kDefaultMajorLength;
	delta_p[1] = vel_h_ * std::sin(theta) * kDefaultHeight;
	delta_p[2] = vel_z_ * (1 + std::cos(theta)) * 0.5 * kDefaultMinorLength;











*/