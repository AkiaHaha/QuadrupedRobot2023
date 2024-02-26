#ifndef BASIC_STATE_CONTROL_H
#define BASIC_STATE_CONTROL_H
#include <aris.hpp>
#include <motor/PidController.h>
#include <control/Plan.h>
#include "server/Server.h"
#include <tools/Operator.h>

namespace robot {
  class  StatePassive2Stance2 : public aris::core::CloneObject<StatePassive2Stance2, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~StatePassive2Stance2();
    explicit StatePassive2Stance2(const std::string& name = "StatePassive2Stance");

  private:
    robot::PidController2 legPidPosController[3] = {
      robot::PidController2(20, 0.02, 0.1),
      robot::PidController2(30, 0.03, 0.15),
      robot::PidController2(40, 0.04, 0.2)
    };
    robot::PidController2 VelLoopController{ 25, 0.1, 0.1 };
    robot::PidController2 PosLoopController{ 1, 1, 1 };

    double init_mPos_[12]{};
    double init_BodyPPFootEE_[28]{};
    double move_height_{};
    double move_BodyPPFootEE_[28]{};
    double move_mPos_[12]{};

    double toq_[12]{};
    double vel_[12]{};
    double pos_[12]{};
    double vel_des_[12]{};
    double toq_cmd_[12]{};
  };


  class  StatePassive2Stance : public aris::core::CloneObject<StatePassive2Stance, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~StatePassive2Stance();
    explicit StatePassive2Stance(const std::string& name = "StatePassive2Stance");

  private:
    robot::PidController2 legPidPosController[3] = {
      robot::PidController2(20, 0.02, 0.1),
      robot::PidController2(30, 0.03, 0.15),
      robot::PidController2(40, 0.04, 0.2)
    };
    robot::PidController2 VelLoopController{ 25, 0.1, 0.1 };
    robot::PidController2 PosLoopController{ 1, 1, 1 };

    double init_mPos_[12]{};
    double init_BodyPPFootEE_[28]{};
    double move_height_{};
    double move_BodyPPFootEE_[28]{};
    double move_mPos_[12]{};
  };

  /// <summary>
  /// Trot move
  /// </summary>
  class  TrotMove : public aris::core::CloneObject<TrotMove, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~TrotMove();
    explicit TrotMove(const std::string& name = "TrotMove");

  private:
    double vel_x{};
    double vel_z{};
    double vel_h{};
    int16_t total_tn{};


    double init_m28[30]{};
    double* move_m28{};
    double move_mb[16]{};
    double move_pee[12]{};
    double move_motor_pos[12]{};
    double init_motor_pos[12]{};
    double period_init_m28[28]{};

    int count_stop{};
    int16_t period_n{};
    int32_t time_in_pn{};

    bool switch_number{};
    //EllipseMovePlan ep;
  };

  /// <summary>
  /// 4 legs run ellipse curve together
  /// </summary>
  class Ellipse4LegDrive3 : public aris::core::CloneObject<Ellipse4LegDrive3, aris::plan::Plan>
  {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~Ellipse4LegDrive3();
    explicit Ellipse4LegDrive3(const std::string& name = "Ellipse4LegDrive3");

  private:
    double moveX_{};
    double moveY_{};
    double moveZ_{};
    double Height_{};

    double theta_{};
    double theta_d_{};
    double theta_dd_{};

    double startMotorPos[12]{};
    double moveMotorPos[12]{};

    double startModelPE[28]{};
    double finalModelPE[28]{};
    double moveModelPE[28]{};

    double startBodyPose[16]{};
    double finalBodyPose[16]{};
    double moveBodyPose[16]{};

    double startLegPoint[12]{};
    double finalLegPoint[12]{};
    double moveLegPoint[12]{};
  };
}
#endif // !BASIC_STATE_CONTROL_H
