#ifndef PID_TEST_H
#define PID_TEST_H
#include <aris.hpp>
#include <memory>
#include "server/Server.h"
#include "model/Model.h"
#include "motor/PidController.h"
#include "tools/Operator.h"
#include "control/Plan.h"
using namespace aris::dynamic;
using namespace aris::plan;

namespace robot {
  class  PidVelCtrl : public aris::core::CloneObject<PidVelCtrl, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PidVelCtrl();
    explicit PidVelCtrl(const std::string& name = "PidVelCtrl");

  private:
    int motorId_{};
    double toq_cmd_{};
    double vel_des_{};
    double vel_error_set_{};
    double vel_{};
    double toq_{};
    double pos_{};
    double pos_init_{};
    double cur_{};
    double kp_{};
    double ki_{};
    double kd_{};
    int flag_mode_set_{};
    int run_time_{};
    int show_period_{};
    robot::PidController2 velLoopController{25, 0.1, 1};
  };

  class  PidPosVelCtrl : public aris::core::CloneObject<PidPosVelCtrl, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PidPosVelCtrl();
    explicit PidPosVelCtrl(const std::string& name = "PidPosVelCtrl");

  private:
    int motorId_{};
    double toq_cmd_{};
    double pos_move_{};
    double pos_des_{};
    double pos_init_{};
    double vel_des_{};
    double pos_{};
    double vel_{};
    double toq_{};
    double cur_{};
    double kp_{};
    double ki_{};
    double kd_{};
    int flag_mode_set_{};
    int run_time_{};
    int show_period_{};
    double error_set_{};
    double rot_vel_{};
    robot::PidController2 VelLoopController{1, 0.1, 0.01};
    robot::PidController2 PosLoopController{15, 0.1, 0.01};
  };

  class  PDMixedControl : public aris::core::CloneObject<PDMixedControl, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PDMixedControl();
    explicit PDMixedControl(const std::string& name = "PDMixedControl");

  private:
    int motorId_{};
    double toq_cmd_{};
    double pos_move_{};
    double pos_des_{};
    double vel_des_{};
    double toq_des_{};
    double pos_{};
    double vel_{};
    double toq_{};
    double cur_{};
    double kp_{};
    double kd_{};

    int flag_mode_set_{};
    int run_time_{};
    int show_period_{};
    double error_set_{};

    robot::PController pvtController{10, 1};
  };

  class  PdPosCtrl : public aris::core::CloneObject<PdPosCtrl, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PdPosCtrl();
    explicit PdPosCtrl(const std::string& name = "PdPosCtrl");

  private:
    int motorId_{};
    double toq_cmd_{};
    double pos_move_{};
    double pos_des_{};
    double toq_des_{};
    double vel_des_{};
    double error_set_{};
    double vel_{};
    double toq_{};
    double pos_{};
    double cur_{};
    double kp_{};
    double kd_{};
    double init_pos_{};
    int flag_mode_set_{};
    int run_time_{};
    int show_period_{};
    robot::PController PdPosController{40, 20};
    double rot_vel_{};
  };

  class  ToqTest : public aris::core::CloneObject<ToqTest, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~ToqTest();
    explicit ToqTest(const std::string& name = "ToqTest");

  private:
    int motorId_{};
    double toq_cmd_{};
    double vel_cmd_{};
    double pos_cmd_{};
    double pos_{};
    double vel_{};
    double toq_{};
    double cur_{};
    int run_time_{};
    int flag_mode_set_{};

  };

  class  PosTest : public aris::core::CloneObject<PosTest, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PosTest();
    explicit PosTest(const std::string& name = "ToqTest");

  private:
    int motorId_{};
    double pos_cmd_{};
    double pos_init_{};
    double pos_{};
    double vel_{};
    double toq_{};
  };

  class  VelTest : public aris::core::CloneObject<VelTest, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~VelTest();
    explicit VelTest(const std::string& name = "VelTest");

  private:
    int motorId_{};
    double vel_cmd_{};
    double pos_{};
    double vel_{};
    double toq_{};
    int flag_mode_set_{};
    int run_time_{};
    int show_period_{};
    double vel_target_{};
    double kVelEqual{};
    double error_set_{};
  };
}
#endif // !PID_TEST_
