#ifndef PID_TEST_H
#define PID_TEST_H
#include <aris.hpp>
#include <memory>
#include "server/server.h"
#include "model/Model.h"
#include "motor/PidController.h"
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
    double vel_{};
    double toq_{};
    double kp_{};
    double ki_{};
    double kd_{};
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
    double vel_des_{};
    double pos_{};
    double vel_{};
    double toq_{};
    double kp_{};
    double ki_{};
    double kd_{};
    robot::PidController2 VelLoopController{25, 0.1, 0.1};
    robot::PidController2 PosLoopController{ 1, 1, 1 };
  };

  class  PidPosVelToqCtrl : public aris::core::CloneObject<PidPosVelToqCtrl, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PidPosVelToqCtrl();
    explicit PidPosVelToqCtrl(const std::string& name = "PidPosVelToqCtrl");

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
    double kp_{};
    double kd_{};

    robot::PController pvtController{10, 1};
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
  };
}
#endif // !PID_TEST_
