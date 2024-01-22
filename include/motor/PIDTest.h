#ifndef PID_TEST_H
#define PID_TEST_H
#include <aris.hpp>
#include <memory>
#include "server/server.h"
#include "model/model.h"
#include "motor/PidController.h"
using namespace aris::dynamic;
using namespace aris::plan;

namespace robot {
  class  PidPosCtrl : public aris::core::CloneObject<PidPosCtrl, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PidPosCtrl();
    explicit PidPosCtrl(const std::string& name = "PidPosCtrl");

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
    double ki_{};
    double kd_{};
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
    double toq_des_{};
    double pos_{};
    double vel_{};
    double toq_{};
    double kp_{};
    double ki_{};
    double kd_{};
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

    robot::PController pvtController{ 10,1 };
  };
}
#endif // !PID_TEST_