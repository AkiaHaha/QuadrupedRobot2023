#ifndef PID_TEST_H
#define PID_TEST_H
#include <aris.hpp>
#include "control/robot.h"
#include "control/plan.h"
#include "server/server.h"
#include "model/model.h"
using namespace aris::dynamic;
using namespace aris::plan;

namespace robot {
  class  PIDTest : public aris::core::CloneObject<PIDTest, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~PIDTest();
    explicit PIDTest(const std::string& name = "PIDTest");

  private:
    int motorId_;
    double toq_cmd_;
    double toq_des_;
    double pos_des_;
    double vel_des_;
    double pos_;
    double vel_;
    double kp_;
    double kd_;
  };
}
#endif // !PID_TEST_