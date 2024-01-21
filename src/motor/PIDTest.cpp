#include "motor/PIDTest.h"

namespace robot {

  auto PIDTest::prepareNrt()->void {

    motorId_ = doubleParam("motorId");


    for (auto& m : motorOptions()) {
      m = aris::plan::Plan::CHECK_NONE;
    }
  }
  auto PIDTest::executeRT()->int {
    if (count() == 1) {
      pos_ = controller()->motorPool()[motorId_].targetPos();
      vel_ = controller()->motorPool()[motorId_].targetVel();
    }

    toq_cmd_ = toq_des_ + kp_ * (pos_des_ - pos_) + (vel_des_ - vel_);

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);

    double toq_now = controller()->motorPool()[motorId_].targetToq();
    double toq_err = abs(toq_now - toq_cmd_);
    int kStop = (toq_err > kErr4) ? 1 : 0;

    return kStop;

  }
  auto PIDTest::collectNrt()->void {}
  PIDTest::PIDTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"Cmd\">"
      "	<Param name=\"coefficient\" default=\"1.0\" abbreviation=\"k\"/>"
      "</Command>");
  }










}
