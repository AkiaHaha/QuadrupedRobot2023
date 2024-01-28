#include "motor/PidTest.h"

namespace robot {
  /*-------------PID Vel Control---------------*/
  auto PidVelCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    vel_des_ = doubleParam("vel_des");
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    ki_ = doubleParam("ki");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto PidVelCtrl::executeRT()->int
  {
    if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
      mout() << "set torque mode success" << std::endl;
    }
    if (count() == 1) {
      mout() << count() << ": pid test go " << std::endl;
      velLoopController.setPidParam(kp_, ki_, kd_);
    }

    toq_ = controller()->motorPool()[motorId_].actualToq();
    vel_ = controller()->motorPool()[motorId_].actualVel();

    toq_cmd_ = velLoopController.getTargetOut(vel_des_, vel_);

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);
    controller()->motorPool()[motorId_].setTargetVel(vel_);

    double vel_err = abs(vel_des_ - vel_);
    int kStop = (vel_err <= kErr4) ? 0 : 1;

    mout() << count() << "\t";
    mout() << "vel errorr: " << vel_err << "\t";
    mout() << "toq actual: " << controller()->motorPool()[motorId_].targetToq() << std::endl;
    return kStop;
  }

  auto PidVelCtrl::collectNrt()->void {}
  PidVelCtrl::PidVelCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pid1\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"1\" abbreviation=\"m\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"ki\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"kd\" default=\"1\" abbreviation=\"d\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidVelCtrl::~PidVelCtrl() = default;

  /*----------PID Pos Vel Control------------*/
  auto PidPosVelCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");

    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    ki_ = doubleParam("ki");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto PidPosVelCtrl::executeRT()->int
  {

    if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
      mout() << "set torque mode success" << std::endl;
    }

    if (count() == 1) {
      pos_ = controller()->motorPool()[motorId_].actualPos();
      pos_des_ = pos_ + pos_move_;

      PosLoopController.setPidParam(kp_, ki_, kd_);
    }

    toq_ = controller()->motorPool()[motorId_].actualToq();
    pos_ = controller()->motorPool()[motorId_].actualPos();
    vel_ = controller()->motorPool()[motorId_].actualVel();

    vel_des_ = PosLoopController.getTargetOut(pos_des_, pos_);
    toq_cmd_ = VelLoopController.getTargetOut(vel_des_, vel_);

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);
    controller()->motorPool()[motorId_].setTargetVel(vel_);
    controller()->motorPool()[motorId_].setTargetPos(pos_);

    double pos_err = abs(pos_des_ - pos_);
    double vel_err = abs(vel_des_ - vel_);
    int kStop = ((pos_err > kErr4) && (vel_err > kErr4)) ? 1 : 0;

    mout() << count() << "\t"
         << "pos errorr: " << pos_err << "\t"
         << "vel errorr: " << vel_err << "\t"
         << "toq actual: " << controller()->motorPool()[motorId_].actualToq() << std::endl;

    return kStop;
  }
  auto PidPosVelCtrl::collectNrt()->void {}
  PidPosVelCtrl::PidPosVelCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pid2\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"1\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"ki\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"kd\" default=\"1\" abbreviation=\"d\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidPosVelCtrl::~PidPosVelCtrl() = default;


  /*----------PID Pos Vel Toq Control------------*/
  auto PidPosVelToqCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");
    vel_des_ = doubleParam("vel_des");
    toq_des_ = doubleParam("toq_des");

    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto PidPosVelToqCtrl::executeRT()->int
  {
    if (setOperationMode(controller(), static_cast<uint8_t>(OperationMode::kTorqueMode), motorId_)) {
      mout() << "set mode success" << std::endl;
    }

    if (count() == 1) {
      pos_ = controller()->motorPool()[motorId_].targetPos();
      pos_des_ = pos_ + pos_move_;

      std::uint8_t imode = controller()->motorPool()[motorId_].modeOfOperation();
      mout() << "Current mode is :" << imode;
    }

    toq_ = controller()->motorPool()[motorId_].actualToq();
    vel_ = controller()->motorPool()[motorId_].actualVel();
    pos_ = controller()->motorPool()[motorId_].actualPos();

    toq_cmd_ = pvtController.getTargetOut(toq_des_, pos_des_, pos_, vel_des_, vel_);
    //toq_cmd_ = toq_des_ + 0.1 * (pos_des_ - pos_) + 0.1 * ( vel_des_ - vel_);


    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);
    controller()->motorPool()[motorId_].setTargetPos(pos_);
    controller()->motorPool()[motorId_].setTargetVel(vel_);

    double toq_err = abs(toq_cmd_ - toq_);
    double vel_err = abs(vel_des_ - vel_);
    double pos_err = abs(pos_des_ - pos_);
    int kStop = ((toq_err <= kErr3) && (vel_err <= kErr3) && (pos_err <= kErr3) ) ? 0 : 1;

    mout() << count() << "\t";
    mout() << "pos errorr: " << pos_err << "\t";
    mout() << "vel errorr: " << vel_err << "\t";
    mout() << "toq actual: " << controller()->motorPool()[motorId_].actualToq();
    mout() << std::endl;

    return kStop;
  }

  auto PidPosVelToqCtrl::collectNrt()->void {}
  PidPosVelToqCtrl::PidPosVelToqCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pid3\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
      "	<Param name=\"toq_des\" default=\"1\" abbreviation=\"t\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"kd\" default=\"0.1\" abbreviation=\"d\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidPosVelToqCtrl::~PidPosVelToqCtrl() = default;

  /*----------Toq Test------------*/
  auto ToqTest::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto ToqTest::executeRT()->int
  {
    toq_cmd_ = 0.1;
    vel_cmd_ = 0.001;
    pos_cmd_ = 0.00001;

    if (setOperationMode(controller(), OperationMode::kPositionMode, motorId_)) {
      mout() << "set mode success" << std::endl;
    }

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);

    vel_ = controller()->motorPool()[motorId_].targetVel();
    pos_ = controller()->motorPool()[motorId_].targetPos();
    toq_ = controller()->motorPool()[motorId_].targetToq();

    mout() << "T---> " << count() << "\t"
      << "actual toq: " << toq_ << "\t"
      << "actual vel: " << vel_ << "\t"
      << "actual pos: " << pos_ << std::endl;

    return 10 - count();
  }

  auto ToqTest::collectNrt()->void {}
  ToqTest::ToqTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"t5\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  ToqTest::~ToqTest() = default;
}