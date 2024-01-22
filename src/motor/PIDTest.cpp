#include "motor/PidTest.h"

namespace robot {
  /*-------------PID Pos Control---------------*/
  auto PidPosCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    ki_ = doubleParam("ki");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto PidPosCtrl::executeRT()->int
  {
    if (count() == 1) {
      pos_ = controller()->motorPool()[motorId_].actualPos();
      
      pos_des_ = pos_ + pos_move_;

      mout() << count() << ": pid test go " << "\t";
      std::cout << "hihia" << std::endl;
    }
    
    toq_ = controller()->motorPool()[motorId_].actualToq();
    pos_ = controller()->motorPool()[motorId_].actualPos();
    vel_ = controller()->motorPool()[motorId_].actualVel();

    toq_cmd_ = kp_ * (pos_des_ - pos_);

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);
    controller()->motorPool()[motorId_].setTargetPos(pos_);
    controller()->motorPool()[motorId_].setTargetVel(vel_);
    
    double pos_err = abs(pos_des_ - pos_);
    int kStop = (pos_err > kErr4) ? 1 : 0;
    
    mout() << count() << "\t";
    mout() << "pos desire: " << pos_des_ << "\t";
    mout() << "pos actual: " << controller()->motorPool()[motorId_].targetPos() << "\t";
    mout() << "pos errorr: " << pos_err << "\t";
    mout() << "toq actual: " << controller()->motorPool()[motorId_].targetToq() << std::endl;
    
    return kStop;
  }
  auto PidPosCtrl::collectNrt()->void {}
  PidPosCtrl::PidPosCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pidP\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"1\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"ki\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"kd\" default=\"1\" abbreviation=\"d\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidPosCtrl::~PidPosCtrl() = default;

  /*----------PID Pos Vel Control------------*/
  auto PidPosVelCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");
    vel_des_ = doubleParam("vel_des");

    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    ki_ = doubleParam("ki");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto PidPosVelCtrl::executeRT()->int
  {
    if (count() == 1) {
      pos_ = controller()->motorPool()[motorId_].actualPos();

      pos_des_ = pos_ + pos_move_;

      mout() << count() << ": pid test go " << "\t";
      std::cout << "hihia" << std::endl;
    }

    toq_ = controller()->motorPool()[motorId_].actualToq();
    pos_ = controller()->motorPool()[motorId_].actualPos();
    vel_ = controller()->motorPool()[motorId_].actualVel();

    toq_cmd_ = kp_ * (pos_des_ - pos_) + kd_ * (vel_des_ - vel_);

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);
    controller()->motorPool()[motorId_].setTargetPos(pos_);
    controller()->motorPool()[motorId_].setTargetVel(vel_);

    double pos_err = abs(pos_des_ - pos_);
    double vel_err = abs(vel_des_ - vel_);
    int kStop = ((pos_err > kErr4)  &&  (vel_err > kErr4)) ? 1 : 0;

    mout() << count() << "\t";
    mout() << "pos desire: " << pos_des_ << "\t";
    mout() << "pos actual: " << controller()->motorPool()[motorId_].actualPos() << "\t";
    mout() << "pos errorr: " << pos_err << "\t";
    mout() << "vel desire: " << vel_des_ << "\t";
    mout() << "vel actual: " << controller()->motorPool()[motorId_].actualVel() << "\t";
    mout() << "vel errorr: " << vel_err << "\t";
    mout() << "toq actual: " << controller()->motorPool()[motorId_].targetToq() << std::endl;

    return kStop;
  }
  auto PidPosVelCtrl::collectNrt()->void {}
  PidPosVelCtrl::PidPosVelCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pidPV\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"1\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
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
    if (count() == 1) {
      pos_ = controller()->motorPool()[motorId_].actualPos();

      pos_des_ = pos_ + pos_move_;

      mout() << count() << ": pid test go " << "\t";
      std::cout << "hihia" << std::endl;
    }

    toq_ = controller()->motorPool()[motorId_].actualToq();
    pos_ = controller()->motorPool()[motorId_].actualPos();
    vel_ = controller()->motorPool()[motorId_].actualVel();

    toq_cmd_ = pvtController.getTargetToq(toq_des_, pos_des_, pos_, vel_des_,vel_);

    controller()->motorPool()[motorId_].setTargetToq(toq_cmd_);
    controller()->motorPool()[motorId_].setTargetPos(pos_);
    controller()->motorPool()[motorId_].setTargetVel(vel_);

    double toq_err = abs(toq_cmd_ - toq_);
    double vel_err = abs(vel_des_ - vel_);
    double pos_err = abs(pos_des_ - pos_);
    int kStop = ( toq_err > kErr4 ) ? 1 : 0;

    mout() << count() << "\t";
    mout() << "pos desire: " << pos_des_ << "\t";
    mout() << "pos actual: " << controller()->motorPool()[motorId_].actualPos() << "\t";
    mout() << "pos errorr: " << pos_err << "\t";
    mout() << "vel desire: " << vel_des_ << "\t";
    mout() << "vel actual: " << controller()->motorPool()[motorId_].actualVel() << "\t";
    mout() << "vel errorr: " << vel_err << "\t";
    mout() << "toq actual: " << controller()->motorPool()[motorId_].targetToq() << std::endl;

    return kStop;
  }
  auto PidPosVelToqCtrl::collectNrt()->void {}
  PidPosVelToqCtrl::PidPosVelToqCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pidPVT\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
      "	<Param name=\"toq_des\" default=\"1\" abbreviation=\"t\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"ki\" default=\"0.01\" abbreviation=\"i\"/>"
      "	<Param name=\"kd\" default=\"0.1\" abbreviation=\"d\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidPosVelToqCtrl::~PidPosVelToqCtrl() = default;

}
