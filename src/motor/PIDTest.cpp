#include "motor/PIDTest.h"

namespace robot {
/*----------PID Vel Control--------------*/
  auto PidVelCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    vel_des_ = doubleParam("vel_des");
    vel_error_set_ = doubleParam("vel_error_set_");
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    ki_ = doubleParam("ki");
    run_time_ = int32Param("run_time_");
    show_period_ = int32Param("show_period_");

    for (auto& m : motorOptions()) m =
    aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
    aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS | aris::plan::Plan::CHECK_NONE;
  }
  auto PidVelCtrl::executeRT()->int
  {
    if(count()==1){
      flag_mode_set_ = 1;
      velLoopController.setPidParam(kp_, ki_, kd_);
      mout() << "pid vel test start ---->" <<std::endl;
    }
    auto& cm = controller()->motorPool()[motorId_];
    
    if(flag_mode_set_ == 1){
      if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
        mout() << "set mode success" << std::endl;
        flag_mode_set_ = 0;
      }else
      return 1;
    }

    vel_ = cm.actualVel();

    toq_cmd_ = velLoopController.getTargetOut(vel_des_, vel_);
    cm.setTargetToq(toq_cmd_ * ktoqFactor);
    cm.setTargetVel(vel_);

    double vel_err = abs(vel_des_ - vel_);
    // int kStop = (vel_err <= vel_error_set_) ? 0 : 1;
    int kStop = run_time_ * 1000 - count();
    
    vel_ = cm.actualVel();
    pos_ = cm.actualPos();
    toq_ = cm.actualToq() / ktoqFactor;
    cur_ = cm.actualCur();

    if(count() % show_period_ == 0){
      mout() << "T---> " << count() << "\t"
        << "cmd toq: " << toq_cmd_ << "\t"
        << "actual toq: " << toq_ << "\t"
        << "actual vel: " << vel_ << "\t"
        << "actual pos: " << pos_ << "\t"
        << "difference P: " << velLoopController.getDif() << "\t" 
        << "divergence D: " << velLoopController.getDiv() << "\t" 
        << "integration I: " << velLoopController.getItg() << std::endl;

    }

  if(kStop == 0){
    cm.setModeOfOperation(8);
    cm.setTargetPos(cm.actualPos());
    aris::server::ControlServer::instance().errorChecker().storeServerData();
  }

    return kStop;
  }

  auto PidVelCtrl::collectNrt()->void {}
  PidVelCtrl::PidVelCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pid1\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
      "	<Param name=\"vel_error_set\" default=\"0.001\" abbreviation=\"e\"/>"
      "	<Param name=\"kp\" default=\"1\" abbreviation=\"p\"/>"
      "	<Param name=\"ki\" default=\"0.1\" abbreviation=\"i\"/>"
      "	<Param name=\"kd\" default=\"0.1\" abbreviation=\"d\"/>"
      "	<Param name=\"run_time_\" default=\"3\" abbreviation=\"t\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidVelCtrl::~PidVelCtrl() = default;

/*----------PID Pos Vel Control----------*/
  auto PidPosVelCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");
    error_set_ = doubleParam("error_set");
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    ki_ = doubleParam("ki");
    run_time_ = int32Param("run_time");
    show_period_ = int32Param("show_period");
    rot_vel_ = doubleParam("rot_vel");

    for (auto& m : motorOptions()) m =
        aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
  }
  auto PidPosVelCtrl::executeRT()->int
  {
    auto& cm = controller()->motorPool()[motorId_];
    if(count()==1){
      flag_mode_set_ = 1;
      PosLoopController.setPidParam(kp_, ki_, kd_);
      pos_init_= cm.actualPos();
      mout() << "pid pos test start ---->" <<std::endl;
    }
    
    if(flag_mode_set_ == 1){
      if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
        mout() << "set mode success" << std::endl;
        flag_mode_set_ = 0;
      }else
      return 1;
    }

    toq_ = cm.actualToq() / ktoqFactor;
    pos_ = cm.actualPos();
    vel_ = cm.actualVel();
    cur_ = cm.actualCur();

    pos_des_ = pos_init_ + std::sin(pi * count() / 1000 / 4 * rot_vel_) * pos_move_;
    vel_des_ = PosLoopController.getTargetOut(pos_des_, pos_);
    toq_cmd_ = VelLoopController.getTargetOut(vel_des_, vel_);

    cm.setTargetToq(toq_cmd_ * ktoqFactor);
    cm.setTargetVel(vel_des_);
    cm.setTargetPos(pos_des_);

    double pos_err = abs(pos_des_ - pos_);
    double vel_err = abs(vel_des_ - vel_);
    // int kNextPeriod = ((pos_err < error_set_) && (vel_err < error_set_)) ? 0 : 1;

    // if(kNextPeriod == 0){
    //   pos_des_ = cm.actualPos() + pos_move_;
    // }

    int kStop = run_time_ * 1000 - count();

    if(count() % show_period_ == 0){
      mout() << "T---> " << count() << "\t"
        << "cmd toq: " << toq_cmd_ << "\t"
        << "actual toq: " << toq_ << "\t"
        << "actual pos: " << pos_ << "\t"      
        << "actual vel: " << vel_ << "\t"
        << "error pos: " << pos_err << "\t"
        << "error vel: " << vel_err << std::endl;
    }

    if(kStop == 0){
      cm.setModeOfOperation(8);
      cm.setTargetPos(cm.actualPos());
      aris::server::ControlServer::instance().errorChecker().storeServerData();
    }

    return kStop;
  }
  auto PidPosVelCtrl::collectNrt()->void {}
  PidPosVelCtrl::PidPosVelCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pid2\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"error_set\" default=\"0.1\" abbreviation=\"e\"/>"
      "	<Param name=\"run_time\" default=\"100000\" abbreviation=\"t\"/>"
      "	<Param name=\"show_period\" default=\"100\" abbreviation=\"n\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"ki\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"kd\" default=\"1\" abbreviation=\"d\"/>"
      "	<Param name=\"rot_vel\" default=\"1\" abbreviation=\"v\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PidPosVelCtrl::~PidPosVelCtrl() = default;


/*----------PD Mixed Control-------------*/
  auto PDMixedControl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");
    vel_des_ = doubleParam("vel_des");
    toq_des_ = doubleParam("toq_des");
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");    
    error_set_ = doubleParam("error_set");
    run_time_ = int32Param("run_time");
    show_period_ = int32Param("show_period");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::CHECK_NONE;
  }
  auto PDMixedControl::executeRT()->int
  {
    auto& cm = controller()->motorPool()[motorId_];

    if(count()==1){
      flag_mode_set_ = 0;
      pos_ = cm.actualPos();
      pos_des_ = pos_ + pos_move_;
      
      pvtController.setPdParam(kp_, kd_);
      mout() << "pd mixed control test start ---->" <<std::endl;
    }
    
    if(flag_mode_set_ == 0){
      if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
        mout() << "set mode success" << std::endl;
        flag_mode_set_ = 1;
      }else
      return 1;
    }

    toq_ = cm.actualToq() / ktoqFactor;
    pos_ = cm.actualPos();
    vel_ = cm.actualVel();
    cur_ = cm.actualCur();

    toq_cmd_ = pvtController.getTargetOut(toq_des_, pos_des_, pos_, vel_des_, vel_);

    cm.setTargetToq(toq_cmd_ * ktoqFactor);
    cm.setTargetPos(pos_);
    cm.setTargetVel(vel_);

    double toq_err = abs(toq_cmd_ - toq_);
    double vel_err = abs(vel_des_ - vel_);
    double pos_err = abs(pos_des_ - pos_);
    // int kNextPeriod = ((toq_err <= error_set_) && (vel_err <= error_set_) && (pos_err <= error_set_) ) ? 0 : 1;
    
    if(pos_err <= error_set_){
      // pos_des_ = cm.actualPos() + pos_move_;
      mout() << kBars50 << std::endl;
    }

    if(count() % show_period_ == 0){
      mout() << "T---> " << count() << "\t"
        << "cmd toq: " << toq_cmd_ << "\t"
        << "actual toq: " << toq_ << "\t"
        << "actual cur: " << cur_ << "\t"
        << "actual pos: " << pos_ << "\t"      
        << "actual vel: " << vel_ << "\t"
        << "error toq: " << toq_err << "\t"
        << "error pos: " << pos_err << "\t"
        << "error vel: " << vel_err << std::endl;
    }

    int kStop = run_time_ * 1000 - count();
    
    if(kStop == 0){
      cm.setModeOfOperation(8);
      cm.setTargetPos(cm.actualPos());
      aris::server::ControlServer::instance().errorChecker().storeServerData();
    }

    return kStop;
  }

  auto PDMixedControl::collectNrt()->void {}
  PDMixedControl::PDMixedControl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pd\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
      "	<Param name=\"toq_des\" default=\"1\" abbreviation=\"t\"/>"
      "	<Param name=\"kp\" default=\"5\" abbreviation=\"p\"/>"
      "	<Param name=\"kd\" default=\"0.1\" abbreviation=\"d\"/>"     
      "	<Param name=\"error_set\" default=\"0.1\" abbreviation=\"e\"/>"
      "	<Param name=\"run_time\" default=\"100000\" abbreviation=\"r\"/>"
      "	<Param name=\"show_period\" default=\"100\" abbreviation=\"n\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PDMixedControl::~PDMixedControl() = default;

/*----------PD Pos Control---------------*/
  auto PdPosCtrl::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_move_ = doubleParam("pos_move");
    toq_des_ = doubleParam("toq_des");
    vel_des_ = doubleParam("vel_des");
    error_set_ = doubleParam("error_set");
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    run_time_ = int32Param("run_time");
    show_period_ = int32Param("show_period");
    rot_vel_ = doubleParam("rot_vel");

    for (auto& m : motorOptions()) m =
         aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS | 
         aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | 
         aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
         aris::plan::Plan::NOT_CHECK_POS_MIN | 
         aris::plan::Plan::NOT_CHECK_POS_MAX | 
         aris::plan::Plan::NOT_CHECK_VEL_MAX | 
         aris::plan::Plan::NOT_CHECK_VEL_MIN ;
  }
  auto PdPosCtrl::executeRT()->int
  {
    auto& cm = controller()->motorPool()[motorId_];
    if(count()==1){
      flag_mode_set_ = 0;
      PdPosController.setPdParam(kp_, kd_);
      // pos_des_ = pos_move_ + cm.actualPos();
      mout() << "pd pos test start ---->" <<std::endl;

      init_pos_ = cm.actualPos();
    }
    
    // Tcurve tcurve(0.5, 0.2);
    pos_des_ = init_pos_ + std::sin(pi * count() / 1000 / 4 * rot_vel_) * pos_move_;

    if(flag_mode_set_ == 0){
      if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
        mout() << "set mode success" << std::endl;
        flag_mode_set_ = 1;
      }else
      return 1;
    }

    vel_ = cm.actualVel();
    pos_ = cm.actualPos();

    toq_cmd_ = PdPosController.getTargetOut(toq_des_, pos_des_, pos_, vel_des_, vel_);
    // double toq2server = toq_cmd_ * ktoqFactor;
    cm.setTargetToq(toq_cmd_ * ktoqFactor);
    // cm.setTargetPos(pos_);

    // double pos_err = abs(pos_des_ - pos_);
    // int kStop = (pos_err <= error_set_) ? 0 : 1;
    
    vel_ = cm.actualVel();
    pos_ = cm.actualPos();
    toq_ = cm.actualToq() / ktoqFactor;
    cur_ = cm.actualCur();

    if(count() % show_period_ == 0){
      mout() << "T---> " << count() << "\t"
        << "actual pos: " << pos_ << "\t"
        << "target pos: " << pos_des_ << "\t"
        << "cmd toq: " << toq_cmd_ << "\t"
        << "actual toq: " << toq_ << "\t"
        << "actual vel: " << vel_ << std::endl;
    }

    int kStop = run_time_ * 1000 - count();

    // if(kStop == 0){
    //   mout() << "Final---> " << count() << "\t"
    //     << "init pos: " << init_pos_ << "\t"
    //     << "final pos: " << pos_ << "\t"
    //     << "error pos: " << pos_err << std::endl;
    //   cm.setModeOfOperation(10);
    // }

    if(kStop == 0){
      cm.setModeOfOperation(8);
      cm.setTargetPos(cm.actualPos());
      aris::server::ControlServer::instance().errorChecker().storeServerData();
    }

    return kStop;
  }

  auto PdPosCtrl::collectNrt()->void {}
  PdPosCtrl::PdPosCtrl(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"pdPos\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_move\" default=\"0.1\" abbreviation=\"q\"/>"
      "	<Param name=\"toq_des\" default=\"0.1\" abbreviation=\"t\"/>"
      "	<Param name=\"vel_des\" default=\"0.1\" abbreviation=\"v\"/>"
      "	<Param name=\"error_set\" default=\"0.001\" abbreviation=\"e\"/>"
      "	<Param name=\"kp\" default=\"1\" abbreviation=\"p\"/>"
      "	<Param name=\"kd\" default=\"0.1\" abbreviation=\"d\"/>"
      "	<Param name=\"run_time\" default=\"3\" abbreviation=\"r\"/>"
      "	<Param name=\"show_period\" default=\"100\" abbreviation=\"n\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PdPosCtrl::~PdPosCtrl() = default;

/*----------Toq Test---------------------*/
  auto ToqTest::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    toq_cmd_ = doubleParam("toq_cmd");
    run_time_ = int32Param("run_time");
    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS | aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
  }
  auto ToqTest::executeRT()->int
  {
    if(count()==1){
    toq_cmd_ = toq_cmd_ * ktoqFactor;
    flag_mode_set_ = 1;
    }
    // vel_cmd_ = 0.001;
    // pos_cmd_ = 0.00001;
    if(flag_mode_set_ == 1){
      if (setOperationMode(controller(), OperationMode::kTorqueMode, motorId_)) {
        mout() << "set mode success" << std::endl;
        flag_mode_set_ = 0;
      }else
      return 1;
    }
    auto& cm = controller()->motorPool()[motorId_];   
    // cm.setOperationMode(10); 

    cm.setTargetToq(toq_cmd_);

    vel_ = cm.actualVel();
    pos_ = cm.actualPos();
    toq_ = cm.actualToq();
    cur_ = cm.actualCur();
    std::uint32_t actual_vel;
    ecMaster()->slavePool()[0].readPdo(0x606c,0x00,&actual_vel,32);
    if(count() % 100 == 0){
      mout() << "T---> " << count() << "\t"
          << "actual toq: " << toq_ << "\t"
          << "cmd toq: " << toq_cmd_ << "\t"
          << "actual cur: " << cur_ << "\t"
          << "actual vel: " << vel_ << "\t"
          << "actual pos: " << pos_ << "\t"
          << "vel count: " << actual_vel << "\t" << std::endl;
    }

    int kStop = run_time_ * 1000  - count();
    return kStop;
  }

  auto ToqTest::collectNrt()->void {}
  ToqTest::ToqTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"t5\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"toq_cmd\" default=\"0.1\" abbreviation=\"t\"/>"
      "	<Param name=\"runTime\" default=\"3\" abbreviation=\"n\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  ToqTest::~ToqTest() = default;

/*----------Pos Test---------------------*/
  auto PosTest::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    pos_cmd_ = doubleParam("pos_cmd");
    for(auto &m:motorOptions()) m = 
    aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto PosTest::executeRT()->int
  {
    if(count() == 1){
    pos_init_ = controller()->motorPool()[motorId_].actualPos();
    }
    auto& cm = controller()->motorPool()[motorId_];
    Tcurve tcurve(5, 2);
    double targetPos = pos_init_ + pos_cmd_ * tcurve.getCurve(count());
    // if (setOperationMode(controller(), OperationMode::kPositionMode, motorId_)) {
    //   mout() << "set mode success" << std::endl;
    // }
    cm.setTargetPos(targetPos);

    vel_ = cm.actualVel();
    pos_ = cm.actualPos();
    toq_ = cm.actualToq();

    mout() << "T---> " << count() << "\t"
      << "actual toq: " << toq_ << "\t"
      << "actual vel: " << vel_ << "\t"
      << "actual pos: " << pos_ << std::endl;

    int kStop = 899 - count();
    if(kStop == 0){
      std::cout << "init pos:" << pos_init_ << "\t" << "final pos" << pos_ << std::endl;
    }
    return kStop;

  }

  auto PosTest::collectNrt()->void {}
  PosTest::PosTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"p0\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"pos_cmd\" default=\"0.3\" abbreviation=\"p\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  PosTest::~PosTest() = default;

/*----------Vel Test---------------------*/
  auto VelTest::prepareNrt()->void
  {
    motorId_ = int32Param("motorId");
    vel_cmd_ = doubleParam("vel_cmd");
    run_time_ = int32Param("run_time");
    show_period_ = int32Param("show_period");
    error_set_ = doubleParam("error_set");

    for(auto &m:motorOptions()) m = 
    aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS | aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
  }
  auto VelTest::executeRT()->int
  {
    auto& cm = controller()->motorPool()[motorId_];

    // if(count() == 1){
    //   flag_mode_set_ = 1;
    // }
    // if(flag_mode_set_ == 1){
    //   if (setOperationMode(controller(), OperationMode::kVelocityMode, motorId_)) {
    //     mout() << "set vel mode success" << std::endl;
    //     flag_mode_set_ = 0;
    //   }else
    //   return 1;
    // }

    if(count() == 1){
      cm.setModeOfOperation(9);
      cm.setTargetPos(cm.actualPos());
      cm.setTargetVel(cm.actualVel());
      kVelEqual = 1;

      cm.setTargetVel(cm.actualVel());
    }
    Tcurve tcurve(5,2);

    if(count() <= 899){
      vel_target_ = vel_cmd_ * tcurve.getCurve(count());
    }
    std::uint32_t actual_vel;
    ecMaster()->slavePool()[0].readPdo(0x606c,0x00,&actual_vel,32);

    cm.setTargetVel(vel_target_);

    vel_ = cm.actualVel();
    pos_ = cm.actualPos();
    toq_ = cm.actualToq();
    double vel_err = vel_cmd_ - vel_;

    if(count() % show_period_ == 0){
      mout() << "T---> " << count() << "\t"
        << "actual vel: " << vel_ << "\t"
        << "pdo 606c: " << actual_vel << "\t"
        << "cmd vel: " << vel_cmd_ << "\t"
        << "target vel: " << vel_target_ << "\t"
        << "vel errorr: " << vel_err << "\t"
        << "actual toq: " << toq_ << "\t"
        << "pdo 606c: " << actual_vel << "\t"
        << "actual pos: " << pos_ << std::endl;
    }

    int kStop = run_time_ * 1000 - count();
    if(kStop == 0){
      cm.setModeOfOperation(8);
      cm.setTargetPos(cm.actualPos());
      aris::server::ControlServer::instance().errorChecker().storeServerData();
    }
    return kStop;
  }

  auto VelTest::collectNrt()->void {}
  VelTest::VelTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"vv\">"
      "	<GroupParam>"
      "	<Param name=\"motorId\" default=\"0\" abbreviation=\"m\"/>"
      "	<Param name=\"vel_cmd\" default=\"0.3\" abbreviation=\"v\"/>"
      "	<Param name=\"run_time\" default=\"1\" abbreviation=\"t\"/>"
      "	<Param name=\"show_period\" default=\"1000\" abbreviation=\"n\"/>"
      "	<Param name=\"error_set\" default=\"0.001\" abbreviation=\"e\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  VelTest::~VelTest() = default;

ARIS_REGISTRATION{
    aris::core::class_<robot::PidVelCtrl>("PidVelCtrl").inherit<aris::plan::Plan>();
    aris::core::class_<robot::PidPosVelCtrl>("PidPosVelCtrl").inherit<aris::plan::Plan>();
    aris::core::class_<robot::PDMixedControl>("PDMixedControl").inherit<aris::plan::Plan>();
    aris::core::class_<robot::ToqTest>("ToqTest").inherit<aris::plan::Plan>();
    aris::core::class_<robot::PosTest>("PosTest").inherit<aris::plan::Plan>();
    aris::core::class_<robot::VelTest>("VelTest").inherit<aris::plan::Plan>();
    aris::core::class_<robot::PdPosCtrl>("PdPosCtrl").inherit<aris::plan::Plan>();
}
}