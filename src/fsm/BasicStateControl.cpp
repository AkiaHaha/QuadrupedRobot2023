#include "fsm/BasicStateControl.h"
#include "algorithm"
using namespace aris::dynamic;
using namespace aris::plan;
using namespace robot;

auto StatePassive2Stance2::prepareNrt()->void {
  move_height_ = doubleParam("body_move_height_updown");

  for (auto& m : motorOptions()) m =
    aris::plan::Plan::CHECK_NONE;
}
auto StatePassive2Stance2::executeRT()->int {
  if (robot::setAllOperationMode(controller(), OperationMode::kTorqueMode)) {
    mout() << "set torque mode success" << std::endl;
  }

  if (count() == 1 ) {
    //read & print init motor pos// 
    for (size_t i = 0; i < 12; i++) {
      init_mPos_[i] = controller()->motorPool()[i].actualPos();
    }
    std::cout << "init motor pool" << std::endl;
    aris::dynamic::dsp(4, 3, init_mPos_);
    //use init motor pos for init pp//
    modelBase()->setInputPos(init_mPos_);
    if (modelBase()->forwardKinematics()) {THROW_FILE_LINE("Forward kinematics failed");}
    modelBase()->getOutputPos(init_BodyPPFootEE_);
    std::copy(init_BodyPPFootEE_, init_BodyPPFootEE_ + 28, move_BodyPPFootEE_);
    std::cout << "set init pos" << std::endl;
  }
  //create tcurve to plan h//
  Tcurve tcurve(5, 2);
  int kStop = tcurve.getTc() * 1000 - count() -1;

  std::cout << "---->" << kStop << std::endl;

  //set body height and use it to get corresponding motor//
  move_BodyPPFootEE_[11] = init_BodyPPFootEE_[11] + move_height_ * tcurve.getCurve(count());
  std::cout << "actual height" << move_BodyPPFootEE_[11] << std::endl;//-------------------
  modelBase()->setOutputPos(move_BodyPPFootEE_);
  if (modelBase()->inverseKinematics()) {THROW_FILE_LINE("Forward kinematics failed");}
  modelBase()->getInputPos(move_mPos_);
  dsp(4, 3, move_mPos_);//--------------
  for (size_t i = 0; i < 12; i++) {
    toq_[i] = controller()->motorPool()[i].actualToq();
    pos_[i] = controller()->motorPool()[i].actualPos();
    vel_[i] = controller()->motorPool()[i].actualVel();

    vel_des_[i] = PosLoopController.getTargetOut(move_mPos_[i], pos_[i]);
    toq_cmd_[i] = VelLoopController.getTargetOut(vel_des_[i], vel_[i]);

    controller()->motorPool()[i].setTargetToq(toq_cmd_[i]);
    controller()->motorPool()[i].setTargetVel(vel_[i]);
    controller()->motorPool()[i].setTargetPos(pos_[i]);
  }
  //print actual motor pos after pid loop//
  std::cout << "<------------actual motor pos----------->" << std::endl;
  dsp(4, 3, pos_);

  if (kStop == 0)
  {
    double m_pos[12]{};
    double m_PE[28]{};
    double m_pp[16]{};
    double L_ee[12]{};
    for (size_t i = 0; i < 12; i++) {
      m_pos[i] = controller()->motorPool()[i].actualPos();
    }
    modelBase()->setInputPos(m_pos);
    if (modelBase()->forwardKinematics()) { THROW_FILE_LINE("Forward kinematics failed"); }
    modelBase()->getOutputPos(m_PE);

    //print pp and ee seperately//
    std::cout << std::endl << "Final motor pos ===> " << std::endl;
    dsp(4, 3, m_pos);
    splitMatrix28(m_PE, m_pp, L_ee);
    std::cout << std::endl << "Final body pose ===>>> " << std::endl;
    dsp(4, 4, m_pp);
    std::cout << std::endl << "Final Leg ee ===> " << std::endl;
    dsp(4, 3, L_ee);

    std::cout << "Finished !!!" << std::endl;
  }
  return kStop;
}
auto StatePassive2Stance2::collectNrt()->void {}
StatePassive2Stance2::StatePassive2Stance2(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"qqqq\">"
    "	<GroupParam>"
    "	<Param name=\"body_move_height_updown\" default=\"-0.1\" abbreviation=\"h\"/>"
    "	</GroupParam>"
    "</Command>");
}
StatePassive2Stance2::~StatePassive2Stance2() = default;

/*-------------------------StatePassive2Stance run in POS LOOP------------------------------------*/
auto StatePassive2Stance::prepareNrt()->void {
   std::cout  << "pre0" << std::endl;  
  move_height_ = doubleParam("body_move_height_updown");

  for (auto& m : motorOptions()) m =
    aris::plan::Plan::CHECK_NONE;
    mout() << "pre" << std::endl;
}
auto StatePassive2Stance::executeRT()->int {
  if (count() == 1 ) {
    //read & print init motor pos// 
    for (size_t i = 0; i < 12; i++) {
      init_mPos_[i] = controller()->motorPool()[i].actualPos();
      mout() << "prexee" << std::endl;

    }
    std::cout << "init motor pool" << std::endl;
    aris::dynamic::dsp(4, 3, init_mPos_);
    
    //use init motor pos for init pp//
    modelBase()->setInputPos(init_mPos_);
    if (modelBase()->forwardKinematics()) {THROW_FILE_LINE("Forward kinematics failed");}
    modelBase()->getOutputPos(init_BodyPPFootEE_);
    std::copy(init_BodyPPFootEE_, init_BodyPPFootEE_ + 28, move_BodyPPFootEE_);
    std::cout << "set init pos" << std::endl;
  }
  
  //create tcurve to plan h//
  Tcurve tcurve(5, 2);
  int kStop = tcurve.getTc() * 1000 - count() -1;
  std::cout << "T---->" << kStop << std::endl;

  //set body height and use it to get corresponding motor//
  move_BodyPPFootEE_[11] = init_BodyPPFootEE_[11] + move_height_ * tcurve.getCurve(count());
  std::cout << "actual height" << move_BodyPPFootEE_[11] << std::endl;//-------------------
  modelBase()->setOutputPos(move_BodyPPFootEE_);
  if (modelBase()->inverseKinematics()) {THROW_FILE_LINE("Forward kinematics failed");}
  modelBase()->getInputPos(move_mPos_);
  dsp(4, 3, move_mPos_);//--------------//
  
  for (size_t i = 0; i < 12; i++) {
    controller()->motorPool()[i].setTargetPos(move_mPos_[i]);
  }

  if (kStop == 0)
  {
    double m_pos[12]{};
    double m_PE[28]{};
    double m_pp[16]{};
    double L_ee[12]{};
    for (size_t i = 0; i < 12; i++) {
      m_pos[i] = controller()->motorPool()[i].actualPos();
    }
    modelBase()->setInputPos(m_pos);
    if (modelBase()->forwardKinematics()) { THROW_FILE_LINE("Forward kinematics failed"); }
    modelBase()->getOutputPos(m_PE);

    //print pp and ee seperately//
    std::cout << std::endl << "Final motor pos ===> " << std::endl;
    dsp(4, 3, m_pos);
    splitMatrix28(m_PE, m_pp, L_ee);
    std::cout << std::endl << "Final body pose ===>>> " << std::endl;
    dsp(4, 4, m_pp);
    std::cout << std::endl << "Final Leg ee ===> " << std::endl;
    dsp(4, 3, L_ee);

    std::cout << "Finished !!!" << std::endl;
  }
  return kStop;
}
auto StatePassive2Stance::collectNrt()->void {}
StatePassive2Stance::StatePassive2Stance(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"qqq\">"
    "	<GroupParam>"
    "	<Param name=\"body_move_height_updown\" default=\"-0.1\" abbreviation=\"h\"/>"
    "	</GroupParam>"
    "</Command>");
}
StatePassive2Stance::~StatePassive2Stance() = default;

/*--------------------------trot move in pos loop-------------------------------*/
auto TrotMove::prepareNrt()->void
{
  vel_x = doubleParam("vel_x");
  vel_z = doubleParam("vel_z");
  vel_h = doubleParam("vel_h");
  total_tn = doubleParam("total_number_of_run_period");
  for (auto& m : motorOptions()) m =
    aris::plan::Plan::CHECK_NONE;
}
auto TrotMove::executeRT()->int
{
  //��ʼ����ʱ1ms, ���ɳ�ʼλ�˾���, ����
  if (count() == 1) {
    for (int8_t i = 0; i < 4; i++) {
      init_motor_pos[0 + 3 * i] = 0;
      init_motor_pos[1 + 3 * i] = 0.78;
      init_motor_pos[2 + 3 * i] = 1.57;
      init_motor_pos[2] = -1.57;
      init_motor_pos[4] = -0.78;
      init_motor_pos[8] = -1.57;
      init_motor_pos[10] = -0.78;
    }

    modelBase()->setInputPos(init_motor_pos);
    modelBase()->forwardKinematics();
    modelBase()->getOutputPos(init_m28);

    this->master()->logFileRawName("trot_test");
    count_stop = kTcurvePeriodCount * 2 * total_tn - 1;
    std::copy(init_m28, init_m28 + 28, period_init_m28);
  }
  std::cout << kBars50 << "count = " << count() << std::endl;

  //����ѭ�������ڵ�һЩ����ָ��
  period_n = count() / kTcurvePeriodCount + 1;
  time_in_pn = count() % kTcurvePeriodCount;
  switch_number = period_n % 2 == 1 ? false : true;

  //���������µĳ�ʼ���������ڽ�����, ��һ��count()
  if (time_in_pn == 0) {
    double motor[12]{};
    for (int8_t i = 0; i < 12; i++) {
      motor[i] = controller()->motorPool()[i].targetPos();
    }
    modelBase()->setInputPos(motor);
    modelBase()->forwardKinematics();
    modelBase()->getOutputPos(period_init_m28);
  }


  //ʹ����Բ�켣��trot�滮����ʵʱ�˶�����,�ɹ�
  double a{}, b{};
  a = (period_n == 1 ? vel_x : (2 * vel_x));
  b = (period_n == 1 ? vel_z : (2 * vel_z));
  EllipseMovePlan ep(a, b, vel_h, switch_number, period_init_m28);
  move_m28 = ep.getCurrentM28(time_in_pn);

  modelBase()->setOutputPos(move_m28);
  if (modelBase()->inverseKinematics()) {
    throw std::runtime_error("Move Status Inverse kinematic position failed wawawaw");
  }
  modelBase()->getInputPos(move_motor_pos);


  //���Ƶ���˶�
  for (int8_t i = 0; i < 12; ++i) {
    controller()->motorPool()[i].setTargetPos(move_motor_pos[i]);
  }
  std::cout << "current motor pos " << count() << std::endl;
  show(4, 3, move_motor_pos);

  double m16[16]{}, m12[12]{};
  std::copy(move_m28, move_m28 + 16, m16);
  std::copy(move_m28 + 16, move_m28 + 28, m12);
  std::cout << "body pose " << count() << std::endl;
  show(4, 4, m16);
  std::cout << "feet pee " << count() << std::endl;
  show(4, 3, m12);

  //��ĩ���˶�����д���ļ�
  for (int8_t i = 0; i < 4; i++) {
    lout() << std::setw(10) << m12[0 + i * 3] << std::setw(10) << m12[1 + i * 3] << std::setw(10) << m12[2 + i * 3] << std::endl;
  }

  //�������н�����ӡ���յĲ���
  if (count() == count_stop) {
    double m16[16]{}, m12[12]{};
    std::copy(init_m28, init_m28 + 16, m16);
    std::copy(init_m28 + 16, init_m28 + 28, m12);
    std::cout << "init motor_pos,pose, pee" << std::endl;
    show(4, 3, init_motor_pos);
    show(4, 4, m16);
  }

  return count_stop - count();
}
auto TrotMove::collectNrt()->void {}
TrotMove::TrotMove(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"trtr\">"
    "	<GroupParam>"
    "	<Param name=\"vel_x\" default=\"0.1\" abbreviation=\"x\"/>"
    "	<Param name=\"vel_z\" default=\"0\" abbreviation=\"z\"/>"
    "	<Param name=\"vel_h\" default=\"0.1\" abbreviation=\"h\"/>"
    "	<Param name=\"total_number_of_run_period\" default=\"2\" abbreviation=\"n\"/>"
    "	</GroupParam>"
    "</Command>");
}
TrotMove::~TrotMove() = default;
/*---------------------------------------------------------------------------------*/
auto Ellipse4LegDrive3::prepareNrt()->void
{
  moveX_ = doubleParam("moveX");
  moveY_ = doubleParam("moveY");
  moveZ_ = doubleParam("moveZ");
  Height_ = doubleParam("Height");

  for (auto& m : motorOptions()) m =
    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
    aris::plan::Plan::NOT_CHECK_ENABLE |
    aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Ellipse4LegDrive3::executeRT()->int
{
  if (count() == 1)
  {
    //---moveializa theta_---//
    theta_ = PI;

    //---read init motor pos and stored in  startMotorPos---//
    for (int i = 0; i < 12; i++) {
      startMotorPos[i] = controller()->motorPool()[i].targetPos();
    }
    std::cout << "start motor pos ===>>>" << std::endl;
    aris::dynamic::dsp(4, 3, startMotorPos);

    //---use fwdKin with startMotorPos to get startModelPE---//
    mout() << "|||Test flag ===>>> 3" << std::endl;

    modelBase()->setInputPos(startMotorPos);
    if (model()->solverPool()[1].kinPos())
    {
      throw std::runtime_error("Leg Initialization Forward Kinematics Position Failed!");
    }
    modelBase()->getOutputPos(startModelPE);

    //---display startModelPE ---//
    for (int i = 0; i < 16; i++) {
      startBodyPose[i] = startModelPE[i];
    }
    for (int i = 0; i < 12; i++) {
      startLegPoint[i] = startModelPE[i + 16];
    }

    std::cout << std::endl << "Start body pose ===>>> " << std::endl;
    aris::dynamic::dsp(4, 4, startBodyPose);

    std::cout << std::endl << "Start 4-legPoint = " << std::endl;
    aris::dynamic::dsp(4, 3, startLegPoint);
  }
  //---init Ellipse plan function---//
  std::cout << std::endl;
  std::cout << "Prepare to init EllipseTrajectory7~" << std::endl;
  static EllipseTrajectory7  e7(startModelPE, moveX_, moveY_, moveZ_, Height_);

  //---use moveAbsolute2 to plan theta---//
  // std::cout << "Prepare to init theta ~" << std::endl;
  aris::Size total_count;
  auto ret = moveAbsolute2(theta_, theta_d_, theta_dd_, 0, 5, 10, 5, 10, 5, 1e-3, 1e-10, theta_, theta_d_, theta_dd_, total_count);

  // std::cout << "To get moveModelPE ~" << std::endl;
  //---use Ellipse plan function to get trajectory point list---//
  e7.getMoveModelPE(theta_, moveModelPE);

  //---display moveModelPE with corresponding count and theta---//
  std::cout << "count = " << count() << "\t" << "theta = " << theta_ << "\t moveModelPE ===>>>  " << std::endl;


  //---display moveModelPE ---//
  for (int i = 0; i < 16; i++) {
    moveBodyPose[i] = moveModelPE[i];
  }
  for (int i = 0; i < 12; i++) {
    moveLegPoint[i] = moveModelPE[i + 16];
  }

  std::cout << std::endl << "Move body pose ===>>> " << std::endl;
  aris::dynamic::dsp(4, 4, moveBodyPose);

  std::cout << std::endl << "Move 4-legPoint = " << std::endl;
  aris::dynamic::dsp(4, 3, moveLegPoint);

  //---use planned target moveModelPose to get target moveMotorPos---//
  modelBase()->setOutputPos(moveModelPE);
  if (model()->solverPool()[0].kinPos())
  {
    throw std::runtime_error("Move Status Inverse kinematic position failed wawawaw");
  }

  std::cout << std::endl << "move status invKin at count = " << count() << " successed ~" << std::endl;

  //---copy motorPos inverse solved from modelPE to moveMotorPos(12)
  for (int i = 0; i < 12; i++) {
    moveMotorPos[i] = model()->motionPool()[i].mp();
    controller()->motorPool()[i].setTargetPos(moveMotorPos[i]);
  }

  if (ret == 0)
  {
    //--Output motor pos--//
    std::cout << "The Ellipse Curve has Run Successfully~" << std::endl << std::endl;
    std::cout << std::endl << "motorPos===>>> " << std::endl;
    aris::dynamic::dsp(4, 3, moveMotorPos);


    //--Solve the Forward Kinematics--//
    modelBase()->setInputPos(moveMotorPos);

    if (model()->solverPool()[1].kinPos())
    {
      throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
    }
    modelBase()->getOutputPos(finalModelPE);

    //--Output body pose and leg point--//        
    for (int i = 0; i < 16; i++) {
      finalBodyPose[i] = finalModelPE[i];
    }
    for (int i = 0; i < 12; i++) {
      finalLegPoint[i] = finalModelPE[i + 16];
    }

    std::cout << std::endl << "Start body pose ===>>> " << std::endl;
    aris::dynamic::dsp(4, 4, finalBodyPose);

    std::cout << std::endl << "Start 4-legPoint = " << std::endl;
    aris::dynamic::dsp(4, 3, finalLegPoint);
  }
  return ret;
}
auto Ellipse4LegDrive3::collectNrt()->void {}
Ellipse4LegDrive3::Ellipse4LegDrive3(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"e43\">"
    "	<GroupParam>"
    "	<Param name=\"moveX\" default=\"0.1\" abbreviation=\"x\"/>"
    "	<Param name=\"moveY\" default=\"0.08\" abbreviation=\"y\"/>"
    "	<Param name=\"moveZ\" default=\"0\" abbreviation=\"z\"/>"
    "	<Param name=\"Height\" default=\"0.1\" abbreviation=\"h\"/>"
    "	</GroupParam>"
    "</Command>");
}
Ellipse4LegDrive3::~Ellipse4LegDrive3() = default;


ARIS_REGISTRATION{
    aris::core::class_<robot::Ellipse4LegDrive3>("Ellipse4LegDrive3").inherit<aris::plan::Plan>();
    aris::core::class_<robot::TrotMove>("TrotMove").inherit<aris::plan::Plan>();
    aris::core::class_<robot::StatePassive2Stance>("StatePassive2Stance").inherit<aris::plan::Plan>();
    aris::core::class_<robot::StatePassive2Stance2>("StatePassive2Stance2").inherit<aris::plan::Plan>();
}
