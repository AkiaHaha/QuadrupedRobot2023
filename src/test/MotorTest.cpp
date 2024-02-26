#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include "control/Robot.h"
#include "control/Plan.h"
#include "server/Server.h"
#include "model/Model.h"
#include "tools/Operator.h"
#include "test/MotorTest.h"
#include <memory>
#ifdef UNIX
#include <unistd.h>
#endif
namespace robot {
/*----------------read information------------------*/
  auto ReadInformation::prepareNrt()->void
  {
    for (auto& m : motorOptions()) m =
      aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];


  }
  auto ReadInformation::executeRT()->int
  {
    //---read all motor's current pos aris-Matrix---//
    // std::cout << "Motor Pos ===>>> " << std::endl;

    for (int i = 0; i < 12; i++) {
      motorPos[i] = controller()->motorPool()[i].actualPos();
    }
    // aris::dynamic::dsp(4, 3, motorPos);


    modelBase()->setInputPos(motorPos);
    modelBase()->getInputPos(modelPosArray);
    std::cout << std::endl << "Motor Pos ===>>> " << std::endl;
    aris::dynamic::dsp(4, 3, motorPos);




    //---use fwdKin to get ModelPose; storage container uses std::vector---//
    modelPoseVec.resize(28);
    if (modelBase()->forwardKinematics())
    {
      throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
    }
    modelBase()->getOutputPos(modelPoseVec.data());

    double body_pose[16]{};
    double leg_point[12]{};

    for (int i = 0; i < 16; i++)
    {
      body_pose[i] = modelPoseVec[i];
    }

    for (int i = 0; i < 12; i++)
    {
      leg_point[i] = modelPoseVec[i + 16];
    }

    std::cout << std::endl << "Body Pose ===>>> " << std::endl;
    aris::dynamic::dsp(4, 4, body_pose);
    std::cout << std::endl << "Leg Point ===>>>" << std::endl;
    aris::dynamic::dsp(4, 3, leg_point);

    return 0;
  }
  auto ReadInformation::collectNrt()->void {}
  ReadInformation::ReadInformation(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"rr\">"
      "	<GroupParam>"
      "	</GroupParam>"
      "</Command>");
  }
  ReadInformation::~ReadInformation() = default;
/*----------------seperate motor move---------------*/
  auto MotorTest::prepareNrt()->void
  {
    motor0_ = doubleParam("motor0");
    motor1_ = doubleParam("motor1");
    motor2_ = doubleParam("motor2");
    motor3_ = doubleParam("motor3");
    motor4_ = doubleParam("motor4");
    motor5_ = doubleParam("motor5");
    motor6_ = doubleParam("motor6");
    motor7_ = doubleParam("motor7");
    motor8_ = doubleParam("motor8");
    motor9_ = doubleParam("motor9");
    motor10_ = doubleParam("motor10");
    motor11_ = doubleParam("motor11");


    for (auto& m : motorOptions()) m =
      aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
  }
  auto MotorTest::executeRT()->int
  {
    if (count() == 1) {
      //--get init motor pos and target motor pos--//
      double pos_commad_[12] = { motor0_, motor1_, motor2_, motor3_, motor4_, motor5_, motor6_, motor7_, motor8_, motor9_, motor10_, motor11_ };

      std::cout << "pos command ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, pos_commad_);

      for (int i = 0; i < 12; i++) {
        pos_[i] = controller()->motorPool()[i].targetPos();
        target_pos_[i] = pos_[i] + pos_commad_[i];
      }

      std::cout << "init pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, pos_);

      std::cout << "target pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, target_pos_);

    }

    //--use fun to plan pos--//
    aris::Size total_count[12]{};

    for (int i = 0; i < 12; i++) {
      ret[i] = moveAbsolute2(pos_[i], pos_d_[i], pos_dd_[i], target_pos_[i], 0.1, 1, 1, 0.5, 0.5, 1e-3, 1e-10, pos_[i], pos_d_[i], pos_dd_[i], total_count[i]);
    }

    //--use ret_all to check ret[12]; if exist 1, it comes 1; if no 1 exist, it keeps 0;--//
    ret_all = 0;
    for (int i = 0; i < 12; i++) {
      if (ret[i] != 0) {
        ret_all = 1; // ֻҪ�����д���ֵ��Ϊ0��Ԫ�أ��ͽ� ret_all ����Ϊ1
        break;       // �ҵ�һ���Ϳ�������ѭ����
      }
    }

    //--set motor pos to planned pos--//
    for (int j = 0; j < 12; j++)
    {
      controller()->motorPool()[j].setTargetPos(pos_[j]);
    }

    //--output motor pos with interval count = 500 --//
    if (count() % 500 == 0)
    {
      for (int i = 0; i < 12; i++) {
        motorPos_[i] = controller()->motorPool()[i].targetPos();
      }
      std::cout << "count= " << count() << "; 12 Motor Pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, motorPos_);
    }

    if (ret_all == 0)
    {
      std::cout << std::endl << "motorPos: " << std::endl;
      std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() << "\t" << controller()->motorPool()[1].actualPos() << "\t" << controller()->motorPool()[2].actualPos() << std::endl;
      std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() << "\t" << controller()->motorPool()[4].actualPos() << "\t" << controller()->motorPool()[5].actualPos() << std::endl;
      std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() << "\t" << controller()->motorPool()[7].actualPos() << "\t" << controller()->motorPool()[8].actualPos() << std::endl;
      std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() << "\t" << controller()->motorPool()[10].actualPos() << "\t" << controller()->motorPool()[11].actualPos() << std::endl;

      double finalPos[12]{ 0 };
      for (int i = 0; i < 12; i++)
      {
        finalPos[i] = controller()->motorPool()[i].actualPos();
      }

      //Solve the Forward Kinematics
      modelBase()->setInputPos(finalPos);

      if (modelBase()->forwardKinematics())
      {
        throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
      }

      double finalPE[28]{ 0 };
      modelBase()->getOutputPos(finalPE);

      std::cout << std::endl << "Final body pose = " << std::endl;
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; ++j)
        {
          std::cout << finalPE[4 * i + j] << "\t";
        }
        std::cout << std::endl;
      }


      std::cout << std::endl << "Final 4-legPos = " << std::endl;
      for (int i = 0; i < 4; i++)
      {
        std::cout << "Leg" << i + 1 << " = ";
        for (int j = 0; j < 3; ++j)
        {
          std::cout << finalPE[16 + 3 * i + j] << "\t";
        }
        std::cout << std::endl;
      }

    }

    return ret_all;
  }
  auto MotorTest::collectNrt()->void {}
  MotorTest::MotorTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"t\">"
      "	<GroupParam>"
      "	<Param name=\"motor0\" default=\"0.0\" abbreviation=\"q\"/>"  // Initialize the end pose of a single leg by giving the initial value of the motor. //
      "	<Param name=\"motor1\" default=\"0.0\" abbreviation=\"a\"/>"  // The given value is in radians. The accuracy in this program is set to four decimal places.//
      "	<Param name=\"motor2\" default=\"0.0\" abbreviation=\"z\"/>"
      "	<Param name=\"motor3\" default=\"0.0\" abbreviation=\"w\"/>"
      "	<Param name=\"motor4\" default=\"0.0\" abbreviation=\"s\"/>"
      "	<Param name=\"motor5\" default=\"0.0\" abbreviation=\"x\"/>"
      "	<Param name=\"motor6\" default=\"0.0\" abbreviation=\"e\"/>"
      "	<Param name=\"motor7\" default=\"0.0\" abbreviation=\"d\"/>"
      "	<Param name=\"motor8\" default=\"0.0\" abbreviation=\"c\"/>"
      "	<Param name=\"motor9\" default=\"0.0\" abbreviation=\"r\"/>"
      "	<Param name=\"motor10\" default=\"0.0\" abbreviation=\"f\"/>"
      "	<Param name=\"motor11\" default=\"0.0\" abbreviation=\"v\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  MotorTest::~MotorTest() = default;
/*----------------set motor pos zero----------------*/
  auto SetMotorPosZero::prepareNrt()->void
  {
    for (auto& m : motorOptions()) m =
      aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
  }
  auto SetMotorPosZero::executeRT()->int
  {
    if (count() == 1) {
      //--get current motor pos--//
      for (int i = 0; i < 12; i++)
      {
        pos_[i] = controller()->motorPool()[i].targetPos();
      }
      std::cout << "current pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, pos_);
    }

    //--use fun to plan pos--//
    aris::Size total_count[12]{};

    for (int i = 0; i < 12; i++) {
      ret[i] = moveAbsolute2(pos_[i], pos_d_[i], pos_dd_[i], 0, 1, 0.1, 1, 0.5, 0.5, 1e-3, 1e-10, pos_[i], pos_d_[i], pos_dd_[i], total_count[i]);
    }

    //--use ret_all to check ret[12]; if exist 1, it comes 1; if no 1 exist, it keeps 0;--//
    ret_all = 0;
    for (int i = 0; i < 12; i++) {
      if (ret[i] != 0) {
        ret_all = 1; // ֻҪ�����д���ֵ��Ϊ0��Ԫ�أ��ͽ� ret_all ����Ϊ1
        break;       // �ҵ�һ���Ϳ�������ѭ����
      }
    }

    //--set motor pos to planned pos--//
    for (int j = 0; j < 12; j++)
    {
      controller()->motorPool()[j].setTargetPos(pos_[j]);
    }

    //--output motor pos with interval count = 500 --//
    if (count() % 500 == 0)
    {
      for (int i = 0; i < 12; i++) {
        motorPos_[i] = controller()->motorPool()[i].targetPos();
      }
      std::cout << "count= " << count() << "; 12 Motor Pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, motorPos_);
    }

    if (ret_all == 0)
    {
      std::cout << std::endl << "motorPos: " << std::endl;
      std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() << "\t" << controller()->motorPool()[1].actualPos() << "\t" << controller()->motorPool()[2].actualPos() << std::endl;
      std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() << "\t" << controller()->motorPool()[4].actualPos() << "\t" << controller()->motorPool()[5].actualPos() << std::endl;
      std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() << "\t" << controller()->motorPool()[7].actualPos() << "\t" << controller()->motorPool()[8].actualPos() << std::endl;
      std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() << "\t" << controller()->motorPool()[10].actualPos() << "\t" << controller()->motorPool()[11].actualPos() << std::endl;

      double finalPos[12]{ 0 };
      for (int i = 0; i < 12; i++)
      {
        finalPos[i] = controller()->motorPool()[i].actualPos();
      }

      //Solve the Forward Kinematics//
      modelBase()->setInputPos(finalPos);

      if (modelBase()->forwardKinematics())
      {
        throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
      }

      double finalPE[28]{ 0 };
      modelBase()->getOutputPos(finalPE);

      std::cout << std::endl << "Final body pose = " << std::endl;
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; ++j)
        {
          std::cout << finalPE[4 * i + j] << "\t";
        }
        std::cout << std::endl;
      }


      std::cout << std::endl << "Final 4-legPos = " << std::endl;
      for (int i = 0; i < 4; i++)
      {
        std::cout << "Leg" << i + 1 << " = ";
        for (int j = 0; j < 3; ++j)
        {
          std::cout << finalPE[16 + 3 * i + j] << "\t";
        }
        std::cout << std::endl;
      }

    }

    return ret_all;
  }
  auto SetMotorPosZero::collectNrt()->void {}
  SetMotorPosZero::SetMotorPosZero(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"zero\">"
      "	<GroupParam>"
      "	</GroupParam>"
      "</Command>");
  }
  SetMotorPosZero::~SetMotorPosZero() = default;
/*----------------Model Motor Initialize------------*/
  auto ModelMotorInitialize::prepareNrt()->void
  {
    motor0_ = doubleParam("motor0");
    motor1_ = doubleParam("motor1");
    motor2_ = doubleParam("motor2");
    motor3_ = doubleParam("motor3");
    motor4_ = doubleParam("motor4");
    motor5_ = doubleParam("motor5");
    motor6_ = doubleParam("motor6");
    motor7_ = doubleParam("motor7");
    motor8_ = doubleParam("motor8");
    motor9_ = doubleParam("motor9");
    motor10_ = doubleParam("motor10");
    motor11_ = doubleParam("motor11");


    for (auto& m : motorOptions()) m =
      aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
  }
  auto ModelMotorInitialize::executeRT()->int
  {
    if (count() == 1) {
      //--get init motor pos and target motor pos--//
      double pos_commad_[12] = { motor0_, motor1_, motor2_, motor3_, motor4_, motor5_, motor6_, motor7_, motor8_, motor9_, motor10_, motor11_ };

      std::cout << "pos command ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, pos_commad_);

      for (int i = 0; i < 12; i++) {
        pos_[i] = controller()->motorPool()[i].targetPos();
        target_pos_[i] = pos_[i] + pos_commad_[i];
      }

      std::cout << "init pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, pos_);

      std::cout << "target pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, target_pos_);

    }

    //--use fun to plan pos--//
    aris::Size total_count[12]{};

    for (int i = 0; i < 12; i++) {
      ret[i] = moveAbsolute2(pos_[i], pos_d_[i], pos_dd_[i], target_pos_[i], 0.1, 1, 1, 0.5, 0.5, 1e-3, 1e-10, pos_[i], pos_d_[i], pos_dd_[i], total_count[i]);
    }

    //--use ret_all to check ret[12]; if exist 1, it comes 1; if no 1 exist, it keeps 0;--//
    ret_all = 0;
    for (int i = 0; i < 12; i++) {
      if (ret[i] != 0) {
        ret_all = 1; // ֻҪ�����д���ֵ��Ϊ0��Ԫ�أ��ͽ� ret_all ����Ϊ1
        break;       // �ҵ�һ���Ϳ�������ѭ����
      }
    }

    //--set motor pos to planned pos--//
    for (int j = 0; j < 12; j++)
    {
      controller()->motorPool()[j].setTargetPos(pos_[j]);
    }

    //--output motor pos with interval count = 500 --//
    if (count() % 500 == 0)
    {
      for (int i = 0; i < 12; i++) {
        motorPos_[i] = controller()->motorPool()[i].targetPos();
      }
      std::cout << "count= " << count() << "; 12 Motor Pos ===>>>" << std::endl;
      aris::dynamic::dsp(4, 3, motorPos_);
    }

    if (ret_all == 0)
    {
      std::cout << std::endl << "motorPos: " << std::endl;
      std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() << "\t" << controller()->motorPool()[1].actualPos() << "\t" << controller()->motorPool()[2].actualPos() << std::endl;
      std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() << "\t" << controller()->motorPool()[4].actualPos() << "\t" << controller()->motorPool()[5].actualPos() << std::endl;
      std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() << "\t" << controller()->motorPool()[7].actualPos() << "\t" << controller()->motorPool()[8].actualPos() << std::endl;
      std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() << "\t" << controller()->motorPool()[10].actualPos() << "\t" << controller()->motorPool()[11].actualPos() << std::endl;

      double finalPos[12]{ 0 };
      for (int i = 0; i < 12; i++)
      {
        finalPos[i] = controller()->motorPool()[i].actualPos();
      }

      //Solve the Forward Kinematics
      modelBase()->setInputPos(finalPos);

      if (modelBase()->forwardKinematics())
      {
        throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
      }

      double finalPE[28]{ 0 };
      modelBase()->getOutputPos(finalPE);

      std::cout << std::endl << "Final body pose = " << std::endl;
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; ++j)
        {
          std::cout << finalPE[4 * i + j] << "\t";
        }
        std::cout << std::endl;
      }


      std::cout << std::endl << "Final 4-legPos = " << std::endl;
      for (int i = 0; i < 4; i++)
      {
        std::cout << "Leg" << i + 1 << " = ";
        for (int j = 0; j < 3; ++j)
        {
          std::cout << finalPE[16 + 3 * i + j] << "\t";
        }
        std::cout << std::endl;
      }

    }

    return ret_all;
  }
  auto ModelMotorInitialize::collectNrt()->void {}
  ModelMotorInitialize::ModelMotorInitialize(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"tt\">"
      "	<GroupParam>"
      "	<Param name=\"motor0\" default=\"0.0\" abbreviation=\"q\"/>"  // Initialize the end pose of a single leg by giving the initial value of the motor. //
      "	<Param name=\"motor1\" default=\"0.78\" abbreviation=\"a\"/>"  // The given value is in radians. The accuracy in this program is set to four decimal places.//
      "	<Param name=\"motor2\" default=\"-1.57\" abbreviation=\"z\"/>"
      "	<Param name=\"motor3\" default=\"0.0\" abbreviation=\"w\"/>"
      "	<Param name=\"motor4\" default=\"-0.78\" abbreviation=\"s\"/>"
      "	<Param name=\"motor5\" default=\"1.57\" abbreviation=\"x\"/>"
      "	<Param name=\"motor6\" default=\"0.0\" abbreviation=\"e\"/>"
      "	<Param name=\"motor7\" default=\"0.78\" abbreviation=\"d\"/>"
      "	<Param name=\"motor8\" default=\"-1.57\" abbreviation=\"c\"/>"
      "	<Param name=\"motor9\" default=\"0.0\" abbreviation=\"r\"/>"
      "	<Param name=\"motor10\" default=\"-0.78\" abbreviation=\"f\"/>"
      "	<Param name=\"motor11\" default=\"1.57\" abbreviation=\"v\"/>"
      "	</GroupParam>"
      "</Command>");

    // aris::core::fromXmlString(command(),
    //    "<Command name=\"t\">"
    //    "	<GroupParam>"                                    
    //    "	<Param name=\"motor0\" default=\"0.0\" abbreviation=\"q\"/>"  // Initialize the end pose of a single leg by giving the initial value of the motor. //
    //    "	<Param name=\"motor1\" default=\"0.0\" abbreviation=\"a\"/>"  // The given value is in radians. The accuracy in this program is set to four decimal places.//
    //    "	<Param name=\"motor2\" default=\"0.0\" abbreviation=\"z\"/>"   
    //    "	<Param name=\"motor3\" default=\"0.0\" abbreviation=\"w\"/>"   
    //    "	<Param name=\"motor4\" default=\"0.0\" abbreviation=\"s\"/>"   
    //    "	<Param name=\"motor5\" default=\"0.0\" abbreviation=\"x\"/>"   
    //    "	<Param name=\"motor6\" default=\"0.0\" abbreviation=\"e\"/>"   
    //    "	<Param name=\"motor7\" default=\"0.0\" abbreviation=\"d\"/>"   
    //    "	<Param name=\"motor8\" default=\"0.0\" abbreviation=\"c\"/>"   
    //    "	<Param name=\"motor9\" default=\"0.0\" abbreviation=\"r\"/>"   
    //    "	<Param name=\"motor10\" default=\"0.0\" abbreviation=\"f\"/>"   
    //    "	<Param name=\"motor11\" default=\"0.0\" abbreviation=\"v\"/>"   
    //    "	</GroupParam>"
    //    "</Command>");
  }
  ModelMotorInitialize::~ModelMotorInitialize() = default;
/*----------------Set Max Torque--------------------*/
  auto SetMaxTorque::prepareNrt()->void {
    for (auto& m : motorOptions()) {
      m =
        Plan::NOT_CHECK_ENABLE |
        Plan::NOT_CHECK_POS_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
      // Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    }
  }
  auto SetMaxTorque::executeRT()->int
  {
    robot::setAllMotorMaxTorque(ecMaster(), 1000);
    return 0;
  }
  auto SetMaxTorque::collectNrt()->void {}
  SetMaxTorque::SetMaxTorque(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"set_max_trq\">"
      "</Command>");
  }
  SetMaxTorque::~SetMaxTorque() = default;

/*----------------save home-------------------------*/
  auto SaveHome::prepareNrt()->void {

    for (auto& option : motorOptions()) option |= NOT_CHECK_ENABLE | Plan::UPDATE_MODEL_POS_FROM_CONTROLLER;

    option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

    auto model = dynamic_cast<aris::dynamic::MultiModel*>(&aris::server::ControlServer::instance().model());


    try {

        for(aris::Size i = 0; i < controller()->motorPool().size(); i++)
        {                
            auto &cm = controller()->motorPool()[i];    
            if((cm.statusWord() & 0x6f) != 0x27){}
            else
            {
                THROW_FILE_LINE("");
                break;
            }           
        }
        std::vector<double>offset;
        offset.resize(controller()->motorPool().size(), 0.0);
        for (aris::Size i = 0; i < offset.size(); i++)
        {
            offset[i] = controller()->motorPool()[i].posOffset();
            controller()->motorPool()[i].setPosOffset(controller()->motorPool()[i].actualPos() + offset[i]);
        }

        auto xmlpath = std::filesystem::absolute(".");

        const std::string xmlfile = "dog.xml";

        xmlpath = xmlpath / xmlfile;

        aris::core::toXmlFile(*controlServer(), xmlpath);

  #ifdef UNIX
          sync();
  #endif // UNIX

    }
    catch (std::exception&) {
        throw std::runtime_error("保存零位失败，请确保机械臂处于去使能状态");
    }

    for (aris::Size i = 0; i < controller()->motorPool().size(); ++i)
        controller()->motorPool()[i].setTargetPos(controller()->motorPool().at(i).actualPos());

    controlServer()->updateDataController2Model(motorOptions());


    double input[12];
    model->getInputPos(input);
    aris::dynamic::dsp(4,3,input);

    model->forwardKinematics();

  }
  SaveHome::SaveHome(const std::string& name) {

      aris::core::fromXmlString(command(),
          "<Command name=\"savehome\">"
          "</Command>");
  }
/*----------------back to home at axis space--------*/
  struct DogHome::Imp
  {
      aris::dynamic::MultiModel* mm{ nullptr };
      double begin_angle[12] = { 0 };
      double angle[12] = { 0 };
  };
  auto DogHome::prepareNrt()->void
  {
      for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE | 
                                      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | 
                                      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS;
  }
  auto DogHome::executeRT()->int
  {
      if (count() == 1)
      {
          this->master()->logFileRawName("home");
          for (size_t i = 0; i < 12; i++)
          {
              imp_->begin_angle[i] = controller()->motorPool()[i].actualPos();
          }
      }


      Tcurve s1(0.5, 0.2);//5.4s
      s1.getCurveParam();

      for(int i=0;i< 12;++i)
      {
          imp_->angle[i] = imp_->begin_angle[i] + (0- imp_->begin_angle[i]) * s1.getCurve(count());
      }

      for(int i=0;i< 12;++i)
      {
          controller()->motorPool()[i].setTargetPos(imp_->angle[i]);
      }
      return s1.getTc() * 1000-count()-1;
  }
  DogHome::DogHome(const std::string &name) : imp_(new Imp)
  {
      aris::core::fromXmlString(command(),
        "<Command name=\"dog_home\"/>");
  }
  DogHome::~DogHome() = default;
  ARIS_DEFINE_BIG_FOUR_CPP(DogHome);

/*----------------single leg test in kaanh shanghai-*/
  auto LegTest::prepareNrt()->void
  {
    leg_id_ = int32Param("leg_id");
    move_x_ = doubleParam("move_x");
    move_h_ = doubleParam("move_h");
    move_z_ = doubleParam("move_z");
    show_period_ = int32Param("show_period");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
      aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS ;
    // this->controlServer()->idleMotionCheckOption()[0];
  }
  auto LegTest::executeRT()->int
  {
    for(size_t i = 0; i < 3; i++){
      test_leg_me_id_[i] = (leg_id_ - 1) * 3 + i;
    }
    //read  init motor param and rbt info//
    if(count() == 1){
      for(size_t i = 0; i < 12; i++) {
        auto& cm = controller()->motorPool()[i];
        pos_init_[i] = cm.actualPos();
        vel_init_[i] = cm.actualVel();
        toq_init_[i] = cm.actualToq();
        // kStop_ = 1;
      }

      flag_mode_set_ = 0;

      modelBase()->setInputPos(pos_init_);
      if (modelBase()->forwardKinematics()) {THROW_FILE_LINE("Forward kinematics failed 1");}
      modelBase()->getOutputPos(BppFee_init_);
      modelBase()->getOutputPos(BppFee_run_);
    }

    //generate planned Fee//
    Tcurve tcurve(0.5,0.2);
    double theta = PI * (1 - tcurve.getCurve(count()));
    BppFee_run_[ 16 + test_leg_me_id_[0]] = BppFee_init_[ 16 + test_leg_me_id_[0]] + (1 + std::cos(theta)) * 0.5 * move_x_;
    BppFee_run_[ 16 + test_leg_me_id_[1]] = BppFee_init_[ 16 + test_leg_me_id_[1]] + std::sin(theta) * move_h_;
    BppFee_run_[ 16 + test_leg_me_id_[2]] = BppFee_init_[ 16 + test_leg_me_id_[2]] + (1 + std::cos(theta)) * 0.5 * move_z_;
    
    kStop_ = tcurve.getTc() * 1000 - count() - 1;

    //use invKin get target motor pos//
    modelBase()->setOutputPos(BppFee_run_);
    if (modelBase()->inverseKinematics()) {THROW_FILE_LINE("Inverse kinematics failed 2");}
    modelBase()->getInputPos(pos_run_);

    //print//
    if(count() % show_period_ == 0){
      mout() << "T--->" << count() << std::endl;
      splitMatrix28(BppFee_run_, Bpp_run_, Fee_run_);
      mout() << "Bpp->"  << std::endl;
      dsp(4, 4, Bpp_run_);
      mout() << "Fee->"  << std::endl;
      dsp(4, 3, Fee_run_);
      mout() << "Mpos->"  << std::endl;
      dsp(4, 3, pos_run_);
    }

    //---Torque mode set---//
    if(flag_mode_set_ == 0){
      for(size_t i = 0; i < 3; i++){
        if (setOperationMode(controller(), OperationMode::kTorqueMode, test_leg_me_id_[i])) {
          mout() << "set mode success" << std::endl;
          flag_mode_set_ = 1;
        }else
        return 1;
      }
    }

    for(size_t i = 0; i < 3; i++){
      double vel[3]{};
      double vel_des[3]{};
      double pos[3]{};
      double toq_cmd[3]{};
      auto& cm = controller()->motorPool()[test_leg_me_id_[i]];


      vel[i] = cm.actualVel();
      pos[i] = cm.actualPos();

      // toq_cmd[test_leg_me_id_[i]] = PdPosController.getTargetOut(2.5, pos_run_[i], pos[i], 0, vel[i]);
      vel_des[i] = PosLoopController.getTargetOut(pos_run_[test_leg_me_id_[i]], pos[i]);
      toq_cmd[i] = VelLoopController.getTargetOut(vel_des[i], vel[i]);

      cm.setTargetToq(toq_cmd[i] * ktoqFactor);
      cm.setTargetPos(pos_run_[test_leg_me_id_[i]]);
      cm.setTargetVel(vel_des[i]);
    }

    if(kStop_ == 0){
      for(size_t i = 0; i < 3; i++){
      auto& cm = controller()->motorPool()[test_leg_me_id_[i]];
      cm.setModeOfOperation(8);
      cm.setTargetPos(cm.actualPos());
      aris::server::ControlServer::instance().errorChecker().storeServerData();
      }
    }

    
    /*
    //----execute target motor pos----//
    for(size_t i = 0; i < 12; i++){
      auto& cm = controller()->motorPool()[i];
      cm.setTargetPos(pos_run_[i]);
    }
    */

    return kStop_;
  }
  auto LegTest::collectNrt()->void {}
  LegTest::LegTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"t6\">"
      "	<GroupParam>"
      "	<Param name=\"move_x\" default=\"0.1\" abbreviation=\"x\"/>"
      "	<Param name=\"move_h\" default=\"0.05\" abbreviation=\"h\"/>"
      "	<Param name=\"move_z\" default=\"0.1\" abbreviation=\"z\"/>"
      "	<Param name=\"leg_id\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"show_period\" default=\"50\" abbreviation=\"n\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  LegTest::~LegTest() = default;


/*----------------single leg step test in kaanh shanghai 2.22.2024-*/
  auto LegStepTest::prepareNrt()->void
  {
    leg_id_ = int32Param("leg_id");
    move_x_ = doubleParam("move_x");
    move_h_ = doubleParam("move_h");
    move_z_ = doubleParam("move_z");
    tcurve_a_ = doubleParam("ta");
    tcurve_v_ = doubleParam("tv");
    step_count_ = int32Param("step_count");
    show_period_ = int32Param("show_period");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
      aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS ;
    // this->controlServer()->idleMotionCheckOption()[0];
  }
  auto LegStepTest::executeRT()->int
  {
    for(size_t i = 0; i < 3; i++){
      test_leg_me_id_[i] = (leg_id_ - 1) * 3 + i;
    }
    //read  init motor param and rbt info//
    if(count() == 1){
      for(size_t i = 0; i < 12; i++) {
        auto& cm = controller()->motorPool()[i];
        pos_init_[i] = cm.actualPos();
        vel_init_[i] = cm.actualVel();
        toq_init_[i] = cm.actualToq();
        // kStop_ = 1;
      }

      flag_mode_set_ = 0;
      flag_step_count_  = step_count_;
      pn_sym_ = 1;

      modelBase()->setInputPos(pos_init_);
      if (modelBase()->forwardKinematics()) {THROW_FILE_LINE("Forward kinematics failed 1");}
      modelBase()->getOutputPos(BppFee_init_);
      modelBase()->getOutputPos(BppFee_run_);
    }

    //generate planned Fee//
    Tcurve tcurve(tcurve_a_,tcurve_v_);
    int t_period = tcurve.getTc() * 1000;

    if(flag_step_count_ != 0){
      int count_period = count() % t_period;
      if(count_period == 0){
        pn_sym_ = -pn_sym_;
        flag_step_count_ -= 1;
        zero_tf_ = 0;
        std::cout << "------end of period--------" << std::endl;
      }else{zero_tf_ = 1;}

      if(pn_sym_ == 1){
        theta_ = PI * (1 - tcurve.getCurve(count_period)) * zero_tf_;
      }else{
        theta_ = PI * tcurve.getCurve(count_period) * zero_tf_;
      }

      BppFee_run_[ 16 + test_leg_me_id_[0]] = BppFee_init_[ 16 + test_leg_me_id_[0]] + (1 + std::cos(theta_)) * 0.5 * move_x_;
      BppFee_run_[ 16 + test_leg_me_id_[1]] = BppFee_init_[ 16 + test_leg_me_id_[1]] + std::sin(theta_) * move_h_;
      BppFee_run_[ 16 + test_leg_me_id_[2]] = BppFee_init_[ 16 + test_leg_me_id_[2]] + (1 + std::cos(theta_)) * 0.5 * move_z_;
    }
    
    kStop_ = tcurve.getTc() * 1000 * step_count_ - count() - 1;

    //use invKin get target motor pos//
    modelBase()->setOutputPos(BppFee_run_);
    if (modelBase()->inverseKinematics()) {THROW_FILE_LINE("Inverse kinematics failed 2");}
    modelBase()->getInputPos(pos_run_);

    //print//
    if(count() % show_period_ == 0){
      mout() << "T--->" << count() << std::endl;
      splitMatrix28(BppFee_run_, Bpp_run_, Fee_run_);
      mout() << "Bpp->"  << std::endl;
      dsp(4, 4, Bpp_run_);
      mout() << "Fee->"  << std::endl;
      dsp(4, 3, Fee_run_);
      mout() << "Mpos->"  << std::endl;
      dsp(4, 3, pos_run_);
      mout() << "pn_sym-> "  << pn_sym_ << std::endl;
    } 

    //---Torque mode set---//
    if(flag_mode_set_ == 0){
      for(size_t i = 0; i < 3; i++){
        if (setOperationMode(controller(), OperationMode::kTorqueMode, test_leg_me_id_[i])) {
          mout() << "set mode success" << std::endl;
          flag_mode_set_ = 1;
        }else
        return 1;
      }
    }

    for(size_t i = 0; i < 3; i++){
      double vel[3]{};
      double vel_des[3]{};
      double pos[3]{};
      double toq_cmd[3]{};
      double toq[3]{};
      auto& cm = controller()->motorPool()[test_leg_me_id_[i]];


      vel[i] = cm.actualVel();
      pos[i] = cm.actualPos();
      toq[i] = cm.actualToq() / ktoqFactor;

      // toq_cmd[test_leg_me_id_[i]] = PdPosController.getTargetOut(2.5, pos_run_[i], pos[i], 0, vel[i]);
      vel_des[i] = PosLoopController.getTargetOut(pos_run_[test_leg_me_id_[i]], pos[i]);
      toq_cmd[i] = VelLoopController.getTargetOut(vel_des[i], vel[i]);

      cm.setTargetToq(toq_cmd[i] * ktoqFactor);
      cm.setTargetPos(pos_run_[test_leg_me_id_[i]]);
      // cm.setTargetVel(vel_des[i]);

      if(i == 2){
        mout() << "T---> " << count() << std::endl
          << "cmd toq: " << toq_cmd[0] << "\t" << toq_cmd[1] << "\t" << toq_cmd[2] << std::endl
          << "actual toq: " << toq[0] << "\t" << toq[1] << "\t" << toq[2] << std::endl
          << "actual pos: " << pos[0] << "\t" << pos[1] << "\t" << pos[2] << std::endl    
          << "actual vel: " << vel[0] << "\t" << vel[1] << "\t" << vel[2] << std::endl << std::endl;      
      }
    }

    if(kStop_ == 0){
      for(size_t i = 0; i < 3; i++){
      auto& cm = controller()->motorPool()[test_leg_me_id_[i]];
      cm.setModeOfOperation(8);
      cm.setTargetPos(cm.actualPos());
      aris::server::ControlServer::instance().errorChecker().storeServerData();
      }
    }

    
    /*
    //----execute target motor pos----//
    for(size_t i = 0; i < 12; i++){
      auto& cm = controller()->motorPool()[i];
      cm.setTargetPos(pos_run_[i]);
    }
    */

    return kStop_;
  }
  auto LegStepTest::collectNrt()->void {}
  LegStepTest::LegStepTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"t6\">"
      "	<GroupParam>"
      "	<Param name=\"move_x\" default=\"0.1\" abbreviation=\"x\"/>"
      "	<Param name=\"move_h\" default=\"0.05\" abbreviation=\"h\"/>"
      "	<Param name=\"move_z\" default=\"0.1\" abbreviation=\"z\"/>"
      "	<Param name=\"leg_id\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"show_period\" default=\"50\" abbreviation=\"n\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  LegStepTest::~LegStepTest() = default;
/*----------------run all the motors in toq mode to keep pos standing-*/
  auto AllToqTest::prepareNrt()->void
  {
    auto& cs = aris::server::ControlServer::instance();
    kp_ = doubleParam("kp");
    kd_ = doubleParam("kd");
    run_time_ = int32Param("run_time");
    test_motor_id_ = int32Param("test_motor_id");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
      aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |
      aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS ;
    
    for(std::size_t i = 0; i < controller()->motorPool().size(); ++i){
        cs.idleMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
    }
    // this->controlServer()->idleMotionCheckOption()[0];
  }
  auto AllToqTest::executeRT()->int
  {
    if(count() == 1){
      PdPosController.setPdParam(kp_, kd_);
    }
    for(size_t i = 0; i < 12; i++){

      auto& cm = controller()->motorPool()[i] ;

      pos[i] = cm.actualPos();
      vel[i] = cm.actualVel();
      toq[i] = cm.actualToq() / ktoqFactor;

      if(flag_start[i] == 0){
          cm.setModeOfOperation(10);
          pos_init[i] = cm.actualPos();
          vel_init[i] = cm.actualVel();
          cm.setTargetVel(cm.actualVel());
          cm.setTargetPos(cm.actualPos());
          flag_start[i] = 1;
          std::cout << "motor " << i << "start -> " << std::endl;
      }

      toq_cmd[i] = PdPosController.getTargetOut(0, pos_init[i], pos[i], 0, vel[i]);
      // pos_last[i] = pos[i];
      // vel_last[i] = vel[i];
      cm.setTargetToq(toq_cmd[i] * ktoqFactor);
    }

    if(count() % 100 == 0){
      mout() << "T---> " << count() << std::endl
           << "pos_now: " << pos[test_motor_id_] << "\t" << "pos_last: " << pos_init[test_motor_id_] << std::endl
           << "vel_now: " << vel[test_motor_id_] << "\t" << "vel_last: " << vel_init[test_motor_id_] << std::endl
           << "toq_now: " << toq[test_motor_id_] << "\t" << "toq_cmd: " << toq_cmd[test_motor_id_] << std::endl << std::endl;
    }
    
    int kStop = 1000 * run_time_ - count();
    return kStop;
  }
  auto AllToqTest::collectNrt()->void {}
  AllToqTest::AllToqTest(const std::string& name)
  {
    aris::core::fromXmlString(command(),
      "<Command name=\"t6\">"
      "	<GroupParam>"
      "	<Param name=\"move_x\" default=\"0.1\" abbreviation=\"x\"/>"
      "	<Param name=\"move_h\" default=\"0.05\" abbreviation=\"h\"/>"
      "	<Param name=\"move_z\" default=\"0.1\" abbreviation=\"z\"/>"
      "	<Param name=\"leg_id\" default=\"1\" abbreviation=\"i\"/>"
      "	<Param name=\"show_period\" default=\"50\" abbreviation=\"n\"/>"
      "	</GroupParam>"
      "</Command>");
  }
  AllToqTest::~AllToqTest() = default;
/*----------------ARIS_REGISTRATION-----------------*/
    ARIS_REGISTRATION{
      aris::core::class_<SaveHome>("SaveHome").inherit<aris::plan::Plan>();
      aris::core::class_<DogHome>("DogHome").inherit<aris::plan::Plan>();
      aris::core::class_<robot::ReadInformation>("ReadInformation").inherit<aris::plan::Plan>();
      aris::core::class_<robot::ModelMotorInitialize>("ModelMotorInitialize").inherit<aris::plan::Plan>();
      aris::core::class_<robot::MotorTest>("MotorTest").inherit<aris::plan::Plan>();
      aris::core::class_<robot::SetMotorPosZero>("SetMotorPosZero").inherit<aris::plan::Plan>();
      aris::core::class_<robot::SetMaxTorque>("SetMaxTorque").inherit<aris::plan::Plan>();
      aris::core::class_<robot::LegTest>("LegTest").inherit<aris::plan::Plan>();
      aris::core::class_<robot::LegStepTest>("LegStepTest").inherit<aris::plan::Plan>();
      aris::core::class_<robot::AllToqTest>("AllToqTest").inherit<aris::plan::Plan>();
    }
}
