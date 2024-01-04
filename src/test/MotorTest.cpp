#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include "control/robot.h"
#include "control/plan.h"
#include "server/server.h"
#include "model/model.h"
#include "tools/operator.h"
#include "test/MotorTest.h"

namespace robot {
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


    model()->setInputPos(motorPos);
    model()->getInputPos(modelPosArray);
    std::cout << std::endl << "Model Pos ===>>> " << std::endl;
    aris::dynamic::dsp(4, 3, modelPosArray);




    //---use fwdKin to get ModelPose; storage container uses std::vector---//
    modelPoseVec.resize(28);
    if (model()->solverPool()[1].kinPos())
    {
      throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
    }
    model()->getOutputPos(modelPoseVec.data());

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
        ret_all = 1; // 只要数组中存在值不为0的元素，就将 ret_all 设置为1
        break;       // 找到一个就可以跳出循环了
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
      model()->setInputPos(finalPos);

      if (model()->solverPool()[1].kinPos())
      {
        throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
      }

      double finalPE[28]{ 0 };
      model()->getOutputPos(finalPE);

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
        ret_all = 1; // 只要数组中存在值不为0的元素，就将 ret_all 设置为1
        break;       // 找到一个就可以跳出循环了
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
      model()->setInputPos(finalPos);

      if (model()->solverPool()[1].kinPos())
      {
        throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
      }

      double finalPE[28]{ 0 };
      model()->getOutputPos(finalPE);

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
        ret_all = 1; // 只要数组中存在值不为0的元素，就将 ret_all 设置为1
        break;       // 找到一个就可以跳出循环了
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
      model()->setInputPos(finalPos);

      if (model()->solverPool()[1].kinPos())
      {
        throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
      }

      double finalPE[28]{ 0 };
      model()->getOutputPos(finalPE);

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

  //ARIS_REGISTRATION{
  //  aris::core::class_<SetMaxTorque>("SetMaxTorque")
  //  .inherit<Plan>();
  //  aris::core::class_<ReadInformation>("ReadInformation")
  //  .inherit<Plan>();
  //  aris::core::class_<ModelMotorInitialize>("ModelMotorInitialize")
  //  .inherit<Plan>();
  //  aris::core::class_<MotorTest>("MotorTest")
  //  .inherit<Plan>();
  //  aris::core::class_<SetMotorPosZero>("SetMotorPosZero")
  //  .inherit<Plan>();
  //}
}