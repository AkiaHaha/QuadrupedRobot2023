#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include "robot.h"
#include "plan.h"
#include "model.h"
#include "operator.h"
using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;

namespace robot {
  auto TrotMove::prepareNrt()->void
  {
    vel_x = doubleParam("vel_x");
    vel_z = doubleParam("vel_z");
    vel_h = doubleParam("vel_h");

    for (auto& m : motorOptions()) m =
      aris::plan::Plan::CHECK_NONE;
  }
  auto TrotMove::executeRT()->int
  {
    if (count() == 1) {
      double current_motor_pos[12]{};
      for (int8_t i = 0; i < 12; i++) {
        current_motor_pos[i] = controller()->motorPool()[i].targetPos();
      }
      model()->setInputPos(current_motor_pos);
      model()->forwardKinematics();
      model()->getOutputPos(init_m28);
    }

    period_n = count() / kTcurvePeriodCount + 1;
    time_in_pn = count() % kTcurvePeriodCount;
    switch_number = period_n % 2 == 1 ? false : true;

    EllipseMovePlan ep(vel_x, vel_z, vel_h, switch_number, init_m28);
    move_m28 = ep.getCurrentM28(time_in_pn);

    model()->setOutputPos(move_m28);
    model()->inverseKinematics();
    model()->getInputPos(move_motor_pos);

    for (int8_t i = 0; i < 12; ++i) {
      controller()->motorPool()[i].setTargetPos(move_motor_pos[i]);
    }
    
    std::cout << "---motor pos---" << std::endl;
    show(4, 3, move_motor_pos);

    std::cout << "---mb and pee---" << std::endl;
    std::copy(move_m28, move_m28 + 16, move_mb);
    std::copy(move_m28 + 17, move_m28 + 28, move_pee);
    show(4, 4, move_mb);
    show(4, 3, move_pee);
    
    return count() - kTcurvePeriodCount;
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
      "	</GroupParam>"
      "</Command>");
  }
  TrotMove::~TrotMove() = default;

auto Ellipse4LegDrive3::prepareNrt()->void
{
    moveX_ = doubleParam("moveX");
    moveY_ = doubleParam("moveY");
    moveZ_ = doubleParam("moveZ");
    Height_ = doubleParam("Height");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |        
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Ellipse4LegDrive3::executeRT()->int
{
    if (count() == 1)
    {
        //---initializa theta_---//
        theta_ = PI;

        //---read init motor pos and stored in  startMotorPos---//
        for (int i = 0; i< 12; i++){
            startMotorPos[i] = controller()-> motorPool()[i].targetPos();
        }
        std::cout << "start motor pos ===>>>" << std::endl;
        aris::dynamic::dsp(4, 3, startMotorPos);

        //---use fwdKin with startMotorPos to get startModelPE---//
        mout() << "|||Test flag ===>>> 3" << std::endl;

        model()->setInputPos(startMotorPos);
        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Initialization Forward Kinematics Position Failed!");
        }   
        model()->getOutputPos(startModelPE);   

        //---display startModelPE ---//
        for (int i = 0; i < 16; i ++){
            startBodyPose[i] = startModelPE[i];
        }         
        for (int i = 0; i < 12; i ++){
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
    auto ret = moveAbsolute2( theta_, theta_d_, theta_dd_, 0, 5, 10, 5, 10, 5, 1e-3, 1e-10, theta_, theta_d_, theta_dd_, total_count);

    // std::cout << "To get moveModelPE ~" << std::endl;
    //---use Ellipse plan function to get trajectory point list---//
    e7.getMoveModelPE(theta_, moveModelPE);

    //---display moveModelPE with corresponding count and theta---//
    std::cout << "count = " << count() << "\t" << "theta = " << theta_ << "\t moveModelPE ===>>>  "  <<std::endl;


    //---display moveModelPE ---//
    for (int i = 0; i < 16; i ++){
        moveBodyPose[i] = moveModelPE[i];
    }         
    for (int i = 0; i < 12; i ++){
        moveLegPoint[i] = moveModelPE[i + 16];
    }

    std::cout << std::endl << "Move body pose ===>>> " << std::endl;       
    aris::dynamic::dsp(4, 4, moveBodyPose);

    std::cout << std::endl << "Move 4-legPoint = " << std::endl;
    aris::dynamic::dsp(4, 3, moveLegPoint);

    //---use planned target moveModelPose to get target moveMotorPos---//
    model()->setOutputPos(moveModelPE);
    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed wawawaw");    
    }

    std::cout << std::endl << "move status invKin at count = " << count() << " successed ~" << std::endl;

    //---copy motorPos inverse solved from modelPE to moveMotorPos(12)
    for (int i = 0; i < 12; i++){
        moveMotorPos[i] = model()->motionPool()[i].mp();
        controller()->motorPool()[i].setTargetPos(moveMotorPos[i]);
    }

    if ( ret == 0 )
    {
        //--Output motor pos--//
        std::cout << "The Ellipse Curve has Run Successfully~" << std::endl << std::endl;
        std::cout <<std::endl<< "motorPos===>>> " << std::endl;
        aris::dynamic::dsp(4, 3, moveMotorPos);


        //--Solve the Forward Kinematics--//
        model()->setInputPos(moveMotorPos) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
        }
        model()->getOutputPos(finalModelPE);
    
        //--Output body pose and leg point--//        
        for (int i = 0; i < 16; i ++){
            finalBodyPose[i] = finalModelPE[i];
        }         
        for (int i = 0; i < 12; i ++){
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
Ellipse4LegDrive3::Ellipse4LegDrive3(const std::string &name) 
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

auto ReadInformation::prepareNrt()->void
{
    for(auto &m:motorOptions()) m =
            aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
}
auto ReadInformation::executeRT()->int
{
    //---read all motor's current pos aris-Matrix---//
    // std::cout << "Motor Pos ===>>> " << std::endl;

    for (int i = 0; i < 12; i++){
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

    std::cout << std::endl << "Body Pose ===>>> " <<std::endl;
    aris::dynamic::dsp(4, 4, body_pose);
    std::cout << std::endl << "Leg Point ===>>>" << std::endl;
    aris::dynamic::dsp(4, 3, leg_point);
    
    return 0;
}
auto ReadInformation::collectNrt()->void {}
ReadInformation::ReadInformation(const std::string &name) 
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


    for(auto &m:motorOptions()) m =
            aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
}
auto MotorTest::executeRT()->int
{
    if (count() == 1){
        //--get init motor pos and target motor pos--//
        double pos_commad_[12] = { motor0_, motor1_, motor2_, motor3_, motor4_, motor5_, motor6_, motor7_, motor8_, motor9_, motor10_, motor11_ };

        std::cout << "pos command ===>>>" << std::endl;
        aris::dynamic::dsp(4, 3, pos_commad_);
    
        for (int i = 0; i < 12; i++){
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

    for (int i =0; i < 12; i++){
        ret[i] = moveAbsolute2( pos_[i], pos_d_[i], pos_dd_[i], target_pos_[i], 0.1, 1, 1, 0.5, 0.5, 1e-3, 1e-10, pos_[i], pos_d_[i], pos_dd_[i], total_count[i]);        
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
    for (int j=0; j<12; j++)
    {
        controller()->motorPool()[j].setTargetPos(pos_[j]); 
    }

    //--output motor pos with interval count = 500 --//
    if (count() % 500 == 0)
    {
        for (int i = 0; i < 12; i++){
        motorPos_[i] = controller()->motorPool()[i].targetPos();
        } 
        std::cout << "count= " << count() << "; 12 Motor Pos ===>>>"  <<std::endl;
        aris::dynamic::dsp(4, 3, motorPos_);
    }

    if ( ret_all == 0 )
    {
        std::cout <<std::endl<< "motorPos: " << std::endl;
        std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() <<"\t"<< controller()->motorPool()[1].actualPos()<<"\t"<< controller()->motorPool()[2].actualPos() << std::endl;
        std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() <<"\t"<< controller()->motorPool()[4].actualPos()<<"\t"<< controller()->motorPool()[5].actualPos() << std::endl;
        std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() <<"\t"<< controller()->motorPool()[7].actualPos()<<"\t"<< controller()->motorPool()[8].actualPos() << std::endl;
        std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() <<"\t"<< controller()->motorPool()[10].actualPos()<<"\t"<< controller()->motorPool()[11].actualPos() << std::endl;
    
        double finalPos[12]{0};
        for (int i = 0; i < 12; i++)
        {
            finalPos[i] = controller()->motorPool()[i].actualPos();
        }

        //Solve the Forward Kinematics
        model()->setInputPos(finalPos) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
        }
       
        double finalPE[28]{0};
        model()->getOutputPos(finalPE);
    
        std::cout << std::endl << "Final body pose = " << std::endl;
        for (int i = 0; i < 4; i++ )
        {
            for (int j =  0; j < 4; ++j)
            {
                std::cout << finalPE[4 * i + j] << "\t";
            }
            std::cout << std::endl;
        }

    
        std::cout << std::endl << "Final 4-legPos = " << std::endl;
        for (int i = 0; i < 4; i++ )
        {
            std::cout << "Leg" << i + 1 << " = ";
            for (int j =  0; j < 3; ++j)
            {
                std::cout << finalPE[16 + 3 * i + j] << "\t";
            }
            std::cout << std::endl;
        }        
    
    }

    return ret_all;
}
auto MotorTest::collectNrt()->void {}
MotorTest::MotorTest(const std::string &name) 
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
    for(auto &m:motorOptions()) m =
            aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
}
auto SetMotorPosZero::executeRT()->int
{
    if (count() == 1){
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

    for (int i =0; i < 12; i++){
        ret[i] = moveAbsolute2( pos_[i], pos_d_[i], pos_dd_[i], 0, 1, 0.1, 1, 0.5, 0.5, 1e-3, 1e-10, pos_[i], pos_d_[i], pos_dd_[i], total_count[i]);        
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
    for (int j=0; j<12; j++)
    {
        controller()->motorPool()[j].setTargetPos(pos_[j]); 
    }

    //--output motor pos with interval count = 500 --//
    if (count() % 500 == 0)
    {
        for (int i = 0; i < 12; i++){
        motorPos_[i] = controller()->motorPool()[i].targetPos();
        } 
        std::cout << "count= " << count() << "; 12 Motor Pos ===>>>"  <<std::endl;
        aris::dynamic::dsp(4, 3, motorPos_);
    }

    if ( ret_all == 0 )
    {
        std::cout <<std::endl<< "motorPos: " << std::endl;
        std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() <<"\t"<< controller()->motorPool()[1].actualPos()<<"\t"<< controller()->motorPool()[2].actualPos() << std::endl;
        std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() <<"\t"<< controller()->motorPool()[4].actualPos()<<"\t"<< controller()->motorPool()[5].actualPos() << std::endl;
        std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() <<"\t"<< controller()->motorPool()[7].actualPos()<<"\t"<< controller()->motorPool()[8].actualPos() << std::endl;
        std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() <<"\t"<< controller()->motorPool()[10].actualPos()<<"\t"<< controller()->motorPool()[11].actualPos() << std::endl;
    
        double finalPos[12]{0};
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
       
        double finalPE[28]{0};
        model()->getOutputPos(finalPE);
    
        std::cout << std::endl << "Final body pose = " << std::endl;
        for (int i = 0; i < 4; i++ )
        {
            for (int j =  0; j < 4; ++j)
            {
                std::cout << finalPE[4 * i + j] << "\t";
            }
            std::cout << std::endl;
        }

    
        std::cout << std::endl << "Final 4-legPos = " << std::endl;
        for (int i = 0; i < 4; i++ )
        {
            std::cout << "Leg" << i + 1 << " = ";
            for (int j =  0; j < 3; ++j)
            {
                std::cout << finalPE[16 + 3 * i + j] << "\t";
            }
            std::cout << std::endl;
        }        
    
    }

    return ret_all;
}
auto SetMotorPosZero::collectNrt()->void {}
SetMotorPosZero::SetMotorPosZero(const std::string &name) 
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


    for(auto &m:motorOptions()) m =
            aris::plan::Plan::CHECK_NONE;

    this->controlServer()->idleMotionCheckOption()[0];
}
auto ModelMotorInitialize::executeRT()->int
{
    if (count() == 1){
        //--get init motor pos and target motor pos--//
        double pos_commad_[12] = { motor0_, motor1_, motor2_, motor3_, motor4_, motor5_, motor6_, motor7_, motor8_, motor9_, motor10_, motor11_ };

        std::cout << "pos command ===>>>" << std::endl;
        aris::dynamic::dsp(4, 3, pos_commad_);
    
        for (int i = 0; i < 12; i++){
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

    for (int i =0; i < 12; i++){
        ret[i] = moveAbsolute2( pos_[i], pos_d_[i], pos_dd_[i], target_pos_[i], 0.1, 1, 1, 0.5, 0.5, 1e-3, 1e-10, pos_[i], pos_d_[i], pos_dd_[i], total_count[i]);        
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
    for (int j=0; j<12; j++)
    {
        controller()->motorPool()[j].setTargetPos(pos_[j]); 
    }

    //--output motor pos with interval count = 500 --//
    if (count() % 500 == 0)
    {
        for (int i = 0; i < 12; i++){
        motorPos_[i] = controller()->motorPool()[i].targetPos();
        } 
        std::cout << "count= " << count() << "; 12 Motor Pos ===>>>"  <<std::endl;
        aris::dynamic::dsp(4, 3, motorPos_);
    }

    if ( ret_all == 0 )
    {
        std::cout <<std::endl<< "motorPos: " << std::endl;
        std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() <<"\t"<< controller()->motorPool()[1].actualPos()<<"\t"<< controller()->motorPool()[2].actualPos() << std::endl;
        std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() <<"\t"<< controller()->motorPool()[4].actualPos()<<"\t"<< controller()->motorPool()[5].actualPos() << std::endl;
        std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() <<"\t"<< controller()->motorPool()[7].actualPos()<<"\t"<< controller()->motorPool()[8].actualPos() << std::endl;
        std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() <<"\t"<< controller()->motorPool()[10].actualPos()<<"\t"<< controller()->motorPool()[11].actualPos() << std::endl;
    
        double finalPos[12]{0};
        for (int i = 0; i < 12; i++)
        {
            finalPos[i] = controller()->motorPool()[i].actualPos();
        }

        //Solve the Forward Kinematics
        model()->setInputPos(finalPos) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Quadrupd Leg Forward Kinematics Failed!");
        }
       
        double finalPE[28]{0};
        model()->getOutputPos(finalPE);
    
        std::cout << std::endl << "Final body pose = " << std::endl;
        for (int i = 0; i < 4; i++ )
        {
            for (int j =  0; j < 4; ++j)
            {
                std::cout << finalPE[4 * i + j] << "\t";
            }
            std::cout << std::endl;
        }

    
        std::cout << std::endl << "Final 4-legPos = " << std::endl;
        for (int i = 0; i < 4; i++ )
        {
            std::cout << "Leg" << i + 1 << " = ";
            for (int j =  0; j < 3; ++j)
            {
                std::cout << finalPE[16 + 3 * i + j] << "\t";
            }
            std::cout << std::endl;
        }        
    
    }

    return ret_all;
}
auto ModelMotorInitialize::collectNrt()->void {}
ModelMotorInitialize::ModelMotorInitialize(const std::string &name) 
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

auto SetMaxTorque::prepareNrt()->void{
    for(auto &m:motorOptions()){ 
        m =
        Plan::NOT_CHECK_ENABLE |
        Plan::NOT_CHECK_POS_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        // Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
    }
}
auto SetMaxTorque::executeRT()->int
{
    setAllMotorMaxTorque(ecMaster(), 1000);

    return 0;
}
auto SetMaxTorque::collectNrt()->void {}
SetMaxTorque::SetMaxTorque(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"set_max_trq\">"
       "</Command>");
}
SetMaxTorque::~SetMaxTorque() = default; 

auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>{
    std::unique_ptr<aris::control::Master> master(new aris::control::EthercatMaster);

    for (aris::Size i = 0; i < 12 ; ++i){
        int phy_id[12]={8,9,10,5,3,4,6,7,11,0,1,2};
//----------------------------------Daniel for Single_leg Servo Motor Success in 2023.10.14--------------------------------------//
        // std::string xml_str =
        //    "<EthercatSlave phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
        //    " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
        //    ">"
        //    "  <SyncManagerPoolObject>"
        //    "		<SyncManager is_tx=\"false\"/>"
        //    "		<SyncManager is_tx=\"true\"/>"
        //    "		<SyncManager is_tx=\"false\">"
        //    "			<Pdo index=\"0x1605\" is_tx=\"false\">"
        //    "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
        //    "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
        //    "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
        //    "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
        //    "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        //    "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
        //    "			</Pdo>"
        //    "		</SyncManager>"
        //    "		<SyncManager is_tx=\"true\">"
        //    "			<Pdo index=\"0x1A07\" is_tx=\"true\">"
        //    "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        //    "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
        //    "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        //    "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
        //    "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
        //    "			</Pdo>"
        //    "		</SyncManager>"
        //    "  </SyncManagerPoolObject>"
        //    "</EthercatSlave>";

//---------------------------------------XML for New Quadruped Robot of Kaanh 11.28------------------------------------------//
        std::string xml_str =
           "<EthercatSlave phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
           " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
           ">"
           "  <SyncManagerPoolObject>"
           "		<SyncManager is_tx=\"false\"/>"
           "		<SyncManager is_tx=\"true\"/>"
           "		<SyncManager is_tx=\"false\">"
           "			<Pdo index=\"0x1600\" is_tx=\"false\">"
           "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"target_position\" index=\"0x607a\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"target_velocity\" index=\"0x60ff\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
           "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
           "			</Pdo>"
           "		</SyncManager>"
           "		<SyncManager is_tx=\"true\">"
           "			<Pdo index=\"0x1a00\" is_tx=\"true\">"
           "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"Modes_of_operation_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
           "				<PdoEntry name=\"Current_actual_value\" index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
           "			</Pdo>"
           "		</SyncManager>"
           "  </SyncManagerPoolObject>"
           "</EthercatSlave>";
    auto& s = master->slavePool().add<aris::control::EthercatSlave>();
    aris::core::fromXmlString(s, xml_str);
   
   // the Macro includes [ WIN32 ]  & [ ARIS_USE_ETHERCAT_SIMULATION ] here
    #ifdef ARIS_USE_ETHERCAT_SIMULATION
            dynamic_cast<aris::control::EthercatSlave&>(master->slavePool().back()).setVirtual(true);
    #endif
    #ifndef ARIS_USE_ETHERCAT_SIMULATION
            dynamic_cast<aris::control::EthercatSlave&>(master->slavePool().back()).scanInfoForCurrentSlave();
    #endif
       s.setSync0ShiftNs(600000);
       s.setDcAssignActivate(0x300);
    }
    return master;
}
auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::Controller);

    for (aris::Size i = 0; i < 12 ; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[12]
        {
            // 0, 0, 0,
            // 0, 0, 0,
            // 0, 0, 0,
            // 0, 0, 0,       
            -PI/2, -PI/4, -PI/2,
            -PI/2, -PI/4, -PI/2,
             PI/2, -PI/4, -PI/2,
             PI/2, -PI/4, -PI/2,
        };
#else
        double pos_offset[12]
        {
            // 0, 0, 0
            // -1.4974336956172, 0.128106570548551, 0.844257485597249,
            // -1.4974336956172, 0.128106570548551, 0.844257485597249,
            // -1.4974336956172, 0.128106570548551, 0.844257485597249,
            // -1.4974336956172, 0.128106570548551, 0.844257485597249, // 度： 85， 7.4, 48.35
            // -0.894181369710104, 0.119132782939402, 0.844199961317703

            // -PI/2, -PI/4, -PI/2,
            // -PI/2, -PI/4, -PI/2,
            //  PI/2, -PI/4, -PI/2,
            //  PI/2, -PI/4, -PI/2,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,

            // 0, PI/4, -PI/2,
            // 0, PI/4, -PI/2,
            // 0, PI/4, -PI/2,
            // 0, PI/4, -PI/2,             
        };
#endif
        double pos_factor[12] //偏置系数//
        {
            // 2000/PI,2000/PI,2000/PI
            131072*5/PI,131072*5/PI,131072*10/PI,
            131072*5/PI,131072*5/PI,131072*10/PI,
            131072*5/PI,131072*5/PI,131072*10/PI,
            131072*5/PI,131072*5/PI,131072*10/PI,

        };
        double max_pos[12] //最大位置//
        {
            // 500*PI,500*PI,500*PI  
            // PI/6, PI/2, 2 * PI/3
            // 1.6, 1.5, 2.2,
            // 1.6, 1.5, 2.2,
            // 1.6, 1.5, 2.2,
            // 1.6, 1.5, 2.2,
            500*PI,500*PI,500*PI,  
            500*PI,500*PI,500*PI,  
            500*PI,500*PI,500*PI,  
            500*PI,500*PI,500*PI,  

        };
        double min_pos[12] //最小位置//
        {
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            // -0.6, -0.4, -1.8,
            // -0.6, -0.4, -1.8,
            // -0.6, -0.4, -1.8,
            // -0.6, -0.4, -1.8,
        };
        double max_vel[12]  //最大速度//
        {
            // 330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
            1, 1, 1,
            1, 1, 1,
            1, 1, 1,
            1, 1, 1,
        };
        double max_acc[12]  //最大加速度//
        {
            // 3000,  3000,  3000
            10, 10, 10,
            10, 10, 10,
            10, 10, 10,
            10, 10, 10,
        };
        //zero_err//
        std::string xml_str =
            "<EthercatMotor min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\" slave=\""+std::to_string(i) + "\">"
            "</EthercatMotor>";

       auto& s = controller->motorPool().add<aris::control::EthercatMotor>();
               aris::core::fromXmlString(s, xml_str);
       s.setEnableWaitingCount(100);
    };
    return controller;
}
auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");
    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");
    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();
    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //-------------自己写的命令-----------------//
    plan_root->planPool().add<SetMaxTorque>();
    plan_root->planPool().add<Ellipse4LegDrive3>();
    plan_root->planPool().add<ReadInformation>();
    plan_root->planPool().add<ModelMotorInitialize>();
    plan_root->planPool().add<MotorTest>();
    plan_root->planPool().add<SetMotorPosZero>();
    plan_root->planPool().add<TrotMove>();

    return plan_root;
}
auto setMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value, size_t index)->bool
{
  return (ecMaster->slavePool()[index].writePdo(0x6072, 0x00, std::uint16_t(value)) ? true : false);
}
auto setAllMotorMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value)->void
{
  for (int i = 0; i < ecMaster->slavePool().size(); ++i)
  {
    if (setMaxTorque(ecMaster, 100, i))
    {
        std::cout << "Set Motor " << i << " max torque failed!" << std::endl;
    }
  }
}
}