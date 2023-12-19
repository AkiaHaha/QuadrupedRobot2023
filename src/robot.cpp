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
  total_tn = doubleParam("total_number_of_run_period");

  for (auto& m : motorOptions()) m =
    aris::plan::Plan::CHECK_NONE;
}
auto TrotMove::executeRT()->int
{
  //初始运行时1ms, 生成初始位姿矩阵, 正常
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
    model()->setInputPos(init_motor_pos);
    model()->forwardKinematics();
    model()->getOutputPos(init_m28);

    this->master()->logFileRawName("trot_test");
    count_stop = kTcurvePeriodCount * 2 * total_tn - 1;
    std::copy(init_m28, init_m28 + 28, period_init_m28);
  }
  std::cout << kBars50 << "count = " << count() << std::endl;

  //生成循环和周期的一些参数指标
  period_n = count() / kTcurvePeriodCount + 1;
  time_in_pn = count() % kTcurvePeriodCount;
  switch_number = period_n % 2 == 1 ? false : true;

  //设置生成新的初始矩阵在周期结束后, 第一个count()
  if (time_in_pn == 0) {
    double motor[12]{};
    for (int8_t i = 0; i < 12; i++) {
      motor[i] = controller()->motorPool()[i].targetPos();
    }
    model()->setInputPos(motor);
    model()->forwardKinematics();
    model()->getOutputPos(period_init_m28);
  }


  //使用椭圆轨迹的trot规划生成实时运动矩阵,成功
  double a{}, b{};
  a = (period_n == 1 ? vel_x : (2 * vel_x));
  b = (period_n == 1 ? vel_z : (2 * vel_z));
  EllipseMovePlan ep(a, b, vel_h, switch_number, period_init_m28);
  move_m28 = ep.getCurrentM28(time_in_pn);

  model()->setOutputPos(move_m28);
  if (model()->inverseKinematics()){
    throw std::runtime_error("Move Status Inverse kinematic position failed wawawaw");}
  model()->getInputPos(move_motor_pos);


  //控制电机运动
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
    
  //将末端运动数据写入文件
  for (int8_t i = 0; i < 4; i++) {
    lout() << std::setw(10) << m12[0 + i * 3] << std::setw(10) << m12[1 + i * 3] << std::setw(10) << m12[2 + i * 3] << std::endl;
  }

  //程序运行结束打印最终的参数
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
        //---moveializa theta_---//
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

//2023.12.19 copy yw code of xml model visulization and test//
std::atomic_bool g_is_enabled = false;
std::atomic_bool g_is_error = false;
std::atomic_bool g_is_manual = false;
std::atomic_bool g_is_auto = false;
std::atomic_bool g_is_running = false;
std::atomic_bool g_is_paused = false;
std::atomic_bool g_is_stopped = false;
std::atomic_bool g_emergency_stop = false;
int interval = 1; //插补周期，单位ms
double g_counter = 1.0 * interval;
double g_count = 0.0;
uint32_t connectioncounter = 0;

auto update_state(aris::server::ControlServer& cs) -> void
{
  static bool motion_state[256] = { false };

  aris::Size motion_num = cs.controller().motorPool().size();
  //Size motion_num = 5;//----for 5 axes

  //获取motion的使能状态，0表示去使能状态，1表示使能状态//
  for (aris::Size i = 0; i < motion_num; i++)
  {
    auto cm = dynamic_cast<aris::control::EthercatMotor*>(&cs.controller().motorPool()[i]);
    if ((cm->statusWord() & 0x6f) != 0x27)
    {
      motion_state[i] = 0;
    }
    else
    {
      motion_state[i] = 1;
    }
  }

  //获取ret_code的值，判断是否报错，if条件可以初始化变量，并且取变量进行条件判断//
  g_is_error.store(cs.errorCode());

  g_is_enabled.store(std::all_of(motion_state, motion_state + motion_num, [](bool i) { return i; }));

  auto& inter = dynamic_cast<aris::server::ProgramWebInterface&>(cs.interfacePool().at(0));
  if (inter.isAutoMode())
  {
    g_is_auto.store(true);
  }
  else
  {
    g_is_auto.store(false);
  }

  g_is_running.store(inter.isAutoRunning());
  g_is_paused.store(inter.isAutoPaused());
  g_is_stopped.store(inter.isAutoStopped());
  //暂停、恢复功能复位//
  if (!inter.isAutoRunning())
  {
    g_counter = 1.0 * interval;
  }

  //software emergency
  if (g_emergency_stop.exchange(false))
  {
    if (connectioncounter < 500)
      connectioncounter++;
    else
    {
      connectioncounter = 0;
      //cs.setErrorCode(-3000);
    }
  }

  //socket lose connection
  //if(!g_socket_connected.exchange(true))cs.setErrorCode(-4000);

  //获取力传感器数据，并进行滤波--条件是力传感器存在
  //这里只是简单通过从站数量超过6进行判断，第七个从站可以是io也可以是力传感器，用户需要通过FS_NUM来设定
  // if (cs.controller().slavePool().size() > FS_NUM)
  // {
  //     auto slave7 = dynamic_cast<aris::control::EthercatSlave*>(&cs.controller().slavePool().at(FS_NUM));
  //     /*static int fcinit = 0;
  //     if ((motion_state[4] == 1) && fcinit < 1)
  //     {
  //         std::uint8_t led1 = 0x01;
  //         slave7->writePdo(0x7010, 1, &led1, 1);
  //         fcinit++;
  //     }*/
  //     std::array<double, 6> outdata = { 0,0,0,0,0,0 };
  //     for (int i = 0; i < 6; i++)
  //     {
  //         slave7->readPdo(0x6020, i + 11, &rawdata[i], 32);
  //         lp[i].get_filter_data(2, 10, 0.001, rawdata[i], outdata[i]);
  //         outdata[i] *= 9.8;
  //     }
  //     filterdata.store(outdata);
  // }
}

//获取状态字——100:去使能,200:手动,300:自动,400:程序运行中,410:程序暂停中,420:程序停止，500:错误//
auto get_state_code() -> std::int32_t
{
  if (g_is_enabled.load())
  {
    if (g_is_error.load())
    {
      return 500;
    }
    else
    {
      if (!g_is_auto.load())
      {
        return 200;
      }
      else
      {
        if (g_is_running.load())
        {
          if (g_is_stopped)
          {
            return 420;
          }
          else if (g_is_paused.load())
          {
            return 410;
          }
          else
          {
            return 400;
          }
        }
        else
        {
          return 300;
        }
      }
    }
  }
  else
  {
    return 100;
  }
}
struct GetParam
{
  std::vector<std::vector<double>> part_pq; // 怎么获得 body_pq?
  std::int32_t state_code;
  bool is_cs_started;
  std::string currentPlan;
  std::int32_t currentPlanId;
};

auto DogGet::prepareNrt() -> void
{
  option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO;
  for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
  GetParam par;
  par.part_pq.resize(model()->partPool().size());
  std::vector<double> temp_pq(7, 0.0);
  std::any param = par;
  if (controlServer()->running())
  {
    controlServer()->getRtData(
      [&](aris::server::ControlServer& cs, const aris::plan::Plan* target, std::any& data)-> void
      {
        // for (aris::Size i(-1); ++i < cs.model().partPool().size();)
        // {
        //     model()->generalMotionPool().at(i).updP();
        //     model()->generalMotionPool().at(i).updA();
        //     model()->generalMotionPool().at(i).updV();
        // }

        auto& get_param = std::any_cast<GetParam&>(data);
        // model()->generalMotionPool().at(0).updP();

        auto m = dynamic_cast<aris::dynamic::Model*>(&cs.model());

        for (aris::Size i(-1); ++i < m->partPool().size();)
        {
          //par.tool->getPq(*par.wobj, std::any_cast<GetParam &>(data).part_pq.data() + i * 7);
          m->partPool().at(i).getPq(temp_pq.data());
          get_param.part_pq[i].assign(temp_pq.begin(), temp_pq.end());
        }

        if (target == nullptr)
        {
          get_param.currentPlan = "none";
          get_param.currentPlanId = -1;
        }
        else
        {
          get_param.currentPlan = target->command().name();
          get_param.currentPlanId = const_cast<aris::plan::Plan*>(target)->cmdId();
        }
      },
      param
    );
  }
  auto out_data = std::any_cast<GetParam&>(param);
  auto& cs = *controlServer();
  auto& inter = dynamic_cast<aris::server::ProgramWebInterface&>(cs.interfacePool().at(0));

  std::vector<std::pair<std::string, std::any>> out_param;
  //out_param.push_back(std::make_pair<std::string, std::any>("part_pq", out_data.part_pq));
  nlohmann::json j = nlohmann::json(out_data.part_pq);
  //std::cout << j.dump() << std::endl;
  out_data.state_code = get_state_code();
  out_data.is_cs_started = controlServer()->running();
  // out_param.push_back(std::make_pair<std::string, std::any>("part_pq", std::make_any<nlohmann::json>(std::move(js))));
  out_param.push_back(std::make_pair<std::string, std::any>("part_pq", j.dump()));
  out_param.push_back(std::make_pair<std::string, std::any>("state_code", out_data.state_code));
  out_param.push_back(std::make_pair(std::string("cs_err_code"), std::make_any<int>(cs.errorCode())));
  out_param.push_back(std::make_pair(std::string("cs_err_msg"), std::make_any<std::string>(cs.errorMsg())));
  out_param.push_back(std::make_pair<std::string, std::any>("cs_is_started", out_data.is_cs_started));
  out_param.push_back(std::make_pair<std::string, std::any>("current_plan", out_data.currentPlan));
  out_param.push_back(std::make_pair<std::string, std::any>("current_plan_id", out_data.currentPlanId));
  out_param.push_back(std::make_pair<std::string, std::any>("pro_err_code", inter.lastErrorCode()));
  // export const getProErrCode = state => state.server.pro_err_code;
  out_param.push_back(std::make_pair<std::string, std::any>("pro_err_line", inter.lastErrorLine())); // 
  out_param.push_back(std::make_pair<std::string, std::any>("pro_err_msg", inter.lastError()));
  // export const getProErrMsg = state => state.server.pro_err_msg;
  out_param.push_back(std::make_pair<std::string, std::any>("line", std::get<1>(inter.currentFileLine())));
  // export const getProgramLine = state => state.server.line;
  out_param.push_back(std::make_pair<std::string, std::any>("file", std::get<0>(inter.currentFileLine())));
  //out_param.push_back(std::make_pair<std::string, std::any>("robot_gait", std::make_any<std::string>(gait)));
  // export const getProgramFile = state = > state.server.file;
  ret() = out_param;
  // option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO;
  // option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
}
  // auto DogGet::executeRT() -> int
  // {
  //     return 0;
  // }
auto DogGet::collectNrt() -> void
{
}
DogGet::DogGet(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"get\">"
    "	<GroupParam>"
    "		<Param name=\"tool\" default=\"tool0\"/>"
    "		<Param name=\"wobj\" default=\"wobj0\"/>"
    "	</GroupParam>"
    "</Command>");
}

ARIS_REGISTRATION
{  
    aris::core::class_<SetMaxTorque>("SetMaxTorque")
        .inherit<Plan>();
    aris::core::class_<Ellipse4LegDrive3>("Ellipse4LegDrive3")
        .inherit<Plan>();
    aris::core::class_<ReadInformation>("ReadInformation")
        .inherit<Plan>();
    aris::core::class_<ModelMotorInitialize>("ModelMotorInitialize")
        .inherit<Plan>();
    aris::core::class_<MotorTest>("MotorTest")
        .inherit<Plan>();
    aris::core::class_<SetMotorPosZero>("SetMotorPosZero")
        .inherit<Plan>();
    aris::core::class_<TrotMove>("TrotMove")
        .inherit<Plan>();
    aris::core::class_<QuadrupedRbtModel>("QuadrupedRbtModel")
        .inherit<Model>();
}
//2023.12.19 copy yw code of xml model visulization and test//

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