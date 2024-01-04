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
using namespace aris::dynamic;
using namespace aris::plan;

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


  ARIS_REGISTRATION{
      aris::core::class_<TrotMove>("TrotMove")
          .inherit<Plan>();
      aris::core::class_<QuadrupedRbtModel>("QuadrupedRbtModel")
          .inherit<Model>();
      aris::core::class_<Ellipse4LegDrive3>("Ellipse4LegDrive3")
          .inherit<Plan>();
  }
}