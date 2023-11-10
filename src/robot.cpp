#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include "robot.h"
#include "cplan.h"
#include "tplan.h"
#include "model.h"
using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;
namespace robot
{
///////////////////////////////////////////////////////< Ellipse-4-Legs Edition-1 >////////////////////////////////////
auto Ellipse4LegDrive::prepareNrt()->void
{
    moveX_ = doubleParam("moveX");
    moveY_ = doubleParam("moveY");
    moveZ_ = doubleParam("moveZ");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |        
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto Ellipse4LegDrive::executeRT()->int
{
    static ellipticalTrajectory4  e4;
    if (count() == 1)
    {
        //----- read the start motorPos to get startPoint by fwdKin ------// <cout startMotorPos & startPoint > //
        static double startMotorPos[12]{0};
        for (int i = 0; i < 12; i++)
        {
            startMotorPos[i] = controller()->motorPool()[i].actualPos();
        }

        std::cout << std::endl<< "startMotorPos-ooo" << std::endl;
        std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() <<"\t"<< controller()->motorPool()[1].actualPos()<<"\t"<< controller()->motorPool()[2].actualPos() << std::endl;
        std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() <<"\t"<< controller()->motorPool()[4].actualPos()<<"\t"<< controller()->motorPool()[5].actualPos() << std::endl;
        std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() <<"\t"<< controller()->motorPool()[7].actualPos()<<"\t"<< controller()->motorPool()[8].actualPos() << std::endl;
        std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() <<"\t"<< controller()->motorPool()[10].actualPos()<<"\t"<< controller()->motorPool()[11].actualPos() << std::endl;

        model()->setInputPos(startMotorPos) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Initialization Forward Kinematics Position Failed!");
        }   
        model()->getOutputPos(startPoint_);
        std::cout << std::endl << "startLegPoint-***" << std::endl;
        std::cout << "leg 1 --> " << startPoint_[0] << "\t" << startPoint_[1] << "\t" << startPoint_[2] << std::endl;
        std::cout << "leg 2 --> " << startPoint_[3] << "\t" << startPoint_[4] << "\t" << startPoint_[5] << std::endl;
        std::cout << "leg 3 --> " << startPoint_[6] << "\t" << startPoint_[7] << "\t" << startPoint_[8] << std::endl;
        std::cout << "leg 4 --> " << startPoint_[9] << "\t" << startPoint_[10] << "\t" << startPoint_[11] << std::endl;

        //--- use user defined endPoint & Height to initialize elipse curve ---// < get unitVector of Major axis & centerPoint > //
        endPoint_[0] = startPoint_[0] + moveX_;
        endPoint_[1] = startPoint_[1]; 
        endPoint_[2] = startPoint_[2] + moveZ_;
        endPoint_[3] = startPoint_[3] + moveX_;
        endPoint_[4] = startPoint_[4]; 
        endPoint_[5] = startPoint_[5] + moveZ_;
        endPoint_[6] = startPoint_[6] + moveX_;
        endPoint_[7] = startPoint_[7]; 
        endPoint_[8] = startPoint_[8] + moveZ_;
        endPoint_[9] = startPoint_[9] + moveX_;
        endPoint_[10] = startPoint_[10]; 
        endPoint_[11] = startPoint_[11] + moveZ_;

        std::cout << std::endl << "endLegPoint-***" << std::endl;
        std::cout << "leg 1 --> " << endPoint_[0] << "\t" << endPoint_[1] << "\t" << endPoint_[2] << std::endl;
        std::cout << "leg 2 --> " << endPoint_[3] << "\t" << endPoint_[4] << "\t" << endPoint_[5] << std::endl;
        std::cout << "leg 3 --> " << endPoint_[6] << "\t" << endPoint_[7] << "\t" << endPoint_[8] << std::endl;
        std::cout << "leg 4 --> " << endPoint_[9] << "\t" << endPoint_[10] << "\t" << endPoint_[11] << std::endl;

        for (int i = 0; i < 3; i++)
        {
            startPoint[i] = startPoint_[i];
        }
        

        e4.init(moveX_, moveY_, moveZ_, startPoint);
        majorAxisUnitVector_ = e4.getMajorAxisUnitVector();
        minorAxisUnitVector_ = e4.getMinorAxisUnitVector();
        centerPoint_ = e4.getCenterPoint();    
        Width_  = e4.getWidth();


        // std::cout << "centerPoint: X: " << centerPoint[0] << "   Y: " << centerPoint[1] << "   Z: " << centerPoint[2] << " " << std::endl;
        // std::cout << "majorAxisUnitVector:  X: " << majorAxisUnitVector[0] << "   Y: " << majorAxisUnitVector[1] << "   Z: " << majorAxisUnitVector[2] << " " << std::endl;
        // std::cout << "minorAxisUnitVector:  X: " << minorAxisUnitVector[0] << "   Y: " << minorAxisUnitVector[1] << "   Z: " << minorAxisUnitVector[2] << " " << std::endl;
        theta_ = aris::PI;
        theta_d_=0;
        theta_dd_=0;   
        Height_ = moveY_;
    }


///////--- use the function moveAbsolute2 to get RT eePoint ---//--- variable by theta ---//---cout theta & eePoint with RT count() ---//////////////////
    aris::Size total_count;
    auto ret = moveAbsolute2( theta_, theta_d_, theta_dd_, 0, 0.1, 1, 0.1, 0.5, 0.5, 1e-3, 1e-10, theta_, theta_d_, theta_dd_, total_count);

    for (int i = 0; i < 3; i++)
    {
        eePoint_[i] = centerPoint_[i] + majorAxisUnitVector_[i] * Width_ * std::cos( theta_ ) + minorAxisUnitVector_[i] * Height_ * std::sin( theta_ );
    }    

    for (int i = 3; i < 5; i++)
    {
        eePoint_[i] = centerPoint_[i-3] + majorAxisUnitVector_[i-3] * Width_ * std::cos( theta_ ) + minorAxisUnitVector_[i-3] * Height_ * std::sin( theta_ );
    }   

    for (int i = 5; i < 8; i++)
    {
        eePoint_[i] = centerPoint_[i-6] + majorAxisUnitVector_[i-6] * Width_ * std::cos( theta_ ) + minorAxisUnitVector_[i-6] * Height_ * std::sin( theta_ );
    }  

    for (int i = 8; i < 11; i++)
    {
        eePoint_[i] = centerPoint_[i-9] + majorAxisUnitVector_[i-9] * Width_ * std::cos( theta_ ) + minorAxisUnitVector_[i-9] * Height_ * std::sin( theta_ );
    }  

    
    if (count() % 10 == 0)
    {
        std::cout << std::endl;     
        std::cout << "count= " << count() << std::endl;
        std::cout << "theta= " << theta_ << "  Height= " << Height_ << "  Width= " << Width_ << std::endl; 

        std::cout << "eeLegPoint-###" << std::endl;
        std::cout << "leg 1 --> " << eePoint_[0] << "\t" << eePoint_[1] << "\t" << eePoint_[2] << std::endl;
        std::cout << "leg 2 --> " << eePoint_[3] << "\t" << eePoint_[4] << "\t" << eePoint_[5] << std::endl;
        std::cout << "leg 3 --> " << eePoint_[6] << "\t" << eePoint_[7] << "\t" << eePoint_[8] << std::endl;
        std::cout << "leg 4 --> " << eePoint_[9] << "\t" << eePoint_[10] << "\t" << eePoint_[11] << std::endl;
    }
      
    //--- use eePoint to get RT motorPos by invKin ---//--- store the RT motorPos by eeMotorPos[12] ---//---cout eeMotorPos---//---drive the motor to eeMotorPos---//

    model()->setOutputPos(eePoint_);
    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed");    
    }    
    
    double eeMotorPos[12]{};
    for (int i = 0; i < 12; i++)
    {
        eeMotorPos[i] = model()->motionPool()[i].mp() ;
    }
    std::cout << "eeMotorPos-###" << std::endl;
    std::cout << "leg 1 --> " << eeMotorPos[0] << "\t" << eeMotorPos[1] << "\t" << eeMotorPos[2] << std::endl;
    std::cout << "leg 2 --> " << eeMotorPos[3] << "\t" << eeMotorPos[4] << "\t" << eeMotorPos[5] << std::endl;
    std::cout << "leg 3 --> " << eeMotorPos[6] << "\t" << eeMotorPos[7] << "\t" << eeMotorPos[8] << std::endl;
    std::cout << "leg 4 --> " << eeMotorPos[9] << "\t" << eeMotorPos[10] << "\t" << eeMotorPos[11] << std::endl;
   
    for(int i=0; i<12; ++i)
    {
    controller()->motorPool()[i].setTargetPos( eeMotorPos[i] );
    }
    std::cout << "Move Successed!" <<std::endl;

    if( ret == 0 )
    {   
        model()->setInputPos(eeMotorPos) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Final Forward Kinematics Position Failed!");
        }   
        model()->getOutputPos(finalPoint_);  

        std::cout << std::endl;
        std::cout << "ret = " << ret <<std::endl;

        std::cout << std::endl<< "startMotorPos-ooo" << std::endl;
        std::cout << "leg 1 --> " << controller()->motorPool()[0].actualPos() <<"\t"<< controller()->motorPool()[1].actualPos()<<"\t"<< controller()->motorPool()[2].actualPos() << std::endl;
        std::cout << "leg 2 --> " << controller()->motorPool()[3].actualPos() <<"\t"<< controller()->motorPool()[4].actualPos()<<"\t"<< controller()->motorPool()[5].actualPos() << std::endl;
        std::cout << "leg 3 --> " << controller()->motorPool()[6].actualPos() <<"\t"<< controller()->motorPool()[7].actualPos()<<"\t"<< controller()->motorPool()[8].actualPos() << std::endl;
        std::cout << "leg 4 --> " << controller()->motorPool()[9].actualPos() <<"\t"<< controller()->motorPool()[10].actualPos()<<"\t"<< controller()->motorPool()[11].actualPos() << std::endl;

        std::cout << std::endl << "finalLegPoint-***" << std::endl;
        std::cout << "leg 1 --> " << finalPoint_[0] << "\t" << finalPoint_[1] << "\t" << finalPoint_[2] << std::endl;
        std::cout << "leg 2 --> " << finalPoint_[3] << "\t" << finalPoint_[4] << "\t" << finalPoint_[5] << std::endl;
        std::cout << "leg 3 --> " << finalPoint_[6] << "\t" << finalPoint_[7] << "\t" << finalPoint_[8] << std::endl;
        std::cout << "leg 4 --> " << finalPoint_[9] << "\t" << finalPoint_[10] << "\t" << finalPoint_[11] << std::endl; 

        std::cout << std::endl << "Congratulations!~"  << std::endl;
    }

    return  ret ; 

} 
auto Ellipse4LegDrive::collectNrt()->void {}
Ellipse4LegDrive::Ellipse4LegDrive(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"e4\">"
       "	<GroupParam>" //下面三个参数决定单腿末端的运动终点坐标,即单腿末端现在的位置坐标为->ee0(X0,Y0,Z0) , 单腿末端由本指令将通过椭圆轨迹移动至坐标->ee(x,y,z) ,下方三个参数即为可由用户在工作空间内自定义的末端位置坐标变量//
       "	<Param name=\"moveX\" default=\"0.1\" abbreviation=\"x\"/>"
       "	<Param name=\"moveY\" default=\"0.08\" abbreviation=\"y\"/>"
       "	<Param name=\"moveZ\" default=\"0\" abbreviation=\"z\"/>"
       "	</GroupParam>"
       "</Command>");
}
Ellipse4LegDrive::~Ellipse4LegDrive() = default; 


///////////////////////////////////////////////////////< 12 motors test  >////////////////////////////////////////////////////
                          ///---------test all the motor hardware by send a target pos variation---------------///
auto MotorTest12::prepareNrt()->void
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
            // aris::plan::Plan::NOT_CHECK_ENABLE |
            // aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
            aris::plan::Plan::CHECK_NONE;
}
auto MotorTest12::executeRT()->int
{
    static double begin_angle[12];
    double iNumber[12] = {motor0_, motor1_, motor2_, motor3_, motor4_, motor5_, motor6_, motor7_, motor8_, motor9_, motor10_, motor11_};


    for(int i=0 ; i < 12; ++i)
    {
        dir[i] = int( std::abs(iNumber[i]) /  iNumber[i] );
        iNumber[i] = int(std::abs(iNumber[i]) * 10000 );
        // std::cout << " iNumber init sucess " << std::endl;
    }

    for(int i=0; i<12; ++i)
    {
        if(iNumberMax <= iNumber[i])
        {
            iNumberMax = iNumber[i];
        }
        // std::cout << " iNumberMax =  " << iNumberMax << std::endl;
    }

    if (count() == 1)
    {
        for(int i=0 ; i < 12; ++i)
        {
        begin_angle[i] = controller()->motorPool()[i].actualPos();
        // std::cout << "begin_angle " << i << " approches~ " <<std::endl;
        }
    }


    for(int j=0; j<12; ++j)
    {
        if (count() <= iNumber[j] )
        {
            if (count() % 100 == 0)
            {
                std::cout << "count= " << count() <<std::endl;
                std::cout << "motor"<< j << "< Pos: " << controller()->motorPool()[0].actualPos() << "\t";
                std::cout << "Vel:" << controller()->motorPool()[0].actualVel() <<" >"<< std::endl << std::endl;
            }
            angle[j] = begin_angle[j] + count() * 0.0001 * dir[j];
            controller()->motorPool()[j].setTargetPos(angle[j]); 
        }
    }
    
    if ( count() == iNumberMax )
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
        std::cout <<"Final BodyPE & Leg Point:"<<std::endl;
        std::cout << finalPE[0] << "\t" << finalPE[1] << "\t" << finalPE[2] << "\t" << finalPE[3] << "\t;\t" << finalPE[4] << "\t" << finalPE[5] << "\t" << finalPE[6] << std::endl;
        std::cout << finalPE[7] << "\t" << finalPE[8] << "\t" << finalPE[9] << "\t" << finalPE[10] << "\t;\t" << finalPE[11] << "\t" << finalPE[12] << "\t" << finalPE[13]<< std::endl;
        std::cout << finalPE[14] << "\t" << finalPE[15] << "\t" << finalPE[16] << "\t" << finalPE[17] << "\t;\t" << finalPE[18] << "\t" << finalPE[19] << "\t" << finalPE[20] << std::endl;
        std::cout << finalPE[21] << "\t" << finalPE[22] << "\t" << finalPE[23] << "\t" << finalPE[24] << "\t;\t" << finalPE[25] << "\t" << finalPE[26] << "\t" << finalPos[27] << std::endl;
    }

    return  iNumberMax - count(); 
}
auto MotorTest12::collectNrt()->void {}
MotorTest12::MotorTest12(const std::string &name) 
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
MotorTest12::~MotorTest12() = default; 

///////////////////////////////////////////////////////< Ellipse-Edition3 >////////////////////////////////////
auto ellipticalTrajectoryDrive3::prepareNrt()->void
{
    Height = doubleParam("Height");
    targetX = doubleParam("targetX");
    targetY = doubleParam("targetY");
    targetZ = doubleParam("targetZ");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |        
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ellipticalTrajectoryDrive3::executeRT()->int
{
    static ellipticalTrajectory3  ellipticalTrajectory;
    if (count() == 1)
    {
        //----- read the start motorPos to get startPoint by fwdKin ------// <cout startMotorPos & startPoint > //
        static double startMotorPos[3]{0};
        startMotorPos[0] = controller()->motorPool()[0].actualPos();
        startMotorPos[1] = controller()->motorPool()[1].actualPos();
        startMotorPos[2] = controller()->motorPool()[2].actualPos();  
        std::cout <<std::endl<< "startMotorPos" << "  m0: " << startMotorPos[0] <<"   m1: "<< startMotorPos[1]<<"   m2: "<< startMotorPos[2] <<""<<std::endl;     
        model()->setInputPos(startMotorPos) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Initialization Forward Kinematics Position Failed!");
        }   
        model()->getOutputPos(startPoint);
        std::cout << "startPoint:  X: " << startPoint[0] << "   Y: " << startPoint[1] << "   Z: " << startPoint[2] << " " << std::endl;

        //--- use user defined endPoint & Height to initialize elipse curve ---// < get unitVector of Major axis & centerPoint > //
        endPoint[0]=targetX;
        endPoint[1]=targetY;
        endPoint[2]=targetZ;   

        std::cout << "endPoint:    X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << "" << std::endl;

        ellipticalTrajectory.init(startPoint, endPoint);
        majorAxisUnitVector = ellipticalTrajectory.getMajorAxisUnitVector();
        minorAxisUnitVector = ellipticalTrajectory.getMinorAxisUnitVector();
        centerPoint = ellipticalTrajectory.getCenterPoint();    
        Width  = ellipticalTrajectory.getWidth();


        std::cout << "centerPoint: X: " << centerPoint[0] << "   Y: " << centerPoint[1] << "   Z: " << centerPoint[2] << " " << std::endl;
        std::cout << "majorAxisUnitVector:  X: " << majorAxisUnitVector[0] << "   Y: " << majorAxisUnitVector[1] << "   Z: " << majorAxisUnitVector[2] << " " << std::endl;
        std::cout << "minorAxisUnitVector:  X: " << minorAxisUnitVector[0] << "   Y: " << minorAxisUnitVector[1] << "   Z: " << minorAxisUnitVector[2] << " " << std::endl;

               
        theta = aris::PI;
        theta_d=0;
        theta_dd=0;   
    }
//////////////////////////////////////////////////////---use tCurve to control theta---//////////////////////////////////////////////////////////////////////////////
    tCurve t1(5,2);
    t1.getCurveParam();
    theta = aris::PI * ( 1- t1.getCurve( count() ) );

    for (int i = 0; i < 3; i++)
    {
        eePoint[i] = centerPoint[i] + majorAxisUnitVector[i] * Width * std::cos( theta ) + minorAxisUnitVector[i] * Height * std::sin( theta );
    }    
    

    // if (count() % 10 == 0)
    // {
    std::cout << std::endl;     
    std::cout << "count= " << count() << std::endl;
    std::cout << "theta= " << theta << "  Height= " << Height << "  Width= " << Width << std::endl; 
    std::cout << "eePoint=" <<"  X: " << eePoint[0] << "\t"<<"Y: " << eePoint[1] << "\t" << "Z: " << eePoint[2] << std::endl;   
    // }
      

    //--- use eePoint to get RT motorPos by invKin ---//--- store the RT motorPos by eeMotorPos[3] ---//---cout eeMotorPos---//---drive the motor to eeMotorPos---//

    model()->setOutputPos(eePoint);
    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed");    
    }    
    
    double eeMotorPos[3]{};
    for (int i = 0; i < 3; i++)
    {
        eeMotorPos[i] = model()->motionPool()[i].mp() ;
    }
    std::cout << "eeMotorPos: " << eeMotorPos[0] << "\t" << eeMotorPos[1] << "\t" << eeMotorPos[2] << std::endl;
   
    for(int i=0; i<3; ++i)
    {
    controller()->motorPool()[i].setTargetPos( eeMotorPos[i] );
    }
    std::cout << "Move Successed!" <<std::endl;

    if( (t1.getTc() * 1000 -1) == count() )
    {   
        std::cout << std::endl;
        std::cout << "count = " << count() <<std::endl;
        std::cout << "startPoint:   < X: " << startPoint[0] << "   Y: " << startPoint[1] << "   Z: " << startPoint[2] << " >" << std::endl;
        std::cout << "endPoint:  < X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << " >" << std::endl;
        std::cout << "finalPoint: < X: " << eePoint[0] << "   Y: " << eePoint[1] << "   Z: " << eePoint[2] << " >" << std::endl;
    }
    return  (t1.getTc() * 1000 -1) - count() ; 

/////////--- use the function moveAbsolute2 to get RT eePoint ---//--- variable by theta ---//---cout theta & eePoint with RT count() ---//////////////////
    // aris::Size total_count;
    // auto ret = moveAbsolute2( theta, theta_d, theta_dd, 0, 0.1, 1, 0.1, 0.5, 0.5, 1e-3, 1e-10, theta, theta_d, theta_dd, total_count);

    // for (int i = 0; i < 3; i++)
    // {
    //     eePoint[i] = centerPoint[i] + majorAxisUnitVector[i] * Width * std::cos( theta ) + minorAxisUnitVector[i] * Height * std::sin( theta );
    // }    
    
    // // if (count() % 10 == 0)
    // // {
    // std::cout << std::endl;     
    // std::cout << "count= " << count() << std::endl;
    // std::cout << "theta= " << theta << "  Height= " << Height << "  Width= " << Width << std::endl; 
    // std::cout << "eePoint=" <<"  X: " << eePoint[0] << "\t"<<"Y: " << eePoint[1] << "\t" << "Z: " << eePoint[2] << std::endl;   
    // // }
      
    // //--- use eePoint to get RT motorPos by invKin ---//--- store the RT motorPos by eeMotorPos[3] ---//---cout eeMotorPos---//---drive the motor to eeMotorPos---//

    // model()->setOutputPos(eePoint);
    // if (model()->solverPool()[0].kinPos())
    // {
    //     throw std::runtime_error("Move Status Inverse kinematic position failed");    
    // }    
    
    // double eeMotorPos[3]{};
    // for (int i = 0; i < 3; i++)
    // {
    //     eeMotorPos[i] = model()->motionPool()[i].mp() ;
    // }
    // std::cout << "eeMotorPos: " << eeMotorPos[0] << "\t" << eeMotorPos[1] << "\t" << eeMotorPos[2] << std::endl;
   
    // for(int i=0; i<3; ++i)
    // {
    // controller()->motorPool()[i].setTargetPos( eeMotorPos[i] );
    // }
    // std::cout << "Move Successed!" <<std::endl;

    // if( ret == 0 )
    // {   
    //     std::cout << std::endl;
    //     std::cout << "ret = " << ret <<std::endl;
    //     std::cout << "startPoint:   < X: " << startPoint[0] << "   Y: " << startPoint[1] << "   Z: " << startPoint[2] << " >" << std::endl;
    //     std::cout << "endPoint:  < X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << " >" << std::endl;
    //     std::cout << "finalPoint: < X: " << eePoint[0] << "   Y: " << eePoint[1] << "   Z: " << eePoint[2] << " >" << std::endl;
    // }
    // return  ret ; 

} 
auto ellipticalTrajectoryDrive3::collectNrt()->void {}
ellipticalTrajectoryDrive3::ellipticalTrajectoryDrive3(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"e3\">"
       "	<GroupParam>" //下面三个参数决定单腿末端的运动终点坐标,即单腿末端现在的位置坐标为->ee0(X0,Y0,Z0) , 单腿末端由本指令将通过椭圆轨迹移动至坐标->ee(x,y,z) ,下方三个参数即为可由用户在工作空间内自定义的末端位置坐标变量//
       "	<Param name=\"Height\" default=\"0.1\" abbreviation=\"h\"/>"
       "	<Param name=\"targetX\" default=\"0\" abbreviation=\"x\"/>"
       "	<Param name=\"targetY\" default=\"-0.5\" abbreviation=\"y\"/>"
       "	<Param name=\"targetZ\" default=\"0\" abbreviation=\"z\"/>"
       "	</GroupParam>"
       "</Command>");
}
ellipticalTrajectoryDrive3::~ellipticalTrajectoryDrive3() = default; 

///////////////////////////////////////////////////////< moveBeeLine-Edition2 >//////////////////////////////////
auto moveBeeLineE2::prepareNrt()->void
{
    moveTargetX = doubleParam("moveTargetX");
    moveTargetY = doubleParam("moveTargetY");
    moveTargetZ = doubleParam("moveTargetZ");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto moveBeeLineE2::executeRT()->int
{
    if (count() == 1)
    {
        static double angleOfStatus1[3]{0};
        angleOfStatus1[0] = controller()->motorPool()[0].targetPos();
        angleOfStatus1[1] = controller()->motorPool()[1].targetPos();
        angleOfStatus1[2] = controller()->motorPool()[2].targetPos();  

        std::cout <<std::endl<< "Initial Angle of Motors " << " < m0: " << angleOfStatus1[0] <<"   m1: "<< angleOfStatus1[1]<<"   m2"<< angleOfStatus1[2] <<" >"<<std::endl;     
        std::cout << "Step 2 Started! The Leg has prepared to move !"<< std::endl; 

        // Forward Kinematics //
        model()->setInputPos(angleOfStatus1) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Status1 Forward Kinematics Position Failed!");
        }   

        model()->getOutputPos(posOfStatus1);

        std::cout<<std::endl<<"EE Pos Coordinates of Status 1 : <"<<"X:"<<posOfStatus1[0]<<"\t"<<"Y:"<<posOfStatus1[1]<<"\t"<<"Z:"<<posOfStatus1[2]<<" >"<<std::endl <<std::endl;             

        for (int i = 0; i < 3; i++)
        {
            posOfMoveStatus[i]=posOfStatus1[i]; 
        }
    }

    aris::Size total_count;
    auto retx = moveAbsolute2( posOfMoveStatus[0], velOfMoveStatus[0], accOfMoveStatus[0], moveTargetX, 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, posOfMoveStatus[0], velOfMoveStatus[0], accOfMoveStatus[0], total_count);
    auto rety = moveAbsolute2( posOfMoveStatus[1], velOfMoveStatus[1], accOfMoveStatus[1], moveTargetY, 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, posOfMoveStatus[1], velOfMoveStatus[1], accOfMoveStatus[1], total_count);
    auto retz = moveAbsolute2( posOfMoveStatus[2], velOfMoveStatus[2], accOfMoveStatus[2], moveTargetZ, 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, posOfMoveStatus[2], velOfMoveStatus[2], accOfMoveStatus[2], total_count);

    auto retmax=std::max( retx, std::max(rety,retz) );

    std::cout<<"count = "<< count() <<std::endl;
    std::cout<<"--> EE Pos Coordinates of Move Status to get invKin slove : <"<<"X:"<<posOfMoveStatus[0]<<"\t"<<"Y:"<<posOfMoveStatus[1]<<"\t"<<"Z:"<<posOfMoveStatus[2]<<" >"<<std::endl;     

    model()->setOutputPos(posOfMoveStatus);

    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed");    
    }        
// //////////---------------10.18 tyr to solve discontinous------------------///
//     for(int i=0; i<10; i++)
//     {
//     double actualPos0 = controller()->motorPool()[0].targetPos();
//     controller()->motorPool()[0].setTargetPos(0.1 * i * (model()->motionPool()[0].mp() - actualPos0));
//     }

//     for(int i=0; i<10; i++)
//     {
//     double actualPos1 = controller()->motorPool()[1].targetPos();
//     controller()->motorPool()[1].setTargetPos(0.1 * i * (model()->motionPool()[1].mp() - actualPos1));
//     }    

//     for(int i=0; i<10; i++)
//     {
//     double actualPos2 = controller()->motorPool()[2].targetPos();
//     controller()->motorPool()[2].setTargetPos(0.1 * i * (model()->motionPool()[2].mp() - actualPos2));
//     }     

    controller()->motorPool()[0].setTargetPos(model()->motionPool()[0].mp());
    controller()->motorPool()[1].setTargetPos(model()->motionPool()[1].mp());
    controller()->motorPool()[2].setTargetPos(model()->motionPool()[2].mp());
// //////////---------------10.18 tyr to solve discontinous------------------///
    
	std::cout << "--> Target Angle of Motor Joint From the Inverse Kinematics  " <<"<Joint0 : "<<model()->motionPool()[0].mp() << "\t" <<"Joint1 : "<< model()->motionPool()[1].mp() << "\t" <<"Joint2 : "<< model()->motionPool()[2].mp() <<" >"<< std::endl << std::endl;
    
    if(retmax == 0 ) 
    {
        std::cout<<std::endl<<"--->"<<"Step 2 finished ! The leg's EE has arrived the target position !"<<std::endl;
        std::cout<<"The Initial Coordinate < X: "<<posOfStatus1[0]<<"   Y: "<<posOfStatus1[1]<<"   Z: "<<posOfStatus1[2]<<" >"<<std::endl<<std::endl;
        std::cout<<"The Final   Coordinate < X: "<<posOfMoveStatus[0]<<"   Y: "<<posOfMoveStatus[1]<<"   Z: "<<posOfMoveStatus[2]<<" >"<<std::endl<<std::endl;
    }
    return  retmax; 
}
auto moveBeeLineE2::collectNrt()->void {}
moveBeeLineE2::moveBeeLineE2(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"m2\">"
       "	<GroupParam>" //下面三个参数决定单腿末端的运动终点坐标,即单腿末端经上一步初始化后末端位置为->ee0(X0,Y0,Z0) , 单腿末端由本指令将移动至坐标->ee(x,y,z) ,下方三个参数即为可由用户自定义的末端位置坐标变量//
       "	<Param name=\"moveTargetX\" default=\"0\" abbreviation=\"x\"/>"
       "	<Param name=\"moveTargetY\" default=\"-0.5\" abbreviation=\"y\"/>"
       "	<Param name=\"moveTargetZ\" default=\"0.3\" abbreviation=\"z\"/>"
       "	</GroupParam>"
       "</Command>");
}
moveBeeLineE2::~moveBeeLineE2() = default; 

///////////////////////////////////////////////////////singleLeg-Initialization >/////////////////////////////////////
auto legInitialization::prepareNrt()->void
{

    initAngle0 = doubleParam("initAngle0");
    initAngle1 = doubleParam("initAngle1");
    initAngle2 = doubleParam("initAngle2");

    for(auto &m:motorOptions()) m =
            // aris::plan::Plan::NOT_CHECK_ENABLE |
            // aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
            aris::plan::Plan::CHECK_NONE;
}
auto legInitialization::executeRT()->int
{
    static double begin_angle[3];
    int iNumber0 =int(std::abs(initAngle0) * 10000) ;
    int iNumber1 =int(std::abs(initAngle1) * 10000) ;
    int iNumber2 =int(std::abs(initAngle2) * 10000) ;
    int dir0 = int( std::abs(initAngle0) /  initAngle0 );
    int dir1 = int( std::abs(initAngle1) /  initAngle1 );
    int dir2 = int( std::abs(initAngle2) /  initAngle2 );

    int iNmuberMax = std::max( iNumber0,std::max(iNumber1,iNumber2));

    if (count() == 1)
    {
        begin_angle[0] = controller()->motorPool()[0].actualPos();
        begin_angle[1] = controller()->motorPool()[1].actualPos();
        begin_angle[2] = controller()->motorPool()[2].actualPos();
    }


    if (count() <= iNumber0 )
    {
        if (count() % 10 == 0)
        {
            std::cout << "count = " << count() <<std::endl;
            std::cout << "<joint0>  --  < pos " << ":" << controller()->motorPool()[0].actualPos() << "\t";
            std::cout << "vel" << ":" << controller()->motorPool()[0].actualVel() <<" >"<< std::endl << std::endl;
        }
        double angle0 = begin_angle[0] + count() * 0.0001 * dir0;
        controller()->motorPool()[0].setTargetPos(angle0); 
    }

    if (count() <= iNumber1)
    {
        if (count() % 10 == 0)
        {
            std::cout << "count = " << count() <<std::endl;
            std::cout << "<joint1>  --  < trq " << ":" << controller()->motorPool()[1].actualToq() << "\t";
            std::cout << "  < pos " << ":" << controller()->motorPool()[1].actualPos() << "\t";
            std::cout << "vel" << ":" << controller()->motorPool()[1].actualVel() <<" >"<< std::endl << std::endl;
        }
        
        double angle1 = begin_angle[1] + count() * 0.0001 * dir1 ;
        controller()->motorPool()[1].setTargetPos(angle1); 
    }

    if (count() <= iNumber2)
    {
        if (count() % 10 == 0)
        {
            std::cout << "count = " << count() <<std::endl;
            std::cout << "<joint2>  --  < pos " << ":" << controller()->motorPool()[2].actualPos() << "\t";
            std::cout << "vel" << ":" << controller()->motorPool()[2].actualVel() <<" >"<< std::endl << std::endl;
        }
        
        double angle2 = begin_angle[2] + count() * 0.0001 * dir2 ;
        controller()->motorPool()[2].setTargetPos(angle2); 
    }
    
    if ( count() == iNmuberMax )
    {
        std::cout <<std::endl<< "motorPos " << " < m0: " << controller()->motorPool()[0].actualPos() <<"   m1: "<< controller()->motorPool()[1].actualPos()<<"   m2: "<< controller()->motorPool()[2].actualPos()<<" >"<<std::endl;
        std::cout << "Step 1 finished! The Leg has arrived the initial Pos!"<< std::endl<< std::endl; 
    
        double angleOfStatus1[3]{0};
        angleOfStatus1[0] = controller()->motorPool()[0].actualPos();
        angleOfStatus1[1] = controller()->motorPool()[1].actualPos();
        angleOfStatus1[2] = controller()->motorPool()[2].actualPos();

        //Solve the Forward Kinematics
        model()->setInputPos(angleOfStatus1) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Status1 Forward Kinematics Position Failed!");
        }
       
        double posOfStatus1[3]{0};
        model()->getOutputPos(posOfStatus1);
        std::cout<<"eePos: <"<<"X:"<<posOfStatus1[0]<<"\t"<<"Y:"<<posOfStatus1[1]<<"\t"<<"Z:"<<posOfStatus1[2]<<" >"<<std::endl<<std::endl;        
    }

    return  (iNmuberMax + 1) - count(); 
}
auto legInitialization::collectNrt()->void {}
legInitialization::legInitialization(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"n\">"
       "	<GroupParam>"                                    
       "	<Param name=\"initAngle0\" default=\"0\" abbreviation=\"i\"/>"//通过给定电机初始值初始化单腿的末端位姿,给定值为弧度制，本程序中精度设定给到小数点后三位//
       "	<Param name=\"initAngle1\" default=\"0.1\" abbreviation=\"j\"/>"      
       "	<Param name=\"initAngle2\" default=\"-0.1\" abbreviation=\"k\"/>"
       "	</GroupParam>"
       "</Command>");
}
legInitialization::~legInitialization() = default; 

///////////////////////////////////////////////////////< cosCurve-singleLeg >///////////////////////////////////////
auto cosCurveDriveTogetherM3::prepareNrt()->void
{
   cef_1 = doubleParam("coefficient1");
   cef_2 = doubleParam("coefficient2");
   cef_3 = doubleParam("coefficient3");
   totalTime=doubleParam("totalTime");
   a_  =doubleParam("amplitude");
   w_  =doubleParam("frequency");
   p_  =doubleParam("phase");

   for(auto &m:motorOptions()) m =
           aris::plan::Plan::NOT_CHECK_ENABLE |
           aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto cosCurveDriveTogetherM3::executeRT()->int
{
   static double begin_angle[3];

   if (count() == 1)//实时线程刚开始运行-> count=1 时的设置//
   {
       begin_angle[0] = controller()->motorPool()[0].targetPos();//将运动的初始位置设定为当下电机的实时位置//
       begin_angle[1] = controller()->motorPool()[1].targetPos();
       begin_angle[2] = controller()->motorPool()[2].targetPos();

       this->master()->logFileRawName("testCosCurve");//建立记录数据的文件夹//
   }

   cosCurve cos1(a_,w_,p_); //创建对象，传入初始参数//

   double angle0 = begin_angle[0] + cef_1  * cos1.getCurve(count()) ;//设置变量angle0存储由余弦曲线和公式运算所得的目标位置//
   double angle1 = begin_angle[1] + cef_2  * cos1.getCurve(count()) ;
   double angle2 = begin_angle[2] + cef_3  * cos1.getCurve(count()) ;

   controller()->motorPool()[0].setTargetPos(angle0);//调用aris函数将当下电机位置更新为angle0所标定位置//
   controller()->motorPool()[1].setTargetPos(angle1);
   controller()->motorPool()[2].setTargetPos(angle2);

   if (count() % 10 == 0)//mout::在屏幕上实时打印电机运行情况，每10ms打印一次//
   {
       mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";//电机实时位置//
       mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;//电机实时速度//
   }

   //lout::向文件夹中实时写入电机运行情况，周期与实时线程相同，为1ms一次//
   lout() << controller()->motorPool()[0].actualPos() <<std::endl;
//    lout() << controller()->motorPool()[0].actualVel() <<std::endl;

   return  totalTime * 1000 - count(); //设置结束实时线程的时间//
}
auto cosCurveDriveTogetherM3::collectNrt()->void {}
cosCurveDriveTogetherM3::cosCurveDriveTogetherM3(const std::string &name) 
{
   aris::core::fromXmlString(command(),
      "<Command name=\"c3\">"//运行本线程的指令//
      "	<GroupParam>"
      "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient3\" default=\"1\" abbreviation=\"h\"/>"//自定义参数的缩写//
      "	<Param name=\"totalTime\" default=\"3\" abbreviation=\"t\"/>"//自定义参数的缩写//
      "	<Param name=\"amplitude\" default=\"1\" abbreviation=\"a\"/>"//自定义振幅参数a_的缩写//
      "	<Param name=\"frequency\" default=\"1\" abbreviation=\"w\"/>"//自定义角频率参数w_的缩写//
      "	<Param name=\"phase\" default=\"1.57\" abbreviation=\"p\"/>"//自定义时间参数t_的缩写//
      "	</GroupParam>"
      "</Command>");
}
cosCurveDriveTogetherM3::~cosCurveDriveTogetherM3() = default;   

/////////////////////////////////////////////////< volecity mode >//////////////////////////////////////////////////
auto VelDrive::prepareNrt()->void
{

    cef_ = doubleParam("coefficient");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto VelDrive::executeRT()->int{
    
    static double begin_vel[3];

    if (count()==1)
    {
        begin_vel[0] = controller()->motorPool()[0].actualVel();
        this->master()->logFileRawName("testVel");
    }
    
    double vel0= begin_vel[0]+cef_*5.0*(1-std::cos(2*PI*count()/2000.0))/2;
    
    controller()->motorPool()[0].setTargetVel(vel0);

    //屏幕打印与文件夹数据记录//
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }
    
    lout() << controller()->motorPool()[0].actualPos() <<"\t";
    lout() << controller()->motorPool()[0].actualVel() <<std::endl;
    
    return 6000-count();//运行时间为6000毫秒即6秒//
}
auto VelDrive::collectNrt()->void{}
VelDrive::VelDrive(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"vel\">"
        "	<Param name=\"coefficient\" default=\"1.0\" abbreviation=\"k\"/>"
        "</Command>");
}
VelDrive::~VelDrive() = default;  

///////////////////////////////////////////////////////< test-forward-Kin >//////////////////////////////////////////
auto forwardKinLegTest::prepareNrt()->void
{
    fwdAngle0 = doubleParam("fwdAngle0");
    fwdAngle1 = doubleParam("fwdAngle1");
    fwdAngle2 = doubleParam("fwdAngle2");
    intervals = doubleParam("intervals");
    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto forwardKinLegTest::executeRT()->int
{
    static double begin_angle[3];

//Solve the Forward Kinematics
    double initAngle[3]{ fwdAngle0, fwdAngle1, fwdAngle2};
    model()->setInputPos(initAngle) ;    

    if (model()->solverPool()[1].kinPos())
    {
        throw std::runtime_error("Forward Kinematics Position Failed!");
    }
    
    double initOtputPos[3]{0};
    model()->getOutputPos(initOtputPos);
    std::cout<<"Output Position test in CmdLine: <"<<initOtputPos[0]<<"\t"<<initOtputPos[1]<<"\t"<<initOtputPos[2]<<" >"<<std::endl;

    if (count() == 1)//实时线程刚开始运行-> count=1 时的设置//
    {
        begin_angle[0] = controller()->motorPool()[0].targetPos();
        begin_angle[1] = controller()->motorPool()[1].targetPos();
        begin_angle[2] = controller()->motorPool()[2].targetPos();

        this->master()->logFileRawName("forward_kin_leg_test");
    }
    return  intervals * 1000 -count(); //设置结束实时线程的时间//
}
auto forwardKinLegTest::collectNrt()->void{}
forwardKinLegTest::forwardKinLegTest(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"fwd\">"
       "	<GroupParam>"
       "	<Param name=\"fwdAngle0\" default=\"0\" abbreviation=\"f\"/>"//the input forward angle of motor 0//
       "	<Param name=\"fwdAngle1\" default=\"1.5\" abbreviation=\"g\"/>"//the input forward angle of motor 1//
       "	<Param name=\"fwdAngle2\" default=\"1.2\" abbreviation=\"h\"/>"//the input forward angle of motor 2//
       "	<Param name=\"intervals\" default=\"0.001\" abbreviation=\"j\"/>"//define the interval of two motions//
       "	</GroupParam>"
       "</Command>");
}
forwardKinLegTest::~forwardKinLegTest() = default; 

///////////////////////////////////////////////////////< tet-inverse-Kin >///////////////////////////////////////////
auto inverseKinLegTest::prepareNrt()->void
{
    invPos_x = doubleParam("invPos_x");
    invPos_y = doubleParam("invPos_y");
    invPos_z = doubleParam("invPos_z");
    intervals=doubleParam("intervals");
    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto inverseKinLegTest::executeRT()->int
{
    static double begin_angle[3];

    double eePos[3]{ invPos_x ,  invPos_y,  invPos_z };

    model()->setOutputPos(eePos);

    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Inverse kinematic position failed");    
    }
    
	std::cout << " Input Position test in CmdLine : "  << model()->motionPool()[0].mp() << "  " << model()->motionPool()[1].mp() << "  " << model()->motionPool()[2].mp() << std::endl;
    

    if (count() == 1)//实时线程刚开始运行-> count=1 时的设置//
    {
        begin_angle[0] = controller()->motorPool()[0].targetPos();
        begin_angle[1] = controller()->motorPool()[1].targetPos();
        begin_angle[2] = controller()->motorPool()[2].targetPos();

        this->master()->logFileRawName("inverse_kin_leg_test");
    }
    return  intervals * 1000 -count(); //设置结束实时线程的时间//
}
auto inverseKinLegTest::collectNrt()->void{}
inverseKinLegTest::inverseKinLegTest(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"inv\">"
       "	<GroupParam>"
       "	<Param name=\"invPos_x\" default=\"0.1315\" abbreviation=\"x\"/>"//the input eePos of axis X//
       "	<Param name=\"invPos_y\" default=\"-0.512151562\" abbreviation=\"y\"/>"//the input eePos of axis Y//
       "	<Param name=\"invPos_z\" default=\"-0.14515615123\" abbreviation=\"z\"/>"//the input eePos of axis Z//
       "	<Param name=\"intervals\" default=\"0.001\" abbreviation=\"t\"/>"//define the interval time of two motions//
       "	</GroupParam>"
       "</Command>");
}
inverseKinLegTest::~inverseKinLegTest() = default; 

///////////////////////////////////////////////< set-maxTorque >/////////////////////////////////////////////////////////
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

//////////////////////////////////////////< Basic Setting: XML for Motor PDO ; Motor Number ; Plan Instruction >/////////////
auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>{
    std::unique_ptr<aris::control::Master> master(new aris::control::EthercatMaster);

    for (aris::Size i = 0; i < 12 ; ++i){
        int phy_id[12]={0,1,2,3,4,5,6,7,8,9,10,11};

//--------------------------XML from Leo---------------------//
//           " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
//           " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
//           " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"        
        // std::string xml_str =
        //    "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
        //    " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
        //    ">"
        //    "	<SyncManagerPoolObject>"
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
        //    "	</SyncManagerPoolObject>"
        //    "</EthercatMotor>";

//--------------------------the original XML for stepper---------------------//
        // std::string xml_str =
        //     "<EthercatSlave phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
        //     " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
        //     ">"
        //     "	<SyncManagerPoolObject>"
        //     "		<SyncManager is_tx=\"false\"/>"
        //     "		<SyncManager is_tx=\"true\"/>"
        //     "		<SyncManager is_tx=\"false\">"
        //     "			<Pdo index=\"0x1600\" is_tx=\"false\">"
        //     "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
        //     "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
        //     "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        //     "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
        //     "			</Pdo>"
        //     "		</SyncManager>"
        //     "		<SyncManager is_tx=\"true\">"
        //     "			<Pdo index=\"0x1a00\" is_tx=\"true\">"
        //     "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        //     "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
        //     "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        //     "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
        //     "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
        //     "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
        //     "			</Pdo>"
        //     "		</SyncManager>"
        //     "	</SyncManagerPoolObject>"
        //     "</EthercatSlave>";

//--------------------------XML from Guojing-----------------------------------------------//
        // std::string xml_str =
        //    "<EthercatSlave phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
        //    " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
        //    ">"
        //    "  <SyncManagerPoolObject>"
        //    "    <SyncManager is_tx=\"false\"/>"
        //    "    <SyncManager is_tx=\"true\"/>"
        //    "    <SyncManager is_tx=\"false\">"
        //    "      <Pdo index=\"0x1605\" is_tx=\"false\">"
        //    "        <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
        //    "        <PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
        //    "        <PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
        //    "        <PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
        //    "        <PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        //    "        <PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
        //    "      </Pdo>"
        //    "    </SyncManager>"
        //    "    <SyncManager is_tx=\"true\">"
        //    "      <Pdo index=\"0x1A02\" is_tx=\"true\">"
        //    "        <PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        //    "        <PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
        //    "        <PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        //    "        <PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
        //    "      </Pdo>"
        //    "      <Pdo index=\"0x1A11\" is_tx=\"true\">"
        //    "        <PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
        //    "      </Pdo>"
        //    "    </SyncManager>"
        //    "  </SyncManagerPoolObject>"
        //    "</EthercatSlave>";

//----------------------------------Daniel for Single_leg Servo Motor Success in 2023.10.14--------------------------------------//
        std::string xml_str =
           "<EthercatSlave phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
           " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
           ">"
           "  <SyncManagerPoolObject>"
           "		<SyncManager is_tx=\"false\"/>"
           "		<SyncManager is_tx=\"true\"/>"
           "		<SyncManager is_tx=\"false\">"
           "			<Pdo index=\"0x1605\" is_tx=\"false\">"
           "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
           "			</Pdo>"
           "		</SyncManager>"
           "		<SyncManager is_tx=\"true\">"
           "			<Pdo index=\"0x1A07\" is_tx=\"true\">"
           "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
           "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
           "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
           "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
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
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,            
        };
#else
        double pos_offset[12]
        {
            // 0, 0, 0
            -1.4974336956172, 0.128106570548551, 0.844257485597249,
            -1.4974336956172, 0.128106570548551, 0.844257485597249,
            -1.4974336956172, 0.128106570548551, 0.844257485597249,
            -1.4974336956172, 0.128106570548551, 0.844257485597249,
            // -0.894181369710104, 0.119132782939402, 0.844199961317703
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
            1.6, 1.5, 2.2,
            1.6, 1.5, 2.2,
            1.6, 1.5, 2.2,
            1.6, 1.5, 2.2,
        };
        double min_pos[12] //最小位置//
        {
            // -500*PI,-500*PI,-500*PI
            -0.6, -0.4, -1.8,
            -0.6, -0.4, -1.8,
            -0.6, -0.4, -1.8,
            -0.6, -0.4, -1.8,
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
    plan_root->planPool().add<VelDrive>();
    plan_root->planPool().add<cosCurveDriveTogetherM3>();
    plan_root->planPool().add<inverseKinLegTest>();
    plan_root->planPool().add<forwardKinLegTest>();
    plan_root->planPool().add<legInitialization>();
    plan_root->planPool().add<moveBeeLineE2>();
    plan_root->planPool().add<ellipticalTrajectoryDrive3>();
    plan_root->planPool().add<MotorTest12>();
    plan_root->planPool().add<Ellipse4LegDrive>();
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