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
///////////////////////////////////////////////////////< EE Ellipse Trajectory Plan Movement edition 3 >////////////////////////////////////
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


///////////////////////////////////////////////////////< EE Ellipse Trajectory Plan Movement edition 2 >////////////////////////////////////
auto ellipticalTrajectoryDrive2::prepareNrt()->void
{
    Height = doubleParam("Height");
    Length = doubleParam("Length");
    Direction = doubleParam("Direction");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS |        
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}


auto ellipticalTrajectoryDrive2::executeRT()->int
{
    if (count() == 1)
    {
        static double angleOfInitial[3]{0};
        angleOfInitial[0] = controller()->motorPool()[0].actualPos();
        angleOfInitial[1] = controller()->motorPool()[1].actualPos();
        angleOfInitial[2] = controller()->motorPool()[2].actualPos();  

        std::cout <<std::endl<< "Initial Angle of Motors " << " < m0: " << angleOfInitial[0] <<"   m1: "<< angleOfInitial[1]<<"   m2: "<< angleOfInitial[2] <<" >"<<std::endl;     
        // std::cout << "Step 3 Started! The Leg has prepared to move !"<< std::endl; 

        // Forward Kinematics //
        model()->setInputPos(angleOfInitial) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Initialization Forward Kinematics Position Failed!");
        }   

        model()->getOutputPos(posOfInitialization);

        std::cout <<std::endl << "startPoint:   < X: " << posOfInitialization[0] << "   Y: " << posOfInitialization[1] << "   Z: " << posOfInitialization[2] << " >" << std::endl;

        for (int i = 0; i < 3; i++)
        {
            startPoint[i]=posOfInitialization[i]; 
        } 

        ellipticalTrajectory2  ellipticalTrajectory(startPoint, Length, Direction);
        unitVector = ellipticalTrajectory.getUnitVector();
        centerPoint = ellipticalTrajectory.getCenterPoint();    
               
        theta = aris::PI;
        theta_d=0;
        theta_dd=0;   

        for (int i = 0; i < 3; i++)
        {
            endPoint[i] = startPoint[i] + Length * unitVector[i];
        }
        std::cout << "targetPoint:  < X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << " >" << std::endl <<std::endl;       
    }

    double unitvectorY[3]={0,1,0};
    aris::Size total_count;
    auto ret = moveAbsolute2( theta, theta_d, theta_dd, 0, 0.1, 1, 0.1, 0.5, 0.5, 1e-3, 1e-10, theta, theta_d, theta_dd, total_count);

    for (int i = 0; i < 3; i++)
    {
        eePoint[i] = centerPoint[i] + unitVector[i] * Width * cos(theta) + unitvectorY[i] * Height * sin( theta );
    }
      
    std::cout << "count = " << count() << std::endl;
    std::cout << "--> theta: " << theta << std::endl; 
    std::cout << "--> eePoint-" << count() << " < X: " << eePoint[0] << "\t"<<"Y: " << eePoint[1] << "\t" << "Z: " << eePoint[2] << " >" << std::endl;   
    model()->setOutputPos(eePoint);

    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed");    
    }    
    //||-------------------------------try to solve discontinous problem due to the uncontrolable step------------------//
    // double coordinate[3]{};
    // double ret0=1;
    // double ret1=1;
    // double ret2=1;

    // for (int i = 0; i < 3; i++)
    // {
    //     coordinate[i] = model()->motionPool()[i].mp();
    // }
    
    // double X=controller()->motorPool()[0].actualPos();
    // double Xd{};
    // double Xdd{};
    // double Y=controller()->motorPool()[1].actualPos();
    // double Yd{};
    // double Ydd{};
    // double Z=controller()->motorPool()[2].actualPos();
    // double Zd{};
    // double Zdd{};
    // while(ret0 == 1)
    // {
    //     ret0 = moveAbsolute2( X, Xd, Xdd, coordinate[0], 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, X, Xd, Xdd, total_count);
    //     controller()->motorPool()[0].setTargetPos(X);
    // }

    // while(ret1 == 1)
    // {
    //     ret1 = moveAbsolute2( Y, Yd, Ydd, coordinate[1], 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, Y, Yd, Ydd, total_count);
    //     controller()->motorPool()[1].setTargetPos(Y);
    // }

    // while(ret2 == 1)
    // {
    //     ret2 = moveAbsolute2( Z, Zd, Z, coordinate[2], 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, Z, Zd, Z, total_count);
    //     controller()->motorPool()[2].setTargetPos(Z);
    // }
    //----------------------------------------------------------------------------------------------------------------------||//
	std::cout << "--> currentMotorPos < Joint0: " << controller()->motorPool()[0].actualPos() << "\t" << "Joint1: " << controller()->motorPool()[1].actualPos() << "\t" << "Joint2: " << controller()->motorPool()[2].actualPos() << " >" << std::endl << std::endl;
	std::cout << "--> targetMotorPos  < Joint0: " << model()->motionPool()[0].mp() << "\t" << "Joint1: " << model()->motionPool()[1].mp() << "\t" << "Joint2: " << model()->motionPool()[2].mp() << " >" << std::endl << std::endl;
   
    for(int i=0; i<3; ++i)
    {
    controller()->motorPool()[i].setTargetPos(model()->motionPool()[i].mp());
    std::cout << "success moved of motor " << i << "~"  <<std::endl;
    }
	std::cout << "--> finalMotorPos < Joint0: " << controller()->motorPool()[0].actualPos() << "\t" << "Joint1: " << controller()->motorPool()[1].actualPos() << "\t" << "Joint2: " << controller()->motorPool()[2].actualPos() << " >" << std::endl << std::endl;
    
    if( ret == 0)
    {   std::cout << "ret = " << ret <<std::endl;
        std::cout << "startPoint:   < X: " << posOfInitialization[0] << "   Y: " << posOfInitialization[1] << "   Z: " << posOfInitialization[2] << " >" << std::endl;
        std::cout << "targetPoint:  < X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << " >" << std::endl;
        std::cout << "currentPoint: < X: " << eePoint[0] << "   Y: " << eePoint[1] << "   Z: " << eePoint[2] << " >" << std::endl;
    }

    return  ret; 
}

auto ellipticalTrajectoryDrive2::collectNrt()->void {}
ellipticalTrajectoryDrive2::ellipticalTrajectoryDrive2(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"e2\">"
       "	<GroupParam>" //下面三个参数决定单腿末端的运动终点坐标,即单腿末端现在的位置坐标为->ee0(X0,Y0,Z0) , 单腿末端由本指令将通过椭圆轨迹移动至坐标->ee(x,y,z) ,下方三个参数即为可由用户在工作空间内自定义的末端位置坐标变量//
       "	<Param name=\"Height\" default=\"0.1\" abbreviation=\"h\"/>"
       "	<Param name=\"Length\" default=\"0.25\" abbreviation=\"l\"/>"
       "	<Param name=\"Direction\" default=\"1.57\" abbreviation=\"d\"/>"
       "	</GroupParam>"
       "</Command>");
}
ellipticalTrajectoryDrive2::~ellipticalTrajectoryDrive2() = default; 

///////////////////////////////////////////////////////< EE Ellipse Trajectory Plan Movement >////////////////////////////////////
auto ellipticalTrajectoryDrive::prepareNrt()->void
{
    moveTargetX = doubleParam("moveTargetX");
    moveTargetY = doubleParam("moveTargetY");
    moveTargetZ = doubleParam("moveTargetZ");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto ellipticalTrajectoryDrive::executeRT()->int
{
    if (count() == 1)
    {
        endPoint[0] = moveTargetX;
        endPoint[1] = moveTargetY;
        endPoint[2] = moveTargetZ;

        static double angleOfInitial[3]{0};
        angleOfInitial[0] = controller()->motorPool()[0].actualPos();
        angleOfInitial[1] = controller()->motorPool()[1].actualPos();
        angleOfInitial[2] = controller()->motorPool()[2].actualPos();  

        std::cout <<std::endl<< "Initial Angle of Motors " << " < m0: " << angleOfInitial[0] <<"   m1: "<< angleOfInitial[1]<<"   m2: "<< angleOfInitial[2] <<" >"<<std::endl;     
        std::cout << "Step 3 Started! The Leg has prepared to move !"<< std::endl; 

        // Forward Kinematics //
        model()->setInputPos(angleOfInitial) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Initialization Forward Kinematics Position Failed!");
        }   

        model()->getOutputPos(posOfInitialization);

        std::cout <<std::endl << "startPoint:   < X: " << posOfInitialization[0] << "   Y: " << posOfInitialization[1] << "   Z: " << posOfInitialization[2] << " >" << std::endl;
        std::cout << "targetPoint:  < X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << " >" << std::endl;          

        for (int i = 0; i < 3; i++)
        {
            startPoint[i]=posOfInitialization[i]; 
        }

        ellipticalTrajectory  ellipticalTrajectory(startPoint, endPoint);
        unitVector = ellipticalTrajectory.getUnitVector();
        centerPoint = ellipticalTrajectory.getCenterPoint();
        
        std::cout << "unitVector of ST: "<< unitVector[0] << "\t" << unitVector[1] << "\t" << unitVector[2] << std::endl;
        std::cout << "centerPoint: "<< centerPoint[0] << "\t" << centerPoint[1] << "\t" << centerPoint[2] << std::endl;         
               
        phi = ellipticalTrajectory.getPhi();
        Width = ellipticalTrajectory.getWidth();
        Height = ellipticalTrajectory.getHeight();
        initializationStatus = ellipticalTrajectory.getInitializationStatus();
        initializationCount = ellipticalTrajectory.getInitializationCount();
        std::cout << "phi: " << phi <<"\t" << "Width: " << Width << "\t"  << "Height" << Height << std::endl;
        std::cout << "iStatus: " << initializationStatus << "\t" << "iCount: " << initializationCount << std::endl;
        theta = aris::PI;
        theta_d=0;
        theta_dd=0;       
    }

    double unitvectorY[3]={0,1,0};
    aris::Size total_count;
    theta_target = aris::PI - phi;
    auto ret = moveAbsolute2( theta, theta_d, theta_dd, theta_target, 0.1, 1.0, 0.1, 0.5, 0.5, 1e-3, 1e-10, theta, theta_d, theta_dd, total_count);

    for (int i = 0; i < 3; i++)
    {
        eePoint[i] = centerPoint[i] + unitVector[i] * Width * cos(theta) + unitvectorY[i] * Height * sin( theta );
    }
      
    std::cout << "count = " << count() << std::endl;
    std::cout << "--> theta: " << theta << std::endl; 
    std::cout << "--> eePoint < X: " << eePoint[0] << "\t"<<"Y: " << eePoint[1] << "\t" << "Z: " << eePoint[2] << " >" << std::endl;   
    model()->setOutputPos(eePoint);

    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed");    
    }    

    double moveMotionPos[3]{0};
    double casualPos[3]{0};
    double posVariation[3]{0};
    for (int i = 0; i < 3; i++)
    {
        moveMotionPos[i] = model()->motionPool()[i].mp() ;
        casualPos[i]=controller()->motorPool()[i].actualPos();
        posVariation[i] =  moveMotionPos[i] - casualPos[i] ;
    }      

    for (int i = 0; i < 100; i++)
    {                                                   
        controller()->motorPool()[0].setTargetPos( casualPos[0] + 0.01 * (i+1) * posVariation[0] );
        controller()->motorPool()[1].setTargetPos( casualPos[1] + 0.01 * (i+1) * posVariation[1] );
        controller()->motorPool()[2].setTargetPos( casualPos[2] + 0.01 * (i+1) * posVariation[2] );
    }

    // for (int i = 0; i < 100; i++)
    // {
    //     controller()->motorPool()[0].setTargetPos( 0.01 * (i+1) * moveMotionPos[0] );
    //     controller()->motorPool()[1].setTargetPos( 0.01 * (i+1) * moveMotionPos[1] );
    //     controller()->motorPool()[2].setTargetPos( 0.01 * (i+1) * moveMotionPos[2] );
    // }
    
    // controller()->motorPool()[0].setTargetPos(model()->motionPool()[0].mp());
    // controller()->motorPool()[1].setTargetPos(model()->motionPool()[1].mp());
    // controller()->motorPool()[2].setTargetPos(model()->motionPool()[2].mp());


	std::cout << "--> motorPos < Joint0: " << model()->motionPool()[0].mp() << "\t" << "Joint1: " << model()->motionPool()[1].mp() << "\t" << "Joint2: " << model()->motionPool()[2].mp() << " >" << std::endl << std::endl;
    
    if( ret == 0)
    {   std::cout << "ret = " << ret <<std::endl;
        std::cout << "startPoint:   < X: " << posOfInitialization[0] << "   Y: " << posOfInitialization[1] << "   Z: " << posOfInitialization[2] << " >" << std::endl;
        std::cout << "targetPoint:  < X: " << endPoint[0] << "   Y: " << endPoint[1] << "   Z: " << endPoint[2] << " >" << std::endl;
        std::cout << "currentPoint: < X: " << eePoint[0] << "   Y: " << eePoint[1] << "   Z: " << eePoint[2] << " >" << std::endl;
        std::cout << "unitVector of ST: "<< unitVector[0] << "\t" << unitVector[1] << "\t" << unitVector[2] << std::endl;
        std::cout << "centerPoint: "<< centerPoint[0] << "\t" << centerPoint[1] << "\t" << centerPoint[2] << std::endl; 
        std::cout << "phi: " << phi <<"\t" << "Width: " << Width << "\t"  << "Height: " << Height << std::endl;
        std::cout << "iStatus: " << initializationStatus << "\t" << "iCount: " << initializationCount << std::endl;
    }

    return  ret; 
}
auto ellipticalTrajectoryDrive::collectNrt()->void {}
ellipticalTrajectoryDrive::ellipticalTrajectoryDrive(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"e\">"
       "	<GroupParam>" //下面三个参数决定单腿末端的运动终点坐标,即单腿末端现在的位置坐标为->ee0(X0,Y0,Z0) , 单腿末端由本指令将通过椭圆轨迹移动至坐标->ee(x,y,z) ,下方三个参数即为可由用户在工作空间内自定义的末端位置坐标变量//
       "	<Param name=\"moveTargetX\" default=\"0\" abbreviation=\"x\"/>"
       "	<Param name=\"moveTargetY\" default=\"-0.4\" abbreviation=\"y\"/>"
       "	<Param name=\"moveTargetZ\" default=\"-0.2\" abbreviation=\"z\"/>"
       "	</GroupParam>"
       "</Command>");
}
ellipticalTrajectoryDrive::~ellipticalTrajectoryDrive() = default; 


///////////////////////////////////////////////////////< moveBeeLineEdition2 >//////////////////////////////////
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

///////////////////////////////////////////////////////legInitialization >/////////////////////////////////////
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

///////////////////////////////////////////////////////<  c3  > cosCurve余弦曲线-run together M3 >//////////////
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

///////////////////////////////////////////////////< tCurve梯形曲线--run together >//////////////////////////////
auto tCurveDriveTogetherM3::prepareNrt()->void
{
   cef_1 = doubleParam("coefficient1");
   cef_2 = doubleParam("coefficient2");
   cef_3 = doubleParam("coefficient3");
   vel   = doubleParam("velocity");
   acc   = doubleParam("acceleration");
   intervals=doubleParam("intervals");

   for(auto &m:motorOptions()) m =
           aris::plan::Plan::NOT_CHECK_ENABLE |
           aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto tCurveDriveTogetherM3::executeRT()->int{
   static double begin_angle[3];

   if (count() == 1)
   {
       begin_angle[0] = controller()->motorPool()[0].targetPos();
       begin_angle[1] = controller()->motorPool()[1].targetPos();
       begin_angle[2] = controller()->motorPool()[2].targetPos();
   }

   tCurve s1(acc,vel);

   s1.getCurveParam();


       double angle0 = begin_angle[0] + cef_1  * s1.getCurve(count()) ;//设置变量angle0存储由梯形曲线和公式运算所得的目标位置//
       controller()->motorPool()[0].setTargetPos(angle0);              //调用aris函数将当下电机位置更新为angle0所标定位置//
       double angle1 = begin_angle[1] + cef_2  * s1.getCurve(count()) ;
       controller()->motorPool()[1].setTargetPos(angle1);
       double angle2 = begin_angle[2] + cef_3  * s1.getCurve(count()) ;
       controller()->motorPool()[2].setTargetPos(angle2);


   if (count() % 10 == 0)//mout::在屏幕上实时打印电机运行情况，每10ms打印一次//
   {
       mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";//电机实时位置//
       mout() << "cur" << ":" << controller()->motorPool()[0].actualCur() << "\t";//电机实时电流//
       mout() << "toq" << ":" << controller()->motorPool()[0].actualToq() << "\t";//电机实时力矩//
       mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;//电机实时速度//
   }

   //lout::向文件夹中实时写入电机运行情况，周期与实时线程相同，为1ms一次//
   lout() << controller()->motorPool()[0].actualVel() <<"\t";
   lout() << controller()->motorPool()[0].actualPos() <<std::endl;

   return s1.getTc() * 1000-count(); //设置结束实时线程的时间，这里是梯形曲线的运行时间Tc//
}
auto tCurveDriveTogetherM3::collectNrt()->void {}
tCurveDriveTogetherM3::tCurveDriveTogetherM3(const std::string &name) 
{
   aris::core::fromXmlString(command(),
      "<Command name=\"t3\">"//运行本线程的指令tcurve_tgt_3//
      "	<GroupParam>"
      "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient3\" default=\"1\" abbreviation=\"h\"/>"//自定义参数的缩写//
      "	<Param name=\"intervals\" default=\"0.88\" abbreviation=\"j\"/>"//自定义参数的缩写//
      "	<Param name=\"acceleration\" default=\"5\" abbreviation=\"a\"/>"//自定义参数的缩写//
      "	<Param name=\"velocity\" default=\"2\" abbreviation=\"v\"/>"//自定义参数的缩写//
      "	</GroupParam>"
      "</Command>");
}
tCurveDriveTogetherM3::~tCurveDriveTogetherM3() = default; 

/////////////////////////////////////////////////< 速度模式 for motor number 2>///////////////////////////////////
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
        begin_vel[1] = controller()->motorPool()[1].actualVel();
        this->master()->logFileRawName("testVel");
    }
    
    double vel1= begin_vel[1]+cef_*5.0*(1-std::cos(2*PI*count()/2000.0))/2;
    
    controller()->motorPool()[1].setTargetVel(vel1);

    //屏幕打印与文件夹数据记录//
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[1].actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[1].actualVel() << std::endl;
    }
    
    lout() << controller()->motorPool()[1].actualPos() <<"\t";
    lout() << controller()->motorPool()[1].actualVel() <<std::endl;
    
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

////////////////////////////////////////////< 单关节正弦往复运动 for motor number 2>///////////////////////////////
struct MoveJSParam//数据结构//
{
    double j1;
    double time;
    uint32_t timenum;
};
auto MoveJS::prepareNrt()->void
{
    MoveJSParam param;//定义数据结构param//

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto &p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motorPool()[0].actualPos();
            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;

    std::vector<std::pair<std::string, std::any>> ret_value;

    for (auto &option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|NOT_CHECK_POS_CONTINUOUS;
    
    ret() = ret_value;
}
auto MoveJS::executeRT()->int{
    // 访问主站 //
    auto &param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;

    if ((1 <= count()) && (count() <= time / 2))
    {
        if (count() == 1)// 获取当前起始点位置 //
        {
            begin_pjs = controller()->motorPool()[1].actualPos();
            step_pjs = controller()->motorPool()[1].actualPos();
            this->master()->logFileRawName("moveJS");//建立记录数据的文件夹
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI*count() / time)) / 2;
        controller()->motorPool().at(1).setTargetPos(step_pjs);
    }

    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        if (count() == time / 2 + 1) // 获取当前起始点位置 //
        {
            begin_pjs = controller()->motorPool()[1].actualPos();
            step_pjs = controller()->motorPool()[1].actualPos();
        }

        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI*(count() - time / 2) / time)) / 2;
        controller()->motorPool().at(1).setTargetPos(step_pjs);
    }

    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {       
        if (count() == totaltime - time / 2 + 1)// 获取当前起始点位置 //
        {
            begin_pjs = controller()->motorPool()[1].actualPos();
            step_pjs = controller()->motorPool()[1].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI*(count() - totaltime + time / 2) / time)) / 2;
        controller()->motorPool().at(1).setTargetPos(step_pjs);
    }

    //屏幕打印与文件夹数据记录//
    if (count() % 10 == 0)
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;
    }

    lout() << controller()->motorPool()[1].actualPos() <<"\t";
    lout() << controller()->motorPool()[1].actualVel() <<std::endl;

    return totaltime - count();}
auto MoveJS::collectNrt()->void {}
MoveJS::MoveJS(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"js\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}

///////////////////////////////////////////////////////< verificate the singleLeg forward Kin >/////////////
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

///////////////////////////////////////////////////////< verificate the singleLeg inverse Kin >/////////////s
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

///////////////////////////////////////////////////<XXX tCurve梯形曲线 M1>//////////////////////////////////////////
auto tCurveDrive::prepareNrt()->void
{
    cef_ = doubleParam("coefficient");
    acc=doubleParam("acceleration");
    vel=doubleParam("velocity");
    totalTime=doubleParam("totalTime");
    motorNumber=int32Param("motorNumber");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::CHECK_NONE;    
    setAllMotorMaxTorque(ecMaster(), 100);
}
auto tCurveDrive::executeRT()->int 
{
    static double begin_angle[3];
    static double angle[3]{0.0};
    double judgeDirection[3]{0};

    if (count() == 1)
    {
        begin_angle[motorNumber] = controller()->motorPool()[motorNumber].actualPos();//将运动的初始位置设定为当下电机的实时位置//
    }

    tCurve s1(acc,vel); 

    s1.getCurveParam();

    judgeDirection[motorNumber]=s1.getClassifyNumber(count());
   
    angle[motorNumber] =  begin_angle[motorNumber] + judgeDirection[motorNumber] * PI * cef_  *  s1.getCurve(count())  ;//设置变量angle0存储由梯形曲线和公式运算所得的目标位置//

    controller()->motorPool()[motorNumber].setTargetPos(angle[motorNumber]);//调用aris函数将当下电机位置更新为angle0所标定位置//

    if (count() % 10 == 0)
    {   mout() << "the motor number is :" << motorNumber <<"\t";
        mout() << "judgeDirection:"<<judgeDirection[0] << "\t";
        mout() << "pos" << ":" << controller()->motorPool()[motorNumber].actualPos() << "\t";//电机实时位置//
        mout() << "vel" << ":" << controller()->motorPool()[motorNumber].actualVel() << std::endl;//电机实时速度//
        mout() << "cur" << ":" << controller()->motorPool()[motorNumber].actualCur() << "\t";//电机实时电流//
        mout() << "toq" << ":" << controller()->motorPool()[motorNumber].actualToq() << "\t";//电机实时力矩//
    }
    return totalTime * 1000-count(); 
}
auto tCurveDrive::collectNrt()->void {}
tCurveDrive::tCurveDrive(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"t\">"//运行本线程的指令//
       "	<GroupParam>"
       "	<Param name=\"coefficient\" default=\"1\" abbreviation=\"k\"/>"//自定义参数的缩写//
       "	<Param name=\"acceleration\" default=\"5\" abbreviation=\"a\"/>"//自定义梯形曲线加速度参量的缩写//
       "	<Param name=\"velocity\" default=\"2\" abbreviation=\"v\"/>"//自定义梯形曲线速度参量的缩写//
       "	<Param name=\"totalTime\" default=\"0.89\" abbreviation=\"t\"/>"//自定义梯形曲线运动总时间参量的缩写//
       "	<Param name=\"motorNumber\" default=\"0\" abbreviation=\"n\"/>"//自定义要运动的电机的序号0,1,2//
       "	</GroupParam>"
       "</Command>");
}
tCurveDrive::~tCurveDrive() = default;

///////////////////////////////////////////////////////<XXX cosCurve余弦曲线-任意数量电机串联同步测试 >///////////////////
auto cosCurveDriveMx::prepareNrt()->void
{
    cef_1 = doubleParam("coefficient1");
    cef_2 = doubleParam("coefficient2");
    cef_3 = doubleParam("coefficient3");
    totalTime=doubleParam("totalTime");
    motorNumber=int32Param("motorNumber");
    a_  =doubleParam("amplititude");
    w_  =doubleParam("frequency");
    p_  =doubleParam("phase");
    
    velocityVariable[0]=cef_1;
    velocityVariable[1]=cef_2;
    velocityVariable[2]=cef_3;


    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto cosCurveDriveMx::executeRT()->int
{
    begin_angle.resize(motorNumber);
    angle.resize(motorNumber);

    if (count() == 1)//实时线程刚开始运行-> count=1 时的设置//
    {
        for(int i=0 ; i < motorNumber ; ++i )
        {
        begin_angle[i] = controller()->motorPool()[i].targetPos();//将运动的初始位置设定为当下电机的实时位置//
        }
    }

    cosCurve cos1(a_,w_,p_); //创建对象，传入初始参数//

    for(int i=0 ; i < motorNumber ; ++i)
    {
        angle[i] = begin_angle[i] + 20 * velocityVariable[i] * cos1.getCurve(count()) ;//设置变量angle[i]存储由梯形曲线和公式运算所得的目标位置//
        controller()->motorPool()[0].setTargetPos(angle[i]);              //调用aris函数将当下电机位置更新为angle0所标定位置//
    }

    if (count() % 10 == 0)
    {
        std::cout<<"count = "<<count()<<" ; the motor <  pos:  "<< controller()->motorPool()[0].targetPos() << "   vel:  "<< controller()->motorPool()[0].targetVel()<< "  >"<<std::endl;
    }

    return  totalTime * 1000-count(); //设置结束实时线程的时间//
}
auto cosCurveDriveMx::collectNrt()->void {}
cosCurveDriveMx::cosCurveDriveMx(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"cmx\">"//运行本线程的指令//
       "	<GroupParam>"
       "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//control the motor 0's direction and displacement//
       "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//control the motor 1's direction and displacement//
       "	<Param name=\"coefficient3\" default=\"1\" abbreviation=\"h\"/>"//control the motor 2's direction and displacement//
       "	<Param name=\"totalTime\" default=\"3\" abbreviation=\"t\"/>"//define the total run time//
       "	<Param name=\"motorNumber\" default=\"1\" abbreviation=\"n\"/>"//define the total run time//       
       "	<Param name=\"amplititude\" default=\"1\" abbreviation=\"a\"/>"//自定义振幅参数a_的缩写//
       "	<Param name=\"frequency\" default=\"1\" abbreviation=\"w\"/>"//自定义角频率参数w_的缩写//
       "	<Param name=\"phase\" default=\"1.57\" abbreviation=\"p\"/>"//自定义时间参数p_的缩写//
       "	</GroupParam>"
       "</Command>");
}
cosCurveDriveMx::~cosCurveDriveMx() = default; 

///////////////////////////////////////////////////////<xxx cosCurve余弦曲线--run in interval M3>///////////////////
auto cosCurveDriveIntervalM3::prepareNrt()->void
{
    cef_1 = doubleParam("coefficient1");
    cef_2 = doubleParam("coefficient2");
    cef_3 = doubleParam("coefficient3");
    intervals=doubleParam("intervals");
    a_  =doubleParam("amplitude");
    w_  =doubleParam("frequency");
    p_  =doubleParam("phase");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto cosCurveDriveIntervalM3::executeRT()->int
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

    if ((1 <= count())&&( count()<=  (intervals * 1 * 1000) ))
    {
        double angle0 = begin_angle[0] + cef_1  * cos1.getCurve(count()) ;//设置变量angle0存储由梯形曲线和公式运算所得的目标位置//
        controller()->motorPool()[0].setTargetPos(angle0);              //调用aris函数将当下电机位置更新为angle0所标定位置//
    }
    else if ((intervals * 1 * 1000 < count() )&& (count()<= intervals * 2 * 1000))
    {
        double angle1 = begin_angle[1] + cef_2  * cos1.getCurve(count()- intervals * 1 * 1000) ;
        controller()->motorPool()[1].setTargetPos(angle1);
    }
    else
    {
        double angle2 = begin_angle[2] + cef_3  * cos1.getCurve(count() - intervals * 2 * 1000) ;
        controller()->motorPool()[2].setTargetPos(angle2);
    }


    if (count() % 10 == 0)//mout::在屏幕上实时打印电机运行情况，每10ms打印一次//
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";//电机实时位置//
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;//电机实时速度//
    }

    //lout::向文件夹中实时写入电机运行情况，周期与实时线程相同，为1ms一次//
    lout() << controller()->motorPool()[0].actualPos() <<std::endl;
    lout() << controller()->motorPool()[0].actualVel() <<std::endl;

    return  intervals * 3 * 1000-count(); //设置结束实时线程的时间//
}
auto cosCurveDriveIntervalM3::collectNrt()->void {}
cosCurveDriveIntervalM3::cosCurveDriveIntervalM3(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ci3\">"//运行本线程的指令ccurve_interval_M3//
       "	<GroupParam>"
       "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//control the motor 0's direction and displacement//
       "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//control the motor 1's direction and displacement//
       "	<Param name=\"coefficient3\" default=\"1\" abbreviation=\"h\"/>"//control the motor 2's direction and displacement//
       "	<Param name=\"intervals\" default=\"1\" abbreviation=\"j\"/>"//define the interval of two motions//
       "	<Param name=\"amplitude\" default=\"1\" abbreviation=\"a\"/>"//自定义振幅参数a_的缩写//
       "	<Param name=\"frequency\" default=\"1\" abbreviation=\"w\"/>"//自定义角频率参数w_的缩写//
       "	<Param name=\"phase\" default=\"1.57\" abbreviation=\"p\"/>"//自定义时间参数p_的缩写//
       "	</GroupParam>"
       "</Command>");
}
cosCurveDriveIntervalM3::~cosCurveDriveIntervalM3() = default;  

///////////////////////////////////////////////////////<XXX moveBeeLine >///////////////////////////////////////////
auto moveBeeLine::prepareNrt()->void
{
    moveDir = doubleParam("moveDir");
    moveTime = doubleParam("moveTime");
    speedControlVariable = doubleParam("speedControlVariable");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto moveBeeLine::executeRT()->int
{
    if (count() == 1)
    {
        mout() << "pos of joint0" << ":" << controller()->motorPool()[0].targetPos() << std::endl;
        mout() << "pos of joint1" << ":" << controller()->motorPool()[1].targetPos() << std::endl;
        mout() << "pos of joint2" << ":" << controller()->motorPool()[2].targetPos() << std::endl;       
        std::cout << "Step 2 Started! The Leg has prepared to move !"<< std::endl; 

        double angleOfStatus1[3]{0};
        angleOfStatus1[0] = controller()->motorPool()[0].targetPos();
        angleOfStatus1[1] = controller()->motorPool()[1].targetPos();
        angleOfStatus1[2] = controller()->motorPool()[2].targetPos();

        //Solve the Forward Kinematics
        model()->setInputPos(angleOfStatus1) ;    

        if (model()->solverPool()[1].kinPos())
        {
            throw std::runtime_error("Leg Status1 Forward Kinematics Position Failed!");
        }
       
        double posOfStatus1[3]{0};
        model()->getOutputPos(posOfStatus1);
        std::cout<<"EE Pos Coordinates of Status 1 : <"<<"X:"<<posOfStatus1[0]<<"\t"<<"Y:"<<posOfStatus1[1]<<"\t"<<"Z:"<<posOfStatus1[2]<<" >"<<std::endl <<std::endl;     
        
        for (int i = 0; i < 3; i++)
        {
            posOfMoveStatus[i]=posOfStatus1[i]; 
        }
    }

    posOfMoveStatus[2] = posOfMoveStatus[2] + count() * 0.00000001 * speedControlVariable ;

    std::cout<<"count = "<< count() <<" -->"<<"EE Pos Coordinates of Move Status to get invKin slove : <"<<"X:"<<posOfMoveStatus[0]<<"\t"<<"Y:"<<posOfMoveStatus[1]<<"\t"<<"Z:"<<posOfMoveStatus[2]<<" >"<<std::endl;     

    model()->setOutputPos(posOfMoveStatus);

    if (model()->solverPool()[0].kinPos())
    {
        throw std::runtime_error("Move Status Inverse kinematic position failed");    
    }        

    controller()->motorPool()[0].setTargetPos(model()->motionPool()[0].mp());
    controller()->motorPool()[1].setTargetPos(model()->motionPool()[1].mp());
    controller()->motorPool()[2].setTargetPos(model()->motionPool()[2].mp());
    
	std::cout << "Target Angle of Motor Joint  " <<"<--  Joint0 : "<<model()->motionPool()[0].mp() << "\t" <<"Joint 1 : "<< model()->motionPool()[1].mp() << "\t" <<"Joint 2 : "<< model()->motionPool()[2].mp() << std::endl << std::endl;
    return  ( moveTime * 1000 ) -count(); 
}
auto moveBeeLine::collectNrt()->void {}
moveBeeLine::moveBeeLine(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"m1\">"
       "	<GroupParam>"                                    
       "	<Param name=\"moveDir\" default=\"1\" abbreviation=\"d\"/>"//单腿的直线轨迹运行的方向，轨迹为平行于Z轴的直线，方向正负与Z轴正负相同//
       "	<Param name=\"moveTime\" default=\"3\" abbreviation=\"t\"/>"//与计时count()有关的运动时间，单位s//
       "	<Param name=\"speedControlVariable\" default=\"1.0\" abbreviation=\"j\"/>"//调节直线轨迹的变化速度//
       "	</GroupParam>"
       "</Command>");
}
moveBeeLine::~moveBeeLine() = default; 

///////////////////////////////////////////////////<xxx anyExecuteTest >//////////////////////////////////////////
auto anyExecuteTest::prepareNrt()->void
{
   cef_1 = doubleParam("coefficient1");
   cef_2 = doubleParam("coefficient2");
   cef_3 = doubleParam("coefficient3");
   intervals=int32Param("intervals");
   vel   = doubleParam("velocity");
   acc   = doubleParam("acceleration");

   for(auto &m:motorOptions()) m =
           aris::plan::Plan::NOT_CHECK_ENABLE |
           aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto anyExecuteTest::executeRT()->int
{
   static double begin_angle[3];

   if (count() == 1)
   {
       begin_angle[0] = controller()->motorPool()[0].targetPos();
       begin_angle[1] = controller()->motorPool()[1].targetPos();
       begin_angle[2] = controller()->motorPool()[2].targetPos();
   }
    double angle2 = begin_angle[2] + count() * 0.001 ;
    controller()->motorPool()[2].setTargetPos(angle2);   
   if (count() % 10 == 0)
   {
       mout() << "pos" << ":" << controller()->motorPool()[2].actualPos() << std::endl;//电机实时位置//
   }
   return intervals -count();
}
auto anyExecuteTest::collectNrt()->void {}
anyExecuteTest::anyExecuteTest(const std::string &name)
{
   aris::core::fromXmlString(command(),
      "<Command name=\"exe\">"//运行本线程的指令//
      "	<GroupParam>"
      "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient3\" default=\"1\" abbreviation=\"h\"/>"//自定义参数的缩写//
      "	<Param name=\"intervals\" default=\"1571\" abbreviation=\"j\"/>"//自定义参数的缩写//
      "	</GroupParam>"
      "</Command>");
}
anyExecuteTest::~anyExecuteTest() = default; 

///////////////////////////////////////////////////<xxx tCurve梯形曲线--run in interval M3 >///////////////////////////
auto tCurveDriveIntervalM3::prepareNrt()->void
{
    cef_1 = doubleParam("coefficient1");
    cef_2 = doubleParam("coefficient2");
    cef_3 = doubleParam("coefficient3");
    vel   = doubleParam("velocity");
    acc   = doubleParam("acceleration");
    intervals=doubleParam("intervals");

    for(auto &m:motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_ENABLE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto tCurveDriveIntervalM3::executeRT()->int
{
    static double begin_angle[3];

    if (count() == 1)//实时线程刚开始运行-> count=1 时的设置//
    {
        begin_angle[0] = controller()->motorPool()[0].targetPos();//将运动的初始位置设定为当下电机的实时位置//
        begin_angle[1] = controller()->motorPool()[1].targetPos();
        begin_angle[2] = controller()->motorPool()[2].targetPos();

        this->master()->logFileRawName("testTCurve");//建立记录数据的文件夹//
    }

    tCurve s1(acc,vel); //梯形曲线构造函数，传入初始参数//

    s1.getCurveParam();//调用函数，由初始的参数得到梯形曲线运行的所需参数//

    if ((1 <=count())&& (count()<= intervals * 1 * 1000 ))
    {
        double angle0 = begin_angle[0] + cef_1  * s1.getCurve(count()) ;//设置变量angle0存储由梯形曲线和公式运算所得的目标位置//
        controller()->motorPool()[0].setTargetPos(angle0);              //调用aris函数将当下电机位置更新为angle0所标定位置//
    }
    else if ((intervals * 1 * 1000 < count()) && (count()<= intervals * 2 * 1000))
    {
        double angle1 = begin_angle[1] + cef_2  * s1.getCurve(count()- intervals * 1 * 1000) ;
        controller()->motorPool()[1].setTargetPos(angle1);
    }
    else
    {
        double angle2 = begin_angle[2] + cef_3  * s1.getCurve(count() - intervals * 2 * 1000) ;
        controller()->motorPool()[2].setTargetPos(angle2);
    }

    if (count() % 10 == 0)//mout::在屏幕上实时打印电机运行情况，每10ms打印一次//
    {
        mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";//电机实时位置//
        mout() << "cur" << ":" << controller()->motorPool()[0].actualCur() << "\t";//电机实时电流//
        mout() << "toq" << ":" << controller()->motorPool()[0].actualToq() << "\t";//电机实时力矩//
        mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;//电机实时速度//
    }

    //lout::向文件夹中实时写入电机运行情况，周期与实时线程相同，为1ms一次//
    lout() << controller()->motorPool()[0].actualVel() <<"\t";
    lout() << controller()->motorPool()[0].actualPos() <<std::endl;

    return intervals * 3 * 1000-count(); //设置结束实时线程的时间，这里是梯形曲线的运行时间Tc//
}
auto tCurveDriveIntervalM3::collectNrt()->void {}
tCurveDriveIntervalM3::tCurveDriveIntervalM3(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ti3\">"//运行本线程的指令tcurve_itv_3//
       "	<GroupParam>"
       "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//the direction and displacement of motor 0//
       "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//the direction and displacement of motor 1//
       "	<Param name=\"coefficient3\" default=\"1\" abbreviation=\"h\"/>"//the direction and displacement of motor 2//
       "	<Param name=\"intervals\" default=\"0.88\" abbreviation=\"j\"/>"//the interval of 2 motors//
       "	<Param name=\"acceleration\" default=\"5\" abbreviation=\"a\"/>"//the acceleration of tcurve//
       "	<Param name=\"velocity\" default=\"2\" abbreviation=\"v\"/>"//the velocity of tcurve //
       "	</GroupParam>"                                              //Tc=(a+v*v)/(a*v) ; [ only (v*v)/a<=1 ] => tcurve could be built; //
       "</Command>");

}
tCurveDriveIntervalM3::~tCurveDriveIntervalM3() = default; 

///////////////////////////////////////////////////////<xxx cosCurve余弦曲线-run together M2 >///////////////////////////
auto cosCurveDriveTogetherM2::prepareNrt()->void
{
   cef_1 = doubleParam("coefficient1");
   cef_2 = doubleParam("coefficient2");
   totalTime=doubleParam("totalTime");
   a_  =doubleParam("amplitude");
   w_  =doubleParam("frequency");
   p_  =doubleParam("phase");

   for(auto &m:motorOptions()) m =
           aris::plan::Plan::NOT_CHECK_ENABLE |
           aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}
auto cosCurveDriveTogetherM2::executeRT()->int
{
   static double begin_angle[3];

   if (count() == 1)//实时线程刚开始运行-> count=1 时的设置//
   {
       begin_angle[0] = controller()->motorPool()[0].targetPos();//将运动的初始位置设定为当下电机的实时位置//
       begin_angle[1] = controller()->motorPool()[1].targetPos();
   }

   cosCurve cos1(a_,w_,p_); //创建对象，传入初始参数//

   double angle0 = begin_angle[0] + cef_1  * cos1.getCurve(count()) ;//设置变量angle0存储由余弦曲线和公式运算所得的目标位置//
   double angle1 = begin_angle[1] + cef_2  * cos1.getCurve(count()) ;

   controller()->motorPool()[0].setTargetPos(angle0);//调用aris函数将当下电机位置更新为angle0所标定位置//
   controller()->motorPool()[1].setTargetPos(angle1);

   if (count() % 10 == 0)//mout::在屏幕上实时打印电机运行情况，每10ms打印一次//
   {
       mout() << "pos" << ":" << controller()->motorPool()[0].actualPos() << "\t";//电机实时位置//
       mout() << "vel" << ":" << controller()->motorPool()[0].actualVel() << std::endl;//电机实时速度//
   }

   return  totalTime * 1000 - count(); //设置结束实时线程的时间//
}
auto cosCurveDriveTogetherM2::collectNrt()->void {}
cosCurveDriveTogetherM2::cosCurveDriveTogetherM2(const std::string &name) 
{
   aris::core::fromXmlString(command(),
      "<Command name=\"c2\">"//运行本线程的指令//
      "	<GroupParam>"
      "	<Param name=\"coefficient1\" default=\"1\" abbreviation=\"f\"/>"//自定义参数的缩写//
      "	<Param name=\"coefficient2\" default=\"1\" abbreviation=\"g\"/>"//自定义参数的缩写//
      "	<Param name=\"totalTime\" default=\"3\" abbreviation=\"t\"/>"//自定义参数的缩写//
      "	<Param name=\"amplitude\" default=\"1\" abbreviation=\"a\"/>"//自定义振幅参数a_的缩写//
      "	<Param name=\"frequency\" default=\"1\" abbreviation=\"w\"/>"//自定义角频率参数w_的缩写//
      "	<Param name=\"phase\" default=\"1.57\" abbreviation=\"p\"/>"//自定义时间参数t_的缩写//
      "	</GroupParam>"
      "</Command>");
}
cosCurveDriveTogetherM2::~cosCurveDriveTogetherM2() = default; 

///////////////////////////////////////////////最大力矩设置/////////////////////////////////////////////////////////
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

    for (aris::Size i = 0; i < 3 ; ++i){
        int phy_id[3]={0,1,2};

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

    for (aris::Size i = 0; i < 3 ; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[3]
        {
            0,0,0
        };
#else
        double pos_offset[3]
        {
            // 0, 0, 0
            -1.4974336956172, 0.128106570548551, 0.844257485597249
            // -0.894181369710104, 0.119132782939402, 0.844199961317703
        };
#endif
        double pos_factor[3] //偏置系数//
        {
            // 2000/PI,2000/PI,2000/PI
            131072*5/PI,131072*5/PI,131072*10/PI
        };
        double max_pos[3] //最大位置//
        {
            // 500*PI,500*PI,500*PI  
            // PI/6, PI/2, 2 * PI/3
            1.6, 1.5, 2.2

        };
        double min_pos[3] //最小位置//
        {
            // -500*PI,-500*PI,-500*PI
            -0.6, -0.4, -1.8
        };
        double max_vel[3]  //最大速度//
        {
            // 330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
            1, 1, 1
        };
        double max_acc[3]  //最大加速度//
        {
            // 3000,  3000,  3000
            10, 10, 10
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
    plan_root->planPool().add<tCurveDrive>();
    plan_root->planPool().add<VelDrive>();
    plan_root->planPool().add<MoveJS>(); 
    plan_root->planPool().add<cosCurveDriveMx>();
    plan_root->planPool().add<tCurveDriveTogetherM3>();
    plan_root->planPool().add<cosCurveDriveTogetherM3>();
    plan_root->planPool().add<cosCurveDriveTogetherM2>();
    plan_root->planPool().add<tCurveDriveIntervalM3>();
    plan_root->planPool().add<cosCurveDriveIntervalM3>();    
    plan_root->planPool().add<inverseKinLegTest>();
    plan_root->planPool().add<forwardKinLegTest>();
    plan_root->planPool().add<anyExecuteTest>();
    plan_root->planPool().add<legInitialization>();
    plan_root->planPool().add<moveBeeLine>();
    plan_root->planPool().add<moveBeeLineE2>();
    plan_root->planPool().add<ellipticalTrajectoryDrive>();
    plan_root->planPool().add<ellipticalTrajectoryDrive2>();
    plan_root->planPool().add<ellipticalTrajectoryDrive3>();

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