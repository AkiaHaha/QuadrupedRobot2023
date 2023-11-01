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
///////////////////////////////////////////////////////< single motor test >/////////////////////////////////////
auto SingleMotorTest::prepareNrt()->void
{
    angle_ = doubleParam("angle");
    for(auto &m:motorOptions()) m =
            aris::plan::Plan::CHECK_NONE;
}
auto SingleMotorTest::executeRT()->int
{
    static double begin_angle;
    int iNumber =int(std::abs(angle_) * 10000) ;
    int dir = int( std::abs(angle_) /  angle_ );

    if (count() == 1)
    {
        begin_angle = controller()->motorPool()[0].actualPos();
    }


    if (count() % 10 == 0)
    {
        std::cout << "count = " << count() <<std::endl;
        std::cout << "< pos " << ":" << controller()->motorPool()[0].actualPos() << "\t";
        std::cout << "vel" << ":" << controller()->motorPool()[0].actualVel() <<" >"<< std::endl << std::endl;
    }
    double angle = begin_angle + count() * 0.0001 * dir;
    controller()->motorPool()[0].setTargetPos(angle); 

    return  iNumber - count(); 
}
auto SingleMotorTest::collectNrt()->void {}
SingleMotorTest::SingleMotorTest(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"t\">"
       "	<GroupParam>"                                    
       "	<Param name=\"angle\" default=\"0.1\" abbreviation=\"k\"/>"//通过给定电机初始值初始化单腿的末端位姿,给定值为弧度制，本程序中精度设定给到小数点后三位//
       "	</GroupParam>"
       "</Command>");
}
SingleMotorTest::~SingleMotorTest() = default; 

///////////////////////////////////////////////////////< Leg Initialization >/////////////////////////////////////
auto legInitialization::prepareNrt()->void
{

    initAngle0 = doubleParam("initAngle0");
    initAngle1 = doubleParam("initAngle1");
    initAngle2 = doubleParam("initAngle2");

    for(auto &m:motorOptions()) m =
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



/////////////////////////////////////////////////< Velocity  mode >///////////////////////////////////
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


///////////////////////////////////////////////< set max torque >/////////////////////////////////////////////////////////
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

    for (aris::Size i = 0; i < 1 ; ++i){
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

    for (aris::Size i = 0; i < 1 ; ++i)
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
    plan_root->planPool().add<VelDrive>();
    plan_root->planPool().add<MoveJS>(); 
    plan_root->planPool().add<legInitialization>();
    plan_root->planPool().add<SingleMotorTest>();
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