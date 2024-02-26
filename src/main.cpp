#include <aris.hpp>
#include "control/Robot.h"
#include "model/Model.h"
#include "control/Plan.h"
#include "server/Server.h"
#include "motor/PidController.h"
#include "tools/Operator.h"

using namespace robot;
auto xmlpath = std::filesystem::absolute(".");	//获取当前工程所在的路径
const std::string xmlfile = "dog.xml";
struct Data{
    double pos[12]{};
    double pos_init[12]{};
    double vel[12]{};
    double vel_init[12]{};
    double toq_cmd[12]{};
    double flag_start[12]{};
};
int main(int argc, char *argv[])
{
    // auto&cs = aris::server::ControlServer::instance();

    // auto master = new aris::control::EthercatMaster;

    // master->scan();

    // cs.resetMaster(robot::createMasterROSMotorTest().release());

    // std::cout << aris::core::toXmlString(cs.master()) << std::endl;

    // cs.resetController(robot::createControllerROSMotorTest().release());

    // cs.resetPlanRoot(robot::createPlanROSMotorTest().release());

    // // cs.resetModel(robot::createSingleLegModelPtr().release());
    // cs.resetModel(robot::createQuadrupedRbtModelPtr().release());


    // cs.resetMiddleWare(std::unique_ptr<aris::server::ProgramMiddleware>(new aris::server::ProgramMiddleware).release()); // 需要引入ProgramMiddleware才有一些指令支持

    // //网页控制代码
    // cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::Type::WEB);
    // cs.interfacePool().add<aris::server::HttpInterface>("", "8001", "X:\\GitRepo\\QuadrupedRobot2023\\build");
    // cs.middleWare();

    // cs.init();//初始化WebSocket/socket服务器//
    
    // cs.open();//开启WebSocket/socket服务器//

    // //cs.start();//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
    
    // try {
    // cs.start();
    // // cs.executeCmd("set_max_trq");
    // #ifdef ARIS_USE_ETHERCAT_SIMULATION
    //     cs.executeCmd("ds -a");
    //     cs.executeCmd("md -a");
    //     cs.executeCmd("en -a");
    // #endif
    // }

    // catch (const std::exception& err) {
    //     std::cout << "failed to start system, please reboot" << std::endl;
    // }

    // cs.runCmdLine();//开启命令行终端，先显示Pdo信息，然后在跳出界面输入指令进行操作//

        xmlpath = xmlpath / xmlfile;
        auto& cs = aris::server::ControlServer::instance();

        std::cout << "xmlpath:" << xmlpath << std::endl;

        aris::core::fromXmlFile(cs, xmlpath);

        cs.init();

// std::cout << &cs.model() << std::endl;
//     for (auto& plan: cs.planRoot().planPool())
//     {
//   //    plan.setModelBase(&cs.model());
//       std::cout << plan.modelBase() << std::endl;
//     }


       //开启控制器服务
       try {
           cs.start();
           cs.executeCmd("md");
       }
       catch (const std::exception& err) {
           std::cout << "系统启动失败" << std::endl;
       }
        // 开启WebSocket/socket服务器//
        cs.open(); 


        

        // aris::dynamic::MultiModel mm;
        // mm.subModels().push_back(robot::createQuadrupedRbtModelPtr2().release());
        // cs.resetModel(&mm);
        // std::cout << aris::core::toXmlString(cs.model()) << std::endl;

//        std::cout << "Qq" << std::endl;

//        std::unique_ptr<aris::dynamic::MultiModel> mm(new aris::dynamic::MultiModel);
//        aris::dynamic::AbenicsParam param;
//        auto m = aris::dynamic::createModelAbenics(param);
//        auto m2 = robot::createQuadrupedRbtModelPtr();
//        mm->subModels().push_back(m.release());
//        std::cout << "aa" << std::endl;
//            cs.resetModel(mm.release());
//            cs.model().init();
//    //  std::cout << "sss" << std::endl;
//            std::cout << aris::core::toXmlString(cs.model()) << std::endl;

        //实时回调函数，每个实时运行周期都运行一次//
        for(std::size_t i = 0; i < cs.controller().motorPool().size(); ++i)
        {
            cs.idleMotionCheckOption()[i] |= aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
        }

        cs.setRtPlanPreCallback([] (aris::server::ControlServer&cs){
            static Data data  ;
            auto ptr = cs.currentExecutePlan();
            robot::PController PdPosController{15, 1};
            if(!ptr){
                for(size_t i = 0; i < 12; i++){

                    auto& cm = cs.controller().motorPool()[i] ;

                    // if((cm.statusWord() & 0x6f) != 0x27) //ds
                    // {
                    //     std::cout << "ds " << i << std::endl;
                    // }
            
                        cm.setModeOfOperation(10);
                        data.pos[i] = cm.actualPos();
                        data.vel[i] = cm.actualVel();

                        if(data.flag_start[i] == 0){
                            data.pos_init[i] = cm.actualPos();
                            data.vel_init[i] = cm.actualVel();
                            cm.setTargetVel(cm.actualVel());
                            cm.setTargetPos(cm.actualPos());
                            data.flag_start[i] = 1;
                            std::cout << "motor " << i << "start -> " << std::endl;
                        }

                        data.toq_cmd[i] = PdPosController.getTargetOut(0, data.pos_init[i], data.pos[i], 0, data.vel[i]);

                        cm.setTargetToq(data.toq_cmd[i] * ktoqFactor);
                    

                    // cm.setTargetVel(data.vel[i]);
                }
            }
            // kaanh::robot::Robot::instanceInCs().rtUpdate(cs);
        });

        //等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
        cs.runCmdLine();
    return 0;
}
