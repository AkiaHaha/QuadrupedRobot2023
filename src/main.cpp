#include <aris.hpp>
#include "control/robot.h"
#include "model/model.h"
#include "control/plan.h"
#include "server/server.h"

using namespace robot;

int main(int argc, char *argv[])
{
    auto&cs = aris::server::ControlServer::instance();

    auto master = new aris::control::EthercatMaster;

    master->scan();

    cs.resetMaster(robot::createMasterROSMotorTest().release());

    std::cout << aris::core::toXmlString(cs.master()) << std::endl;

    cs.resetController(robot::createControllerROSMotorTest().release());

    cs.resetPlanRoot(robot::createPlanROSMotorTest().release());

    // cs.resetModel(robot::createSingleLegModelPtr().release());
    cs.resetModel(robot::createQuadrupedRbtModelPtr().release());


    cs.resetMiddleWare(std::unique_ptr<aris::server::ProgramMiddleware>(new aris::server::ProgramMiddleware).release()); // 需要引入ProgramMiddleware才有一些指令支持

    //网页控制代码
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::Type::WEB);
    cs.interfacePool().add<aris::server::HttpInterface>("", "8001", "X:\\GitRepo\\QuadrupedRobot2023\\build");
    cs.middleWare();

    cs.init();//初始化WebSocket/socket服务器//
    
    cs.open();//开启WebSocket/socket服务器//

    //cs.start();//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
    
    try {
    cs.start();
    // cs.executeCmd("set_max_trq");
    // cs.executeCmd("ds -a");
    // cs.executeCmd("md -a");
    // cs.executeCmd("en -a");
    }

    catch (const std::exception& err) {
        std::cout << "failed to start system, please reboot" << std::endl;
    }

    cs.runCmdLine();//开启命令行终端，先显示Pdo信息，然后在跳出界面输入指令进行操作//

    return 0;
}