#include "robot.h"
#include <aris.hpp>
#include "model.h"

int main(int argc, char *argv[])
{
    auto&cs = aris::server::ControlServer::instance();

    auto master = new aris::control::EthercatMaster;

    master->scan();

    cs.resetMaster(robot::createMasterROSMotorTest().release());

    std::cout << aris::core::toXmlString(cs.master()) << std::endl;

    cs.resetController(robot::createControllerROSMotorTest().release());

    cs.resetPlanRoot(robot::createPlanROSMotorTest().release());

    cs.resetModel(robot::createSingleLegModelPtr().release());


    cs.init();//初始化WebSocket/socket服务器//
    
    cs.open();//开启WebSocket/socket服务器//

    //cs.start();//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
    
    try {
    cs.start();
    cs.executeCmd("set_max_trq");
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