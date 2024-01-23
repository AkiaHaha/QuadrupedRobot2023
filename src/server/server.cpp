#include "server/server.h"
#include "test/MotorTest.h"
#include "motor/PIDTest.h"
#include "fsm/BasicStateControl.h"


namespace robot {
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
  auto get_state_code() -> std::int32_t
    //获取状态字——100:去使能,200:手动,300:自动,400:程序运行中,410:程序暂停中,420:程序停止，500:错误
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
  //2023.12.19 copy yw code of xml model visulization and test//

  auto createMasterROSMotorTest() -> std::unique_ptr<aris::control::Master> {
    std::unique_ptr<aris::control::Master> master(new aris::control::EthercatMaster);

    for (aris::Size i = 0; i < 12; ++i) {
      int phy_id[12] = { 8,9,10,5,3,4,6,7,11,0,1,2 };
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
  auto createControllerROSMotorTest() -> std::unique_ptr<aris::control::Controller>
  {
    std::unique_ptr<aris::control::Controller> controller(new aris::control::Controller);

    for (aris::Size i = 0; i < 12; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
      double pos_offset[12]
      {
        // 0, 0, 0,
        // 0, 0, 0,
        // 0, 0, 0,
        // 0, 0, 0,       
        -PI / 2, -PI / 4, -PI / 2,
        -PI / 2, -PI / 4, -PI / 2,
         PI / 2, -PI / 4, -PI / 2,
         PI / 2, -PI / 4, -PI / 2,
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
        131072 * 5 / PI,131072 * 5 / PI,131072 * 10 / PI,
        131072 * 5 / PI,131072 * 5 / PI,131072 * 10 / PI,
        131072 * 5 / PI,131072 * 5 / PI,131072 * 10 / PI,
        131072 * 5 / PI,131072 * 5 / PI,131072 * 10 / PI,

      };
      double max_pos[12] //最大位置//
      {
        // 500*PI,500*PI,500*PI  
        // PI/6, PI/2, 2 * PI/3
        // 1.6, 1.5, 2.2,
        // 1.6, 1.5, 2.2,
        // 1.6, 1.5, 2.2,
        // 1.6, 1.5, 2.2,
        500 * PI,500 * PI,500 * PI,
        500 * PI,500 * PI,500 * PI,
        500 * PI,500 * PI,500 * PI,
        500 * PI,500 * PI,500 * PI,

      };
      double min_pos[12] //最小位置//
      {
          -500 * PI,-500 * PI,-500 * PI,
          -500 * PI,-500 * PI,-500 * PI,
          -500 * PI,-500 * PI,-500 * PI,
          -500 * PI,-500 * PI,-500 * PI,
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
        " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\" slave=\"" + std::to_string(i) + "\">"
        "</EthercatMotor>";

      auto& s = controller->motorPool().add<aris::control::EthercatMotor>();
      aris::core::fromXmlString(s, xml_str);
      s.setEnableWaitingCount(100);
    };
    return controller;
  }
  auto createPlanROSMotorTest() -> std::unique_ptr<aris::plan::PlanRoot>
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
    auto& rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");
    auto& mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");
    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();
    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //-------------自己写的命令-----------------//
    plan_root->planPool().add<robot::Ellipse4LegDrive3>();
    plan_root->planPool().add<robot::TrotMove>();
    plan_root->planPool().add<robot::ReadInformation>();
    plan_root->planPool().add<robot::ModelMotorInitialize>();
    plan_root->planPool().add<robot::MotorTest>();
    plan_root->planPool().add<robot::SetMotorPosZero>();
    plan_root->planPool().add<robot::PidVelCtrl>();
    plan_root->planPool().add<robot::PidPosVelCtrl>();
    plan_root->planPool().add<robot::PidPosVelToqCtrl>(); 
    plan_root->planPool().add<robot::ToqTest>(); 
    plan_root->planPool().add<StatePassive2Stand>();

    return plan_root;
  }
  auto setMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value, size_t index) -> bool
  {
    return (ecMaster->slavePool()[index].writePdo(0x6072, 0x00, std::uint16_t(value)) ? true : false);
  }
  auto setAllMotorMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value) -> void
  {
    for (int i = 0; i < ecMaster->slavePool().size(); ++i)
    {
      if (setMaxTorque(ecMaster, 100, i))
      {
        std::cout << "Set Motor " << i << " max torque failed!" << std::endl;
      }
    }
  }
  auto setOperationMode(aris::control::Controller* controller, std::uint8_t mode, size_t index) -> bool {
    if (controller->motorPool()[index].modeOfOperation() == mode) {
      return true;
    }
    controller->motorPool()[index].setModeOfOperation(mode);
    if (controller->motorPool()[index].modeOfOperation() == mode) {
      std::cout <<  "Switch mode to :" << mode << std::endl;
      return true;
    }
    return false;
  }
}