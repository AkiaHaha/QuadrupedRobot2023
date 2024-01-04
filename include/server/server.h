#ifndef QRBT_SERVER_H
#define QRBT_SERVER_H
#include <aris.hpp>
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include "control/robot.h"
#include "model/model.h"
#include "tools/operator.h"
#include <memory>
#include "tools/operator.h"
#include "control/plan.h"
#include "tools/json.hpp"
#include "control/robot.h"
#include "control/plan.h"
#include "server/server.h"
#include "model/model.h"
#include "tools/operator.h"
using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;

namespace robot {
  class DogGet : public aris::core::CloneObject<DogGet, aris::plan::Plan>
  {
  public:
    auto virtual prepareNrt()->void;
    auto virtual collectNrt()->void;
    explicit DogGet(const std::string& name = "DogGet_plan");
    // ARIS_DEFINE_BIG_FOUR(DogGet);
    // ����в�������Ҫָ����Ӧ�ĳ�Ա����������ȡ��ʹ�ò���
  };

  /// <summary>
  /// define motor drive pdo and sequence of motor pool
  /// </summary>
  auto createMasterROSMotorTest() -> std::unique_ptr<aris::control::Master>;

  /// <summary>
  /// set motor defult setting of drive
  /// </summary>
  auto createControllerROSMotorTest() -> std::unique_ptr<aris::control::Controller>;

  /// <summary>
  /// plan pool of command
  /// </summary>
  auto createPlanROSMotorTest() -> std::unique_ptr<aris::plan::PlanRoot>;

  /// <summary>
  /// set all motors max torque
  /// </summary>
  auto setAllMotorMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value) -> void;

  /// <summary>
  /// set single motor max torque
  /// </summary>
  auto setMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value, size_t index) -> bool;
}
#endif // !QRBT_SERVER_H