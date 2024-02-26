#ifndef QRBT_SERVER_H
#define QRBT_SERVER_H
#include <aris.hpp>
#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <math.h>
#include <memory>
#include "tools/Operator.h"
#include "control/Plan.h"
#include "tools/json.hpp"
#include "control/Robot.h"
#include "control/Plan.h"
#include "server/Server.h"
#include "model/Model.h"
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

  /// <summary>
  /// functions from GJ
  /// </summary>
  /// <param name="controller"></param>
  /// <param name="mode"></param>
  /// <param name="index"></param>
  /// <returns></returns>
  auto setOperationMode(aris::control::Controller* controller, std::uint8_t mode, size_t index) -> bool;
  auto setAllOperationMode(aris::control::Controller* controller, std::uint8_t mode) -> bool;


//--------------------------------------/ 
    class  MotorConfig :public aris::control::EthercatMotor
    {

    public:

        auto setFeedConstant(double feed_constant)->void;
        auto getFeedConstant()const->double;

        auto setGearRatio(double gear_ratio)->void;
        auto getGearRatio()const->double;

        auto setResolution(int resolution)->void;
        auto getResolution()const ->int;

        auto setDirection(int direction)->void;
        auto getDirection()const ->int;

        auto setMotionType(std::string motion_type)->void;
        auto getMotionType()const->std::string;

    private:

        struct Imp;

        aris::core::ImpPtr<Imp>imp_;

    public:

        virtual ~MotorConfig();

        MotorConfig(aris::control::EthercatSlave* slave = nullptr

            , double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0

            , double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0

            , double unit_factor = 180.0 / aris::PI, double feed_constant = 360.0, double gear_ratio = 1.0, int resolution = 131072, int direction = 1, int motion_type = 1);

        MotorConfig(const MotorConfig& other) = delete;

        MotorConfig(EthercatMotor&& other) = delete;

        MotorConfig& operator=(const MotorConfig& other) = delete;

        MotorConfig& operator=(MotorConfig&& other) = delete;

    };

}
#endif // !QRBT_SERVER_H
