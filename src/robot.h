#ifndef ROBOT_H_
#define ROBOT_H_
#include <memory>
#include "aris.hpp"
#include "operator.h"
#include "plan.h"
#include "json.hpp"
#define ARIS_USE_ETHERCAT_SIMULATION

namespace robot
{   
  /// <summary>
  /// Trot move
  /// </summary>
  class  TrotMove : public aris::core::CloneObject<TrotMove, aris::plan::Plan> {
  public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    virtual ~TrotMove();
    explicit TrotMove(const std::string& name = "TrotMove");

  private:
    double vel_x{};
    double vel_z{};
    double vel_h{};
    int16_t total_tn{};


    double init_m28[30]{};
    double* move_m28{};
    double move_mb[16]{};
    double move_pee[12]{};
    double move_motor_pos[12]{};
    double init_motor_pos[12]{};
    double period_init_m28[28]{};
    
    int count_stop{};
    int16_t period_n{};
    int32_t time_in_pn{};

    bool switch_number{};
    //EllipseMovePlan ep;
  };

    /// <summary>
    /// 4 legs run ellipse curve together
    /// </summary>
    class Ellipse4LegDrive3 : public aris::core::CloneObject<Ellipse4LegDrive3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~Ellipse4LegDrive3();
        explicit Ellipse4LegDrive3(const std::string &name = "Ellipse4LegDrive3");

    private:
        double moveX_{};
        double moveY_{};
        double moveZ_{};
        double Height_{};

        double theta_{};
        double theta_d_{};
        double theta_dd_{};

        double startMotorPos[12]{};
        double moveMotorPos[12]{};

        double startModelPE[28]{};
        double finalModelPE[28]{};
        double moveModelPE[28]{};

        double startBodyPose[16]{};
        double finalBodyPose[16]{};
        double moveBodyPose[16]{};

        double startLegPoint[12]{};
        double finalLegPoint[12]{};
        double moveLegPoint[12]{};
    };

    /// <summary>
    /// read motor pos and model pose and print them autolly;
    /// </summary>
    class ReadInformation : public aris::core::CloneObject<ReadInformation,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ReadInformation();
        explicit ReadInformation(const std::string &name = "ReadInformation");

    private:
        double motorPos[4 * 3]{};
        double modelPose[4 * 7]{};
        std::vector<double> modelPoseVec;
        std::vector<double> modelPos;
        double modelPosArray[12]{};
    };

    /// <summary>
    /// drive motors to init pos so the model in init pose
    /// </summary>
    class SetMotorPosZero : public aris::core::CloneObject<SetMotorPosZero,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMotorPosZero();
        explicit SetMotorPosZero(const std::string &name = "SetMotorPosZero");

    private:
        int ret_all{};
        int ret[12]{};        
        double pos_[12]{};
        double pos_d_[12]{};
        double pos_dd_[12]{};
        double motorPos_[12]{};
    };

    /// <summary>
    /// drive 12 motors seperately
    /// </summary>
    class MotorTest : public aris::core::CloneObject<MotorTest,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MotorTest();
        explicit MotorTest(const std::string &name = "MotorTest");

    private:
        double motor0_{};
        double motor1_{};
        double motor2_{};
        double motor3_{};
        double motor4_{};
        double motor5_{};
        double motor6_{};
        double motor7_{};
        double motor8_{};
        double motor9_{};
        double motor10_{};
        double motor11_{};

        double pos_[12]{};
        double target_pos_[12]{};
        double pos_d_[12]{};
        double pos_dd_[12]{};
        double motorPos_[12]{};
        int ret_all{};
        int ret[12]{};
    };

    /// <summary>
    /// set all motor to init pos according to robot's model
    /// </summary>
    class ModelMotorInitialize : public aris::core::CloneObject<ModelMotorInitialize,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ModelMotorInitialize();
        explicit ModelMotorInitialize(const std::string &name = "ModelMotorInitialize");

    private:
        double motor0_{};
        double motor1_{};
        double motor2_{};
        double motor3_{};
        double motor4_{};
        double motor5_{};
        double motor6_{};
        double motor7_{};
        double motor8_{};
        double motor9_{};
        double motor10_{};
        double motor11_{};

        double pos_[12]{};
        double target_pos_[12]{};
        double pos_d_[12]{};
        double pos_dd_[12]{};
        double motorPos_[12]{};
        int ret_all{};
        int ret[12]{};
    };

    /// <summary>
    /// set motor drive max torque
    /// </summary>
    class SetMaxTorque : public aris::core::CloneObject<SetMaxTorque,aris::plan::Plan> 
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMaxTorque();
        explicit SetMaxTorque(const std::string &name = "set_max_torque");
    };  

    /// <summary>
    /// dog get plan
    /// </summary>
    class DogGet : public aris::core::CloneObject<DogGet, aris::plan::Plan>
    {
    public:
      auto virtual prepareNrt()->void;
      auto virtual collectNrt()->void;
      explicit DogGet(const std::string& name = "DogGet_plan");
      // ARIS_DEFINE_BIG_FOUR(DogGet);
      // 如果有参数，需要指定相应的成员变量用来获取和使用参数
    };

    /// <summary>
    /// define motor drive pdo and sequence of motor pool
    /// </summary>
    auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>;

    /// <summary>
    /// set motor defult setting of drive
    /// </summary>
    auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>;

    /// <summary>
    /// plan pool of command
    /// </summary>
    auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>;

    /// <summary>
    /// set all motors max torque
    /// </summary>
    auto setAllMotorMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value)->void;

    /// <summary>
    /// set single motor max torque
    /// </summary>
    auto setMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value, size_t index)->bool;
}
#endif