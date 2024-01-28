#ifndef MOTOR_TEST_H_
#define MOTOR_TEST_H_
#include <memory>
#include "aris.hpp"
#include "tools/Operator.h"
#include "control/Plan.h"
#include "tools/json.hpp"

namespace robot {
    /// <summary>
    /// read motor pos and model pose and print them autolly;
    /// </summary>
    class ReadInformation : public aris::core::CloneObject<ReadInformation, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ReadInformation();
        explicit ReadInformation(const std::string& name = "ReadInformation");

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
    class SetMotorPosZero : public aris::core::CloneObject<SetMotorPosZero, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMotorPosZero();
        explicit SetMotorPosZero(const std::string& name = "SetMotorPosZero");

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
    class MotorTest : public aris::core::CloneObject<MotorTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MotorTest();
        explicit MotorTest(const std::string& name = "MotorTest");

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
    class ModelMotorInitialize : public aris::core::CloneObject<ModelMotorInitialize, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ModelMotorInitialize();
        explicit ModelMotorInitialize(const std::string& name = "ModelMotorInitialize");

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
    class SetMaxTorque : public aris::core::CloneObject<SetMaxTorque, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMaxTorque();
        explicit SetMaxTorque(const std::string& name = "set_max_torque");
    };

}
#endif