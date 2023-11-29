#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include "aris.hpp"
#include "operator.h"

// #define ARIS_USE_ETHERCAT_SIMULATION

namespace robot
{   
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


    class SetZeroPose : public aris::core::CloneObject<SetZeroPose,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetZeroPose();
        explicit SetZeroPose(const std::string &name = "SetZeroPose");

    private:
        double pos_[12]{};
        double pos_d_[12]{};
        double pos_dd_[12]{};
        double motorPos_[12]{};
        int ret_all{};
        int ret[12]{};
    };

    class MotorTest12E3 : public aris::core::CloneObject<MotorTest12E3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MotorTest12E3();
        explicit MotorTest12E3(const std::string &name = "MotorTest12E3");

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

    class MotorTest12E2 : public aris::core::CloneObject<MotorTest12E2,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MotorTest12E2();
        explicit MotorTest12E2(const std::string &name = "MotorTest12E2");

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

    class SetMaxTorque : public aris::core::CloneObject<SetMaxTorque,aris::plan::Plan> 
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMaxTorque();
        explicit SetMaxTorque(const std::string &name = "set_max_torque");
    };  

    auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>;
    auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>;
    auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>;
    auto setAllMotorMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value)->void;
    auto setMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value, size_t index)->bool;
}
#endif