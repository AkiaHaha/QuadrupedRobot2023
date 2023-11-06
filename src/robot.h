#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

// #define ARIS_USE_ETHERCAT_SIMULATION

namespace robot
{   
    class MotorTest12 : public aris::core::CloneObject<MotorTest12,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~MotorTest12();
        explicit MotorTest12(const std::string &name = "MotorTest12");

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

        int iNumberMax{};
        double angle[12]{};    
        int dir[12]{};
    };

    class ellipticalTrajectoryDrive3 : public aris::core::CloneObject<ellipticalTrajectoryDrive3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ellipticalTrajectoryDrive3();
        explicit ellipticalTrajectoryDrive3(const std::string &name = "ellipticalTrajectoryDrive3");

    private:
        double targetX{};
        double targetY{};
        double targetZ{};
        double Height{};
        double Width{};
        double Length{};
        double eePoint[3]{};
        double endPoint[3]{};
        double startPoint[3]{};
        double* majorAxisUnitVector;
        double* minorAxisUnitVector;    
        double* centerPoint;
        double theta{};
        double theta_d{};
        double theta_dd{};
    };

    class moveBeeLineE2 :public aris::core::CloneObject<moveBeeLineE2,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~moveBeeLineE2();
        explicit moveBeeLineE2(const std::string &name = "single_leg_move_bee_line_e2");

    private: 

        double posOfMoveStatus[3]{0};
        double velOfMoveStatus[3]{0};
        double accOfMoveStatus[3]{0};
        double posOfStatus1[3]{0};
        double moveTargetX;
        double moveTargetY;
        double moveTargetZ;
    };    
     
    class legInitialization :public aris::core::CloneObject<legInitialization,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~legInitialization();
        explicit legInitialization(const std::string &name = "leg_initialization");

    private: 
        double initAngle0;
        double initAngle1;
        double initAngle2;   
    };
    
    class forwardKinLegTest :public aris::core::CloneObject<forwardKinLegTest,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~forwardKinLegTest();
        explicit forwardKinLegTest(const std::string &name = "fwd_kin_single_leg");

    private:
        double fwdAngle0;
        double fwdAngle1;
        double fwdAngle2;
        double intervals;      
    };
    
    class inverseKinLegTest :public aris::core::CloneObject<inverseKinLegTest,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~inverseKinLegTest();
        explicit inverseKinLegTest(const std::string &name = "int_kin_single_leg");

    private:
        double invPos_x;
        double invPos_y;
        double invPos_z;
        double intervals;      
    };
    
    class cosCurveDriveTogetherM3 :public aris::core::CloneObject<cosCurveDriveTogetherM3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~cosCurveDriveTogetherM3();
        explicit cosCurveDriveTogetherM3(const std::string &name = "ccurve_drive_tgt_m3");

    private:
        double cef_1;
        double cef_2;
        double cef_3;
        double totalTime;
        double a_;
        double w_;
        double p_;
    };

    class VelDrive : public aris::core::CloneObject<VelDrive,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~VelDrive();
        explicit VelDrive(const std::string &name = "vel_drive");

     private:
        double cef_;
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