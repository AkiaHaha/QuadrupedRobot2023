#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

// #define ARIS_USE_ETHERCAT_SIMULATION

namespace robot
{   
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

    class ellipticalTrajectoryDrive2 : public aris::core::CloneObject<ellipticalTrajectoryDrive2,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ellipticalTrajectoryDrive2();
        explicit ellipticalTrajectoryDrive2(const std::string &name = "ellipticalTrajectoryDrive2");

    private:
        double Height{};
        double Width{};
        double Length{};
        double Direction{};
        double eePoint[3]{};
        double endPoint[3]{};
        double startPoint[3]{};
        double posOfInitialization[3]{};
        double* unitVector;
        double* centerPoint;
        double theta{};
        double theta_d{};
        double theta_dd{};
    };



    class ellipticalTrajectoryDrive : public aris::core::CloneObject<ellipticalTrajectoryDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~ellipticalTrajectoryDrive();
        explicit ellipticalTrajectoryDrive(const std::string &name = "ellipticalTrajectoryDrive");

    private:
        double phi{};
        double Height{};
        double Width{};
        double moveTargetX{};
        double moveTargetY{};
        double moveTargetZ{};
        double eePoint[3]{};
        double endPoint[3]{};
        double startPoint[3]{};
        double posOfInitialization[3]{};
        double* unitVector;
        double* centerPoint;
        bool initializationStatus;
        int initializationCount{};
        double theta{};
        double theta_d{};
        double theta_dd{};
        double theta_target{};
    };

    class tCurveDrive :public aris::core::CloneObject<tCurveDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~tCurveDrive();
        explicit tCurveDrive(const std::string &name = "tcurve_drive");

    private:
        double cef_;
        double acc;
        double vel;
        double totalTime;
        int motorNumber;
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
     
    class moveBeeLine :public aris::core::CloneObject<moveBeeLine,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~moveBeeLine();
        explicit moveBeeLine(const std::string &name = "single_leg_move_bee_line");

    private: 
        double moveDir;
        double moveTime;
        double posOfMoveStatus[3]{0};
        double speedControlVariable;

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
    
    class anyExecuteTest :public aris::core::CloneObject<anyExecuteTest,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~anyExecuteTest();
        explicit anyExecuteTest(const std::string &name = "any_execute");

    private:
        double cef_1;
        double cef_2;
        double cef_3;
        double vel;
        double acc;
        int intervals;      
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
    
    class cosCurveDriveMx :public aris::core::CloneObject<cosCurveDriveMx,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~cosCurveDriveMx();
        explicit cosCurveDriveMx(const std::string &name = "ccurve_drive_mx");

    private:
        double cef_1;
        double cef_2;
        double cef_3;
        double totalTime;
        double a_;
        double w_;
        double p_;
        int motorNumber;
        double velocityVariable[3];
        std::vector<double> begin_angle; 
        std::vector<double> angle;   
    };

    class cosCurveDriveTogetherM2 :public aris::core::CloneObject<cosCurveDriveTogetherM2,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~cosCurveDriveTogetherM2();
        explicit cosCurveDriveTogetherM2(const std::string &name = "ccurve_drive_tgt_m2");

    private:
        double cef_1;
        double cef_2;
        double totalTime;
        double a_;
        double w_;
        double p_;
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

    class cosCurveDriveIntervalM3 :public aris::core::CloneObject<cosCurveDriveIntervalM3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~cosCurveDriveIntervalM3();
        explicit cosCurveDriveIntervalM3(const std::string &name = "ccurve_drive_itv_m3");

    private:
        double cef_1;
        double cef_2;
        double cef_3;
        double intervals;
        double a_;
        double w_;
        double p_;
    };

    class tCurveDriveIntervalM3 :public aris::core::CloneObject<tCurveDriveIntervalM3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~tCurveDriveIntervalM3();
        explicit tCurveDriveIntervalM3(const std::string &name = "tcurve_drive_itv_m3");

    private:
        double cef_1;
        double cef_2;
        double cef_3;
        double vel;
        double acc;
        double intervals;
    };

    class tCurveDriveTogetherM3 :public aris::core::CloneObject<tCurveDriveTogetherM3,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~tCurveDriveTogetherM3();
        explicit tCurveDriveTogetherM3(const std::string &name = "tcurve_drive_tgt_m3");

    private:
        double cef_1;
        double cef_2;
        double cef_3;
        double vel;
        double acc;
        double intervals;
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

    class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
      {
      public:
          auto virtual prepareNrt()->void;
          auto virtual executeRT()->int;
          auto virtual collectNrt()->void;

          explicit MoveJS(const std::string &name = "MoveJS_plan");

      };

    auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>;
    auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>;
    auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>;
    auto setAllMotorMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value)->void;
    auto setMaxTorque(aris::control::EthercatMaster* ecMaster, std::uint16_t value, size_t index)->bool;
}
#endif