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

        std::vector<double> PE{28, 0.0};

        
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

    class Ellipse4LegDrive2 : public aris::core::CloneObject<Ellipse4LegDrive2,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~Ellipse4LegDrive2();
        explicit Ellipse4LegDrive2(const std::string &name = "Ellipse4LegDrive2");

    private:
        double moveX_{};
        double moveY_{};
        double moveZ_{};
        double Height_{};

        double theta_{};
        double theta_d_{};
        double theta_dd_{};

        Matrix<double> startMotorPos = Matrix<double>(4 ,3);
        Matrix<double> finalMotorPos = Matrix<double>(4 ,3);
        Matrix<double> moveMotorPos = Matrix<double>(4 ,3);

        Matrix<double> finalModelPE = Matrix<double>(4, 7);
        Matrix<double> moveModelPE = Matrix<double>(4, 7);
        Matrix<double> startModelPE = Matrix<double>(4, 7);

        double transfer_1[12];
        double transfer_2[28];
        double transfer_3[28];
    };

    class Ellipse4LegDrive : public aris::core::CloneObject<Ellipse4LegDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~Ellipse4LegDrive();
        explicit Ellipse4LegDrive(const std::string &name = "Ellipse4LegDrive");

    private:
        double moveX_{};
        double moveY_{};
        double moveZ_{};
        double Height_{};

        double startMotorPos[12]{};
        double finalPoint_[12]{};

        double eePoint_[12]{};

        double startPE_[28]{};
        double finalPE_[28]{};
        double movePE_[28]{};

        double* majorAxisUnitVector_1;
        double* minorAxisUnitVector_1;    
        double* centerPoint_1;
        double Width_1{};

        double* majorAxisUnitVector_2;
        double* minorAxisUnitVector_2;    
        double* centerPoint_2;
        double Width_2{};


        double* majorAxisUnitVector_3;
        double* minorAxisUnitVector_3;    
        double* centerPoint_3;
        double Width_3{};

        
        double* majorAxisUnitVector_4;
        double* minorAxisUnitVector_4;    
        double* centerPoint_4;
        double Width_4{}; 

        double startPoint1[3]{};
        double startPoint2[3]{};
        double startPoint3[3]{};
        double startPoint4[3]{};    

        double theta_{};
        double theta_d_{};
        double theta_dd_{};
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