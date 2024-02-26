#ifndef MOTOR_TEST_H_
#define MOTOR_TEST_H_
#include <memory>
#include "aris.hpp"
#include "tools/Operator.h"
#include "control/Plan.h"
#include "tools/json.hpp"
#include "motor/PidController.h"

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



    class SaveHome : public aris::core::CloneObject<SaveHome, aris::plan::Plan>
    {

    public:

        auto virtual prepareNrt()->void override;
        explicit SaveHome(const std::string& name = "SaveHome_plan");
    };


    class DogHome :public aris::core::CloneObject<DogHome, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogHome();
        explicit DogHome(const std::string& name = "dog_home");
        ARIS_DECLARE_BIG_FOUR(DogHome)
    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

    class LegTest : public aris::core::CloneObject<LegTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~LegTest();
        explicit LegTest(const std::string& name = "LegTest");

    private:
        int leg_id_{};
        double move_x_{};
        double move_h_{};
        double move_z_{};

        double pos_init_[12]{};
        double vel_init_[12]{};
        double toq_init_[12]{};
        double BppFee_init_[28]{};

        double pos_run_[12]{};
        double BppFee_run_[28]{};
        double Bpp_run_[16]{};
        double Fee_run_[12]{};

        double foot_ee_[3]{};
        double testLeg_motor_pos_[3]{};

        double target_pos_[12]{};

        int test_leg_me_id_[3]{};
        int kStop_{};
        int show_period_{};

        // robot::PController PdPosController{5, 1};
        robot::PidController2 VelLoopController{1, 0.1, 0.01};
        robot::PidController2 PosLoopController{15, 0.1, 0.01}; 

        int flag_mode_set_{};
    };

    class LegStepTest : public aris::core::CloneObject<LegStepTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~LegStepTest();
        explicit LegStepTest(const std::string& name = "LegStepTest");

    private:
        int leg_id_{};
        double move_x_{};
        double move_h_{};
        double move_z_{};

        double pos_init_[12]{};
        double vel_init_[12]{};
        double toq_init_[12]{};
        double BppFee_init_[28]{};

        double pos_run_[12]{};
        double BppFee_run_[28]{};
        double Bpp_run_[16]{};
        double Fee_run_[12]{};

        double foot_ee_[3]{};
        double testLeg_motor_pos_[3]{};

        double target_pos_[12]{};

        int test_leg_me_id_[3]{};
        int kStop_{};
        int show_period_{};

        // robot::PController PdPosController{5, 1};
        robot::PidController2 VelLoopController{1, 0.1, 0.01};
        robot::PidController2 PosLoopController{15, 0.1, 0.01}; 

        int flag_mode_set_{};
        int step_count_{};
        int flag_step_count_{};
        int pn_sym_{};
        int zero_tf_{};
        double theta_;
        double tcurve_a_{};
        double tcurve_v_{};

    };

    class AllToqTest : public aris::core::CloneObject<AllToqTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~AllToqTest();
        explicit AllToqTest(const std::string& name = "AllToqTest");

    private:

        double pos[12]{};
        double pos_init[12]{};
        double vel[12]{};
        double vel_init[12]{};
        double toq[12]{};
        double toq_cmd[12]{};
        double flag_start[12]{};

        // robot::PidController2 VelLoopController{1, 0.1, 0.01};
        // robot::PidController2 PosLoopController{15, 0.1, 0.01}; 
        robot::PController PdPosController{25, 0.15};
        double kp_{};
        double kd_{};
        int run_time_{};
        int test_motor_id_{};
    };

}
#endif

