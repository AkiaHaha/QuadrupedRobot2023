#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

// #define ARIS_USE_ETHERCAT_SIMULATION

namespace robot
{   
    class SetMaxTorque : public aris::core::CloneObject<SetMaxTorque,aris::plan::Plan> 
    {
        public:
            auto virtual prepareNrt()->void;
            auto virtual executeRT()->int;
            auto virtual collectNrt()->void;

            virtual ~SetMaxTorque();
            explicit SetMaxTorque(const std::string &name = "set_max_torque");
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

    class SingleMotorTest :public aris::core::CloneObject<SingleMotorTest,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SingleMotorTest();
        explicit SingleMotorTest(const std::string &name = "SingleMotorTest");

    private:
        double angle_;
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