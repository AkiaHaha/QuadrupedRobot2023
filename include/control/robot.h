#ifndef ROBOT_H_
#define ROBOT_H_
#include <memory>
#include "aris.hpp"
#include "tools/Operator.h"
#include "control/Plan.h"
#include "tools/json.hpp"
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
}
#endif