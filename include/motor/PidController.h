#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <aris.hpp>
#include <memory>
#include "server/Server.h"
#include "model/Model.h"
#include "tools/Operator.h"

namespace robot {
    class PidController {
        public:
            PidController() = default;
            explicit PidController(const double kpq, const double kiq, const double kdq, const double kpv, const double kiv, const double kdv)
                : kp_q_(kpq), ki_q_(kiq), kd_q_(kdq), kp_v_(kpv), ki_v_(kiv), kd_v_(kdv) {};
            virtual ~PidController() = default;

            auto getTargetOut(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double;
            //auto setPidParam(const double kp, const double ki, const double kd, const double max_integral = 1, const double max_output = 0) -> void;

        private:
            double kp_q_{};
            double ki_q_{};
            double kd_q_{};
            double itg_q_{};
            double div_q_{};
            double dif_now_q_{};
            double dif_lst_q_{};
            double toq_q_{}; //output torque of q;

            double kp_v_{};
            double ki_v_{};
            double kd_v_{};
            double itg_v_{};
            double div_v_{};
            double dif_now_v_{};
            double dif_lst_v_{};
            double toq_v_{}; //output torque of v;
    };

    class PController {
    public:
        PController() = default;
        explicit PController(const double kp, const double kd)
            : kp_(kp), kd_(kd){};
        virtual ~PController() = default;

        auto getTargetOut(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double;
        auto setPdParam(const double kp, const double kd) -> void;

    private:
        double kp_{};
        double kd_{};
        double dif_q_{};
        double dif_v_{};
        double maxToq_{};
    };

    class PidController2 {
    public:
      PidController2() = default;
      explicit PidController2(const double kp, const double ki, const double kd)
        : kp_(kp), ki_(ki), kd_(kd) {};
      virtual ~PidController2() = default;

      auto getTargetOut(const double target, const double actual) -> double;
      auto setPidParam(const double kp, const double ki, const double kd) -> void;
      auto getDif() -> double;
      auto getDiv() -> double;
      auto getItg() -> double;
      //auto setPidParam(const double kp, const double ki, const double kd, const double max_integral = 1, const double max_output = 0) -> void;

    private:
      double kp_{};
      double ki_{};
      double kd_{};
      double itg_{};
      double div_{};
      double dif_now_{};
      double dif_lst_{};
      double toq_{}; //output torque
      double maxToq_{};
    };
}
#endif // !PID_TEST_