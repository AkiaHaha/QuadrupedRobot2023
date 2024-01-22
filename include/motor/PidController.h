#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <aris.hpp>
#include <memory>
#include "server/server.h"
#include "model/model.h"

namespace robot {
    class PidController {
        public:
            PidController() = default;
            explicit PidController(const double kpq, const double kiq, const double kdq, const double kpv, const double kiv, const double kdv)
                : kp_q_(kpq), ki_q_(kiq), kd_q_(kdq), kp_v_(kpv), ki_v_(kiv), kd_v_(kdv) {};
            virtual ~PidController() = default;

            auto getTargetToq(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double;
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

        auto getTargetToq(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double;

    private:
        double kp_{};
        double kd_{};
        double dif_q_{};
        double dif_v_{};
    };
}
#endif // !PID_TEST_