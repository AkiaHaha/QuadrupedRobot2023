#include "motor/PidController.h"

auto robot::PidController::getTargetOut(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double {
  dif_now_q_ = target_q - actual_q;
  div_q_ = dif_now_q_ - dif_lst_q_;
  itg_q_ += div_q_;
  dif_lst_q_ = dif_now_q_;
  toq_q_ = kp_q_ * dif_now_q_ + kd_q_ * div_q_ + ki_q_ + ki_q_ * itg_q_;

  dif_now_v_ = target_v - actual_v;
  div_v_ = dif_now_v_ - dif_lst_v_;
  itg_v_ += div_v_;
  dif_lst_v_ = dif_now_v_;
  toq_v_ = kp_v_ * dif_now_v_ + kd_v_ * div_v_ + ki_v_ + ki_v_ * itg_v_;

  double TorqueCmd = toq_des + toq_q_ + toq_v_;

  return TorqueCmd;
}

auto robot::PController::getTargetOut(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double {
  dif_q_ = target_q - actual_q;
  dif_v_ = target_v - actual_v;
  double TorqueCmd = toq_des + kp_ * dif_q_ + kd_ * dif_v_;
  return TorqueCmd;
}

auto robot::PidController2::getTargetOut(const double target, const double actual) -> double
{
  dif_now_ = target - actual;
  div_ = dif_now_ - dif_lst_;
  itg_ += div_;
  dif_lst_ = dif_now_;
  toq_ = kp_ * dif_now_ + kd_ * div_ + ki_ + ki_ * itg_;
  return toq_;
}

auto robot::PidController2::setPidParam(const double kp, const double ki, const double kd) -> void
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
