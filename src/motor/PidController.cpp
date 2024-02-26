#include "motor/PidController.h"

/*--------------Pid controller-------------*/
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
/*------------P controller----------------*/
auto robot::PController::getTargetOut(const double toq_des, const double target_q, const double actual_q, const double target_v, const double actual_v) -> double {
  maxToq_ = kRatedToq;
  dif_q_ = target_q - actual_q;
  dif_v_ = target_v - actual_v;
  double toq_ = toq_des + kp_ * dif_q_ + kd_ * dif_v_;
  
  if(toq_ > maxToq_){
    toq_ = maxToq_;
  }
  else if
  (toq_ < -maxToq_){
    toq_ = -maxToq_;
  }
  
  return toq_;
}
auto robot::PController::setPdParam(const double kp, const double kd) -> void
{
  kp_ = kp;
  kd_ = kd;
}

/*--------------Pid controller 2-------------*/
auto robot::PidController2::getTargetOut(const double target, const double actual) -> double
{
  maxToq_ = kRatedToq;
  dif_now_ = target - actual;
  div_ = dif_now_ - dif_lst_;
  itg_ += dif_now_;
  dif_lst_ = dif_now_;
  toq_ = kp_ * dif_now_ + kd_ * div_ + ki_ * itg_;

  if(toq_ > maxToq_){
    toq_ = maxToq_;
  }
  else if
  (toq_ < -maxToq_){
    toq_ = -maxToq_;
  }
  // toq_ = toq_ * ktoqFactor;
  return toq_;
}

auto robot::PidController2::setPidParam(const double kp, const double ki, const double kd) -> void
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

auto robot::PidController2::getDif() -> double{
  return dif_now_;
}

auto robot::PidController2::getDiv() -> double{
  return div_;
}

auto robot::PidController2::getItg() -> double{
  return itg_;
}