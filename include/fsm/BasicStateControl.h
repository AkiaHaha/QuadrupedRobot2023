#ifndef BASIC_STATE_CONTROL_H
#define BASIC_STATE_CONTROL_H
#include <aris.hpp>
#include <motor/PidController.h>

class  StatePassive2Stand : public aris::core::CloneObject<StatePassive2Stand, aris::plan::Plan> {
    public:
      auto virtual prepareNrt()->void;
      auto virtual executeRT()->int;
      auto virtual collectNrt()->void;

      virtual ~StatePassive2Stand();
      explicit StatePassive2Stand(const std::string& name = "StatePassive2Stand");

    private:
      robot::PidController2 legPidPosController[3] = {
        robot::PidController2(20, 0.02, 0.1),
        robot::PidController2(30, 0.03, 0.15),
        robot::PidController2(40, 0.04, 0.2)
      };

};


#endif // !BASIC_STATE_CONTROL_H
