#include "fsm/BasicStateControl.h"
using namespace aris::dynamic;
using namespace aris::plan;

auto StatePassive2Stand::prepareNrt()->void {
  for (auto& m : motorOptions()) m =
    aris::plan::Plan::CHECK_NONE;
}
auto StatePassive2Stand::executeRT()->int {
  return 0;
}
auto StatePassive2Stand::collectNrt()->void {}
StatePassive2Stand::StatePassive2Stand(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"Cmd\">"
    "	<GroupParam>"
    "	<Param name=\"Param\" default=\"0\" abbreviation=\"x\"/>"
    "	</GroupParam>"
    "</Command>");
}
StatePassive2Stand::~StatePassive2Stand() = default;

