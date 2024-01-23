/*
*------------------------------------------------------------------------*
class  Name : public aris::core::CloneObject<Name, aris::plan::Plan> {
public:
  auto virtual prepareNrt()->void;
  auto virtual executeRT()->int;
  auto virtual collectNrt()->void;

  virtual ~Name();
  explicit Name(const std::string& name = "Name");

private:
};
*-------------------------------------------------------------------------*
auto Name::prepareNrt()->void
{
  for (auto& m : motorOptions()) m =
    aris::plan::Plan::CHECK_NONE;
}
auto Name::executeRT()->int
{
  return 0;
}
auto Name::collectNrt()->void {}
Name::Name(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"Cmd\">"
    "	<GroupParam>"
    "	<Param name=\"Param\" default=\"0\" abbreviation=\"x\"/>"
    "	</GroupParam>"
    "</Command>");
}
Name::~Name() = default;
*/