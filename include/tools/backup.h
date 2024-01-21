/*

*------------------------------------------------------------------------*
auto Name::prepareNrt()->void {

  cef = doubleParam("coefficient");

  for (auto& m : motorOptions()) {
    m = aris::plan::Plan::CHECK_NONE;

  }
}
auto Name::executeRT()->int {

  return 0;

}
auto Name::collectNrt()->void {}
Name::Name(const std::string& name)
{
  aris::core::fromXmlString(command(),
    "<Command name=\"Cmd\">"
    "	<Param name=\"coefficient\" default=\"1.0\" abbreviation=\"k\"/>"
    "</Command>");
}
*-------------------------------------------------------------------------*
class Name : public aris::core::CloneObject<Name, aris::plan::Plan> {
public:
  auto virtual prepareNrt()->void;
  auto virtual executeRT()->int;
  auto virtual collectNrt()->void;

  virtual ~Name() = default;
  explicit Name(const std::string& name = "Name");

private:
  double param;
};
*-------------------------------------------------------------------------*





*/