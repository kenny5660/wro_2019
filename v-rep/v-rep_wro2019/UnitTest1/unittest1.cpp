#include "CppUnitTest.h"
#include "Robot.h"
#include "b0RemoteApi.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest1 {
TEST_CLASS(UnitTest1) {
 public:
  std::shared_ptr<RobotVrep> robot;
  TEST_METHOD_INITIALIZE(initTests) {

    robot = std::make_shared<RobotVrep>(
        std::make_shared<b0RemoteApi>("b0RemoteApi_c++Client", "b0RemoteApiAddOn"));
    robot->Init();
  }
  TEST_METHOD(TestMethod1) {
    // TODO: Разместите здесь код своего теста
  }
};
}  // namespace UnitTest1