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
        std::make_shared<b0RemoteApi>("b0RemoteApi_c++Client", "b0RemoteApi"));
    robot->Init();
  }
  TEST_METHOD(InitTest) {
    // TODO: Разместите здесь код своего теста
  }
  TEST_METHOD(OmniWheelTest) {
    robot->GetOmni()->MoveWithSpeed(std::make_pair(130, 0), 0);
    robot->GetOmni()->Stop();
    robot->GetOmni()->MoveWithSpeed(std::make_pair(0, 130), 0);
    robot->GetOmni()->Stop();
    robot->GetOmni()->MoveWithSpeed(std::make_pair(-130, -130), 0);
    robot->GetOmni()->Stop();
  }
  TEST_METHOD(MotorTest) {
    robot->GetOmni()->GetMotor(OmniWheels::MotorDir::RIGHT)->MoveContinue(-100);
  }
};
}  // namespace UnitTest1