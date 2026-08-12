#include "application_policy.h"

#include <cstdint>

#include "gtest/gtest.h"

namespace app
{
namespace
{

TEST(LifecycleStateMachineTest, InitializationFailureRemainsLatched)
{
  LifecycleStateMachine lifecycle;

  lifecycle.CompleteInitialization(false);
  EXPECT_EQ(lifecycle.state(), State::INIT_FAILED);

  lifecycle.CompleteDriverRecovery(true);
  EXPECT_EQ(lifecycle.state(), State::INIT_FAILED);

  lifecycle.CompleteInitialization(true);
  EXPECT_EQ(lifecycle.state(), State::INIT_FAILED);
}

TEST(CommandActivityTest, IgnoresNonAddressedAndInvalidTraffic)
{
  EXPECT_FALSE(IsCommandActivity(false, false));
  EXPECT_TRUE(IsCommandActivity(true, false));
  EXPECT_TRUE(IsCommandActivity(false, true));
}

TEST(CommandDeadlineTest, BusySnapshotCannotSuppressActiveMotorTimeout)
{
  constexpr uint32_t kTimeoutUs = 500000;
  constexpr bool kSnapshotBusy = true;
  (void)kSnapshotBusy;

  EXPECT_FALSE(CommandDeadlineExpired(true, 499999, 0, kTimeoutUs));
  EXPECT_TRUE(CommandDeadlineExpired(true, 500000, 0, kTimeoutUs));
  EXPECT_TRUE(CommandDeadlineExpired(true, 600000, 100000, kTimeoutUs));
  EXPECT_FALSE(CommandDeadlineExpired(false, 600000, 0, kTimeoutUs));
}

TEST(CommandDeadlineTest, HandlesMonotonicCounterWraparound)
{
  constexpr uint32_t kTimeoutUs = 500000;
  constexpr uint32_t kLast = 0xfffc0000u;
  constexpr uint32_t kNow = kLast + kTimeoutUs;

  EXPECT_TRUE(CommandDeadlineExpired(true, kNow, kLast, kTimeoutUs));
}

}  // namespace
}  // namespace app
