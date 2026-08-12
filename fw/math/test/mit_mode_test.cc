// MIT impedance mode: torque law, limits, Kt guard, gain normalization.
#include "math/servo_mode/mit_mode.h"

#include <cmath>

#include "gtest/gtest.h"

namespace math { namespace servo_mode {
namespace {

MitMode::Options DefaultOptions()
{
  MitMode::Options options;
  options.max_torque_Nm = 1.0f;
  options.torque_constant_Nm_A = 0.1f;
  options.max_iq_A = 2.0f;
  return options;
}

TEST(MitModeTest, InactiveReturnsZero)
{
  MitMode mode(DefaultOptions());
  EXPECT_FLOAT_EQ(mode.Step(1e-4f, 1.0f, 2.0f), 0.0f);
  EXPECT_FALSE(mode.active());
}

TEST(MitModeTest, TorqueLawKpKdFeedforward)
{
  MitMode mode(DefaultOptions());
  MitMode::Command command;
  command.position_rad = 1.0f;
  command.velocity_rad_s = 2.0f;
  command.kp_nm_per_rad = 0.2f;
  command.kd_nm_s_per_rad = 0.1f;
  command.feedforward_Nm = 0.05f;
  mode.Start(command);

  // T = 0.2*(1-0.5) + 0.1*(2-1) + 0.05 = 0.25 Nm, iq = T/Kt = 2.5 → clamp 2.0.
  const float iq = mode.Step(1e-4f, 0.5f, 1.0f);
  EXPECT_FLOAT_EQ(mode.torque_Nm(), 0.25f);
  EXPECT_FLOAT_EQ(iq, 2.0f);
}

TEST(MitModeTest, TorqueClampedToCommandLimit)
{
  MitMode mode(DefaultOptions());
  MitMode::Command command;
  command.kp_nm_per_rad = 100.0f;
  command.position_rad = 10.0f;
  command.max_torque_Nm = 0.3f;
  mode.Start(command);

  mode.Step(1e-4f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(mode.torque_Nm(), 0.3f);
}

TEST(MitModeTest, NanCommandLimitFallsBackToOptions)
{
  MitMode mode(DefaultOptions());
  MitMode::Command command;
  command.kp_nm_per_rad = 100.0f;
  command.position_rad = 10.0f;
  // max_torque_Nm stays NaN → options_.max_torque_Nm (1.0) applies.
  mode.Start(command);

  mode.Step(1e-4f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(mode.torque_Nm(), 1.0f);
}

TEST(MitModeTest, TinyKtDoesNotDivideByZero)
{
  MitMode::Options options = DefaultOptions();
  options.torque_constant_Nm_A = 0.0f;  // degenerate config
  MitMode mode(options);
  MitMode::Command command;
  command.feedforward_Nm = 0.5f;
  mode.Start(command);

  const float iq = mode.Step(1e-4f, 0.0f, 0.0f);
  EXPECT_TRUE(std::isfinite(iq));
  EXPECT_FLOAT_EQ(iq, options.max_iq_A);  // clamped, not inf
}

TEST(MitModeTest, NegativeGainsNormalizedToZero)
{
  MitMode mode(DefaultOptions());
  MitMode::Command command;
  command.kp_nm_per_rad = -5.0f;
  command.kd_nm_s_per_rad = -1.0f;
  command.position_rad = 10.0f;
  mode.Start(command);

  // Negative gains would invert feedback into positive feedback; they must be
  // dropped to zero, leaving zero torque here.
  const float iq = mode.Step(1e-4f, 0.0f, 5.0f);
  EXPECT_FLOAT_EQ(iq, 0.0f);
  EXPECT_FLOAT_EQ(mode.torque_Nm(), 0.0f);
}

TEST(MitModeTest, StopZeroesOutputs)
{
  MitMode mode(DefaultOptions());
  MitMode::Command command;
  command.feedforward_Nm = 0.5f;
  mode.Start(command);
  mode.Step(1e-4f, 0.0f, 0.0f);
  ASSERT_NE(mode.iq_ref_A(), 0.0f);

  mode.Stop();
  EXPECT_FALSE(mode.active());
  EXPECT_FLOAT_EQ(mode.iq_ref_A(), 0.0f);
  EXPECT_FLOAT_EQ(mode.torque_Nm(), 0.0f);
}

}  // namespace
}  // namespace servo_mode
}  // namespace math
