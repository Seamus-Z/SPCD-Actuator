// ServoMode trajectory branches: velocity/position modes, accel/vel limits,
// stop-position edge cases, torque/iq clamping. These are the edges that are
// hard to reproduce on real hardware.
#include "math/servo_mode/servo_mode.h"

#include <cmath>

#include "gtest/gtest.h"
#include "math/constants.h"

namespace math { namespace servo_mode {
namespace {

constexpr float kDt = 1.0f / 15000.0f;

// Mirrors board_config.h ServoModeOptions (sign=-1 servo convention).
// NOTE: max_velocity_cmd_rad_s must be > 0 like the real board config.
// With the default 0 ("unlimited"), NormalizeCommandLimits copies it into
// velocity_limit when only an accel limit is given, clamping the velocity
// command to 0 — a latent quirk that never fires with board values.
ServoMode::Options DefaultOptions()
{
  ServoMode::Options options;
  options.pid.kp = 4.0f;
  options.pid.ki = 0.0f;
  options.pid.kd = 0.05f;
  options.pid.ilimit = 0.0f;
  options.pid.sign = -1;
  options.max_torque_Nm = 0.3f;
  options.torque_constant_Nm_A = 0.1f;
  options.max_iq_A = 3.0f;
  options.max_velocity_cmd_rad_s = 200.0f;
  return options;
}

TEST(ServoModeTest, InactiveReturnsZero)
{
  ServoMode servo(DefaultOptions());
  EXPECT_FLOAT_EQ(servo.Step(kDt, 1.0f, 2.0f), 0.0f);
}

TEST(ServoModeTest, PureVelocityCommandDrivesPositiveIq)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.velocity_rad_s = 10.0f;  // no position, no limits
  servo.Start(command);

  // First step only captures the cold-start control state (measured velocity
  // overrides the command); the command takes effect from the second step.
  servo.Step(kDt, 0.0f, 0.0f);
  const float iq = servo.Step(kDt, 0.0f, 0.0f);
  // Rotor at standstill, commanded forward: torque and iq must be positive.
  EXPECT_GT(iq, 0.0f);
  EXPECT_TRUE(servo.trajectory_done());
  EXPECT_NEAR(servo.control_velocity(), 10.0f, 1e-4f);
}

TEST(ServoModeTest, PositionStepWithoutLimitsJumpsSetpoint)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.position_rad = 2.0f;
  command.velocity_rad_s = 0.0f;
  servo.Start(command);

  servo.Step(kDt, 0.0f, 0.0f);
  // Without velocity/accel limits the setpoint applies immediately.
  EXPECT_NEAR(servo.control_position_rad(), 2.0f, 1e-4f);
  EXPECT_TRUE(servo.trajectory_done());
}

TEST(ServoModeTest, AccelLimitedVelocityRampReachesTarget)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.velocity_rad_s = 10.0f;
  command.accel_limit_rad_s2 = 100.0f;  // 0→10 rad/s needs 0.1 s
  servo.Start(command);

  float prev_v = 0.0f;
  bool monotonic = true;
  for (int i = 0; i < 3000; ++i)  // 0.2 s
  {
    servo.Step(kDt, 0.0f, 0.0f);
    const float v = servo.control_velocity();
    if (v < prev_v - 1e-4f)
    {
      monotonic = false;
    }
    prev_v = v;
  }
  EXPECT_TRUE(monotonic);
  EXPECT_TRUE(servo.trajectory_done());
  EXPECT_NEAR(servo.control_velocity(), 10.0f, 1e-3f);
}

TEST(ServoModeTest, PositionMoveRespectsVelocityLimit)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.position_rad = 20.0f;
  command.velocity_limit_rad_s = 5.0f;
  command.accel_limit_rad_s2 = 200.0f;
  servo.Start(command);

  float position = 0.0f;
  for (int i = 0; i < 15000 * 6; ++i)  // 6 s, enough for 20 rad at 5 rad/s
  {
    servo.Step(kDt, position, servo.control_velocity());
    // Follow the trajectory perfectly (isolates the planner from the PID).
    position = servo.control_position_rad();
    EXPECT_LE(std::fabs(servo.control_velocity()), 5.0f + 1e-2f);
    if (servo.trajectory_done())
    {
      break;
    }
  }
  EXPECT_TRUE(servo.trajectory_done());
  EXPECT_NEAR(servo.control_position_rad(), 20.0f, 0.05f);
}

TEST(ServoModeTest, TargetEqualsCurrentPositionSettlesQuickly)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.position_rad = 1.0f;  // already there
  command.velocity_limit_rad_s = 5.0f;
  command.accel_limit_rad_s2 = 50.0f;
  servo.Start(command);

  for (int i = 0; i < 100; ++i)
  {
    servo.Step(kDt, 1.0f, 0.0f);
    if (servo.trajectory_done())
    {
      break;
    }
  }
  EXPECT_TRUE(servo.trajectory_done());
  EXPECT_NEAR(servo.control_position_rad(), 1.0f, 1e-3f);
  EXPECT_NEAR(servo.control_velocity(), 0.0f, 1e-3f);
}

TEST(ServoModeTest, StopPositionAheadArrestsMotion)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.velocity_rad_s = 10.0f;
  command.stop_position_rad = 1.0f;
  servo.Start(command);

  float position = 0.0f;
  bool arrested = false;
  for (int i = 0; i < 15000; ++i)  // 1 s; 10 rad/s hits 1 rad in 0.1 s
  {
    servo.Step(kDt, position, servo.control_velocity());
    position = servo.control_position_rad();  // follow the planner exactly
    if (servo.control_velocity() == 0.0f && i > 2)
    {
      arrested = true;
      break;
    }
  }
  EXPECT_TRUE(arrested);
  EXPECT_NEAR(servo.control_position_rad(), 1.0f, 1e-3f);
  EXPECT_FLOAT_EQ(servo.control_velocity(), 0.0f);
  EXPECT_FLOAT_EQ(servo.velocity_cmd(), 0.0f);
}

TEST(ServoModeTest, StopPositionBehindTravelDirectionStopsImmediately)
{
  // Stop point in the *opposite* direction of travel: the guard must arrest
  // motion instead of driving away from the stop forever.
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.velocity_rad_s = 10.0f;      // moving forward
  command.stop_position_rad = -1.0f;   // stop point behind
  servo.Start(command);

  // Step 1 captures cold-start state; step 2 starts moving forward and the
  // stop guard must arrest immediately (position already beyond the stop).
  servo.Step(kDt, 0.0f, 0.0f);
  servo.Step(kDt, 0.0f, 0.0f);
  EXPECT_TRUE(servo.trajectory_done());
  EXPECT_FLOAT_EQ(servo.control_velocity(), 0.0f);
  EXPECT_NEAR(servo.control_position_rad(), -1.0f, 1e-4f);
  EXPECT_FLOAT_EQ(servo.velocity_cmd(), 0.0f);
}

TEST(ServoModeTest, ZeroVelocityLimitFreezesWithoutNan)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.position_rad = 5.0f;
  command.velocity_limit_rad_s = 0.0f;  // degenerate: no motion allowed
  servo.Start(command);

  for (int i = 0; i < 100; ++i)
  {
    const float iq = servo.Step(kDt, 0.0f, 0.0f);
    EXPECT_TRUE(std::isfinite(iq));
  }
  EXPECT_TRUE(std::isfinite(servo.control_position_rad()));
  EXPECT_FALSE(servo.faulted());
}

TEST(ServoModeTest, NanAccelWithVelocityLimitStillConverges)
{
  ServoMode servo(DefaultOptions());
  ServoMode::Command command;
  command.position_rad = 1.0f;
  command.velocity_limit_rad_s = 10.0f;
  // accel_limit stays NaN → constant-velocity approach branch.
  servo.Start(command);

  float position = 0.0f;
  for (int i = 0; i < 15000; ++i)
  {
    servo.Step(kDt, position, servo.control_velocity());
    position = servo.control_position_rad();
    if (servo.trajectory_done())
    {
      break;
    }
  }
  EXPECT_TRUE(servo.trajectory_done());
  EXPECT_NEAR(servo.control_position_rad(), 1.0f, 1e-2f);
}

TEST(ServoModeTest, TorqueAndIqClampsApply)
{
  ServoMode::Options options = DefaultOptions();
  options.pid.kp = 1000.0f;  // force saturation
  ServoMode servo(options);
  ServoMode::Command command;
  command.position_rad = 100.0f;
  servo.Start(command);

  const float iq = servo.Step(kDt, 0.0f, 0.0f);
  EXPECT_LE(std::fabs(servo.torque_Nm()), options.max_torque_Nm + 1e-5f);
  EXPECT_LE(std::fabs(iq), options.max_iq_A + 1e-5f);
}

TEST(ServoModeTest, CommandMaxTorqueOverridesOptions)
{
  ServoMode::Options options = DefaultOptions();
  options.pid.kp = 1000.0f;
  ServoMode servo(options);
  ServoMode::Command command;
  command.position_rad = 100.0f;
  command.max_torque_Nm = 0.1f;  // tighter than options (0.3)
  servo.Start(command);

  servo.Step(kDt, 0.0f, 0.0f);
  EXPECT_LE(std::fabs(servo.torque_Nm()), 0.1f + 1e-5f);
}

TEST(ServoModeTest, StopClearsState)
{
  ServoMode servo(DefaultOptions());
  servo.Start(1.0f, 2.0f);
  servo.Step(kDt, 0.0f, 0.0f);
  servo.Stop();
  EXPECT_FALSE(servo.active());
  EXPECT_FLOAT_EQ(servo.iq_ref_A(), 0.0f);
  EXPECT_FLOAT_EQ(servo.Step(kDt, 0.0f, 0.0f), 0.0f);
}

}  // namespace
}  // namespace servo_mode
}  // namespace math
