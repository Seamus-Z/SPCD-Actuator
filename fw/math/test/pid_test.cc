// PID boundary behavior: integral windup, ilimit=0, sign, desired-rate slew.
#include "math/servo_mode/pid.h"

#include <cmath>

#include "gtest/gtest.h"

namespace math { namespace servo_mode {
namespace {

TEST(PidTest, IntegralSaturatesAtIlimit)
{
  PID::Config config;
  config.ki = 1.0f;
  config.ilimit = 0.5f;
  PID::State state;
  PID pid(&config, &state);

  // Constant error of +1 for 100 steps; unclamped integral would reach 10.
  for (int i = 0; i < 100; ++i)
  {
    pid.Apply(1.0f, 0.0f, 0.0f, 0.0f, 0.1f);
  }
  EXPECT_FLOAT_EQ(state.integral, 0.5f);
  EXPECT_FLOAT_EQ(state.command, 0.5f);
}

TEST(PidTest, ZeroIlimitDisablesIntegral)
{
  PID::Config config;
  config.kp = 2.0f;
  config.ki = 10.0f;
  config.ilimit = 0.0f;
  PID::State state;
  PID pid(&config, &state);

  for (int i = 0; i < 50; ++i)
  {
    pid.Apply(1.0f, 0.0f, 0.0f, 0.0f, 0.1f);
  }
  // Integral clamped to [−0, 0]: pure P remains.
  EXPECT_FLOAT_EQ(state.integral, 0.0f);
  EXPECT_FLOAT_EQ(state.command, 2.0f);
}

TEST(PidTest, SignFlipsCommand)
{
  PID::Config config;
  config.kp = 1.0f;
  config.sign = -1;
  PID::State state;
  PID pid(&config, &state);

  // measured > desired with sign=-1 (servo convention) drives negative error
  // into a positive corrective command.
  const float out = pid.Apply(0.0f, 1.0f, 0.0f, 0.0f, 0.001f);
  EXPECT_FLOAT_EQ(state.error, -1.0f);
  EXPECT_FLOAT_EQ(out, 1.0f);
}

TEST(PidTest, IntegralRateLimit)
{
  PID::Config config;
  config.ki = 100.0f;
  config.ilimit = 10.0f;
  config.iratelimit = 1.0f;  // max integral growth: 1.0/s
  PID::State state;
  PID pid(&config, &state);

  // Error large enough that unclamped update (error*ki*dt = 10) far exceeds
  // the per-step rate cap (iratelimit*dt = 0.1).
  pid.Apply(1.0f, 0.0f, 0.0f, 0.0f, 0.1f);
  EXPECT_FLOAT_EQ(state.integral, 0.1f);
}

TEST(PidTest, DesiredRateSlewsSetpoint)
{
  PID::Config config;
  config.kp = 1.0f;
  config.max_desired_rate = 1.0f;  // units/s
  PID::State state;
  PID pid(&config, &state);

  // First call: state.desired is NaN, so the setpoint applies unslewed.
  pid.Apply(0.0f, 0.0f, 0.0f, 0.0f, 0.1f);
  EXPECT_FLOAT_EQ(state.desired, 0.0f);

  // Second call requests a 10.0 jump; slew limits it to rate*dt = 0.1.
  pid.Apply(0.0f, 10.0f, 0.0f, 0.0f, 0.1f);
  EXPECT_FLOAT_EQ(state.desired, 0.1f);
}

TEST(PidTest, DerivativeTermUsesRateError)
{
  PID::Config config;
  config.kd = 2.0f;
  PID::State state;
  PID pid(&config, &state);

  const float out = pid.Apply(0.0f, 0.0f, 3.0f, 1.0f, 0.001f);
  EXPECT_FLOAT_EQ(state.error_rate, 2.0f);
  EXPECT_FLOAT_EQ(out, 4.0f);
}

}  // namespace
}  // namespace servo_mode
}  // namespace math
