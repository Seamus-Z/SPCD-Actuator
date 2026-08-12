// EncoderPll: wrap-around unwinding, spike rejection, velocity clamps.
#include "math/servo_mode/encoder_pll.h"

#include <cmath>

#include "gtest/gtest.h"
#include "math/constants.h"

namespace math { namespace servo_mode {
namespace {

constexpr float kDt = 1.0f / 15000.0f;  // firmware control period

EncoderPll::Options DefaultOptions()
{
  EncoderPll::Options options;
  options.pll_filter_hz = 400.0f;
  options.pole_pairs = 14.0f;
  options.spike_error_rad = 0.15f;
  return options;
}

TEST(EncoderPllTest, FirstUpdateActsAsReset)
{
  EncoderPll pll(DefaultOptions());
  EXPECT_FALSE(pll.theta_valid());

  pll.Update(1.5f, kDt);
  EXPECT_TRUE(pll.theta_valid());
  EXPECT_FLOAT_EQ(pll.filtered_mech_rad(), 1.5f);
  EXPECT_FLOAT_EQ(pll.position_rad(), 1.5f);
  EXPECT_FLOAT_EQ(pll.velocity_mech(), 0.0f);
}

TEST(EncoderPllTest, TracksConstantVelocityAcrossWrap)
{
  EncoderPll pll(DefaultOptions());
  const float w = 10.0f;  // rad/s, crosses the 2π boundary repeatedly

  float theta = 6.0f;  // start just below wrap
  pll.Reset(theta);
  const int steps = 15000;  // 1 s
  for (int i = 0; i < steps; ++i)
  {
    theta += w * kDt;
    pll.Update(math::WrapZeroToTwoPi(theta), kDt);
  }
  // Unwrapped position must be continuous through the wraps, and the tracked
  // velocity must converge to the true rate.
  EXPECT_NEAR(pll.velocity_mech(), w, 0.05f);
  EXPECT_NEAR(pll.position_rad(), theta, 0.05f);
  EXPECT_EQ(pll.spike_count(), 0u);
}

TEST(EncoderPllTest, SingleSampleSpikeIsCountedAndClamped)
{
  EncoderPll pll(DefaultOptions());
  pll.Reset(1.0f);
  for (int i = 0; i < 1000; ++i)
  {
    pll.Update(1.0f, kDt);  // settle at standstill
  }
  ASSERT_EQ(pll.spike_count(), 0u);

  // One glitched sample: 0.3 rad jump (above the 0.15 rad threshold).
  pll.Update(1.3f, kDt);
  EXPECT_GE(pll.spike_count(), 1u);
  EXPECT_NEAR(pll.last_spike_error_rad(), 0.3f, 1e-3f);

  // Recovery: back to the real angle, velocity settles to ~0 again.
  for (int i = 0; i < 2000; ++i)
  {
    pll.Update(1.0f, kDt);
  }
  EXPECT_NEAR(pll.velocity_mech(), 0.0f, 0.1f);
}

TEST(EncoderPllTest, AbsoluteVelocityCeilingApplies)
{
  EncoderPll::Options options = DefaultOptions();
  options.max_velocity_mech_rad_s = 5.0f;
  options.spike_error_rad = 0.0f;  // isolate the velocity clamp
  EncoderPll pll(options);

  float theta = 0.0f;
  pll.Reset(theta);
  const float w = 50.0f;  // 10x above the ceiling
  for (int i = 0; i < 5000; ++i)
  {
    theta += w * kDt;
    pll.Update(math::WrapZeroToTwoPi(theta), kDt);
    EXPECT_LE(std::fabs(pll.velocity_mech()), 5.0f + 1e-3f);
  }
}

TEST(EncoderPllTest, ElectricalThetaFollowsFilteredTimesPolePairs)
{
  EncoderPll pll(DefaultOptions());
  pll.Reset(0.5f, 0.25f);
  const float expected =
      math::WrapZeroToTwoPi(0.5f * 14.0f + 0.25f);
  EXPECT_NEAR(pll.electrical_theta(), expected, 1e-5f);
}

TEST(EncoderPllTest, TinyDtDoesNotBlowUp)
{
  EncoderPll pll(DefaultOptions());
  pll.Reset(1.0f);
  pll.Update(1.001f, 0.0f);  // dt=0 gets clamped internally
  EXPECT_TRUE(std::isfinite(pll.velocity_mech()));
  EXPECT_TRUE(std::isfinite(pll.position_rad()));
}

}  // namespace
}  // namespace servo_mode
}  // namespace math
