#include "math/protection/safety_monitor.h"
#include "math/protection/thermistor.h"

#include "gtest/gtest.h"

namespace math {
namespace protection {
namespace {

TEST(ThermistorTest, RoomTemperatureNear25C)
{
  Thermistor ntc;
  ntc.r25_ohm = 47000.0f;
  // 10 kΩ to GND, 47 kΩ NTC to 3.3 V (moteus vt_sense):
  // V = 3.3 * 10k / (47k+10k) ≈ 0.579 V → raw ≈ 719
  float temp_C = 0.0f;
  ASSERT_TRUE(ntc.ToCelsius(719, &temp_C));
  EXPECT_NEAR(temp_C, 25.0f, 2.0f);
}

TEST(ThermistorTest, RejectsOpenAndShort)
{
  Thermistor ntc;
  float temp_C = 0.0f;
  EXPECT_FALSE(ntc.ToCelsius(0, &temp_C));
  EXPECT_FALSE(ntc.ToCelsius(4095, &temp_C));
}

TEST(SafetyMonitorTest, InvalidCurrentStopsImmediately)
{
  SafetyMonitor monitor;
  SafetyMonitor::Config config;
  config.overcurrent_A = 10.0f;
  monitor.set_config(config);

  SafetyMonitor::Input in;
  in.current_ok = false;
  in.dt_s = 1.0e-4f;
  EXPECT_EQ(monitor.Evaluate(in), Trip::CurrentSenseInvalid);
}

TEST(SafetyMonitorTest, PeakOvercurrentNeedsConsecutiveHits)
{
  SafetyMonitor monitor;
  SafetyMonitor::Config config;
  config.overcurrent_A = 10.0f;
  config.overcurrent_count = 5;
  monitor.set_config(config);

  SafetyMonitor::Input in;
  in.current_ok = true;
  in.i1_A = 12.0f;
  in.dt_s = 6.67e-5f;
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_EQ(monitor.Evaluate(in), Trip::None);
  }
  EXPECT_EQ(monitor.Evaluate(in), Trip::PeakOvercurrent);
}

TEST(SafetyMonitorTest, I2tIntegratesAboveRatedCurrent)
{
  SafetyMonitor monitor;
  SafetyMonitor::Config config;
  config.i2t_rated_A = 5.0f;
  config.i2t_tau_s = 1.0f;
  monitor.set_config(config);

  SafetyMonitor::Input in;
  in.current_ok = true;
  in.i1_A = 10.0f;
  in.dt_s = 0.05f;
  Trip trip = Trip::None;
  for (int i = 0; i < 30 && trip == Trip::None; ++i)
  {
    trip = monitor.Evaluate(in);
  }
  EXPECT_EQ(trip, Trip::I2t);
}

TEST(SafetyMonitorTest, I2tCoolsBelowRated)
{
  SafetyMonitor monitor;
  SafetyMonitor::Config config;
  config.i2t_rated_A = 5.0f;
  config.i2t_tau_s = 10.0f;
  monitor.set_config(config);

  SafetyMonitor::Input in;
  in.current_ok = true;
  in.i1_A = 7.0f;
  in.dt_s = 0.1f;
  (void)monitor.Evaluate(in);
  EXPECT_GT(monitor.i2t_energy_A2s(), 0.0f);
  in.i1_A = 0.0f;
  for (int i = 0; i < 50; ++i)
  {
    (void)monitor.Evaluate(in);
  }
  EXPECT_FLOAT_EQ(monitor.i2t_energy_A2s(), 0.0f);
}

TEST(SafetyMonitorTest, BusAndFetLimits)
{
  SafetyMonitor monitor;
  SafetyMonitor::Config config;
  config.bus_min_V = 8.0f;
  config.bus_max_V = 54.0f;
  config.fet_fault_C = 100.0f;
  monitor.set_config(config);

  SafetyMonitor::Input in;
  in.current_ok = true;
  in.bus_ok = true;
  in.bus_V = 7.0f;
  EXPECT_EQ(monitor.Evaluate(in), Trip::BusUndervoltage);

  monitor.Reset();
  in.bus_V = 60.0f;
  EXPECT_EQ(monitor.Evaluate(in), Trip::BusOvervoltage);

  monitor.Reset();
  in.bus_V = 48.0f;
  in.fet_ok = true;
  in.fet_temp_C = 110.0f;
  EXPECT_EQ(monitor.Evaluate(in), Trip::FetOvertemperature);
}

TEST(SafetyMonitorTest, MissingBusSampleDoesNotTrip)
{
  SafetyMonitor monitor;
  SafetyMonitor::Input in;
  in.current_ok = true;
  in.bus_ok = false;
  in.bus_V = 0.0f;
  EXPECT_EQ(monitor.Evaluate(in), Trip::None);
}

}  // namespace
}  // namespace protection
}  // namespace math
