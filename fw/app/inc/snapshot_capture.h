// PWM-rate current snapshot buffer (ISR fill, main-loop CAN dump).
#pragma once

#include <cstdint>

#include "telemetry/xt_can.h"

namespace app
{

class SnapshotCapture
{
 public:
  enum class State : uint8_t
  {
    Idle = 0,
    Capturing = 1,
    Ready = 2,
    Sending = 3,
  };

  State state() const { return state_; }
  bool busy() const { return state_ != State::Idle; }

  // Arm capture. Returns false if busy or bad args.
  bool Arm(uint16_t n_samples, uint8_t decimate, uint16_t sample_hz)
  {
    if (state_ != State::Idle)
    {
      return false;
    }
    if (n_samples == 0)
    {
      n_samples = telemetry::xt_can::kSnapMaxSamples;
    }
    if (n_samples > telemetry::xt_can::kSnapMaxSamples)
    {
      n_samples = telemetry::xt_can::kSnapMaxSamples;
    }
    if (decimate == 0)
    {
      decimate = 1;
    }
    target_ = n_samples;
    decimate_ = decimate;
    sample_hz_ = sample_hz;
    count_ = 0;
    decim_count_ = 0;
    send_index_ = 0;
    duration_us_ = 0;
    state_ = State::Capturing;
    return true;
  }

  // Called from control ISR (PWM rate).
  void PushIsr(float id_A, float iq_A, float i1_A, float i2_A, float i3_A,
               uint32_t dt_us)
  {
    if (state_ != State::Capturing)
    {
      return;
    }
    ++decim_count_;
    if (decim_count_ < decimate_)
    {
      return;
    }
    decim_count_ = 0;

    const uint16_t i = count_;
    int16_t* row = &buf_[static_cast<size_t>(i) * telemetry::xt_can::kSnapChannelCount];
    row[0] = ToMilli16(id_A);
    row[1] = ToMilli16(iq_A);
    row[2] = ToMilli16(i1_A);
    row[3] = ToMilli16(i2_A);
    row[4] = ToMilli16(i3_A);
    duration_us_ += dt_us * decimate_;
    ++count_;
    if (count_ >= target_)
    {
      state_ = State::Ready;
    }
  }

  void BeginSend()
  {
    if (state_ == State::Ready)
    {
      send_index_ = 0;
      state_ = State::Sending;
    }
  }

  void Finish() { state_ = State::Idle; }

  uint16_t count() const { return count_; }
  uint16_t target() const { return target_; }
  uint16_t sample_hz() const { return sample_hz_; }
  uint8_t decimate() const { return decimate_; }
  uint32_t duration_us() const { return duration_us_; }
  uint16_t send_index() const { return send_index_; }

  void FillMeta(telemetry::xt_can::SnapMeta* meta, uint8_t seq) const
  {
    meta->hdr.seq = seq;
    meta->n_samples = count_;
    meta->sample_hz = sample_hz_ / (decimate_ ? decimate_ : 1);
    meta->channel_mask = telemetry::xt_can::kSnapChDefault;
    meta->channels = telemetry::xt_can::kSnapChannelCount;
    meta->decimate = decimate_;
    meta->duration_us = duration_us_;
  }

  // Fill one SnapData frame; returns false when done.
  bool FillDataFrame(telemetry::xt_can::SnapData* out, uint8_t seq)
  {
    if (state_ != State::Sending || send_index_ >= count_)
    {
      return false;
    }
    const uint16_t remain = static_cast<uint16_t>(count_ - send_index_);
    const uint8_t n =
        remain > telemetry::xt_can::kSnapSamplesPerFrame
            ? telemetry::xt_can::kSnapSamplesPerFrame
            : static_cast<uint8_t>(remain);

    out->hdr.seq = seq;
    out->start_index = send_index_;
    out->n = n;
    out->reserved = 0;
    const size_t words =
        static_cast<size_t>(n) * telemetry::xt_can::kSnapChannelCount;
    const int16_t* src =
        &buf_[static_cast<size_t>(send_index_) * telemetry::xt_can::kSnapChannelCount];
    for (size_t i = 0; i < words; ++i)
    {
      out->samples[i] = src[i];
    }
    // Clear unused tail for determinism.
    for (size_t i = words;
         i < sizeof(out->samples) / sizeof(out->samples[0]); ++i)
    {
      out->samples[i] = 0;
    }
    send_index_ = static_cast<uint16_t>(send_index_ + n);
    return true;
  }

 private:
  static int16_t ToMilli16(float a)
  {
    const float m = a * 1000.0f;
    if (m > 32767.0f)
    {
      return 32767;
    }
    if (m < -32768.0f)
    {
      return -32768;
    }
    return static_cast<int16_t>(m);
  }

  volatile State state_ = State::Idle;
  uint16_t target_ = 0;
  uint16_t count_ = 0;
  uint16_t send_index_ = 0;
  uint8_t decimate_ = 1;
  uint8_t decim_count_ = 0;
  uint16_t sample_hz_ = 15000;
  uint32_t duration_us_ = 0;
  int16_t buf_[telemetry::xt_can::kSnapMaxSamples *
               telemetry::xt_can::kSnapChannelCount] = {};
};

}  // namespace app
