// PWM-rate snapshot acquisition and main-loop CAN publication.
#pragma once

#include <cstddef>
#include <cstdint>

#include "middleware/communication/binary_link.h"
#include "protocol/xt_can.h"

namespace middleware::snapshot
{

class SnapshotService
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
  bool capturing() const { return state_ == State::Capturing; }

  bool Arm(uint8_t sequence, uint16_t sample_count, uint8_t decimate,
           uint16_t nominal_sample_hz)
  {
    if (state_ != State::Idle)
    {
      return false;
    }
    if (sample_count == 0)
    {
      sample_count = protocol::xt_can::kSnapMaxSamples;
    }
    if (sample_count > protocol::xt_can::kSnapMaxSamples)
    {
      sample_count = protocol::xt_can::kSnapMaxSamples;
    }
    if (decimate == 0)
    {
      decimate = 1;
    }

    sequence_ = sequence;
    target_ = sample_count;
    decimate_ = decimate;
    nominal_sample_hz_ = nominal_sample_hz;
    count_ = 0;
    decimation_count_ = 0;
    send_index_ = 0;
    duration_us_ = 0;
    dt_total_us_ = 0;
    state_ = State::Capturing;
    return true;
  }

  void Abort() { state_ = State::Idle; }

  // ISR-only producer. The main loop owns state transitions after Ready.
  void PushIsr(float id_A, float iq_A, float i1_A, float i2_A, float i3_A,
               float theta_mech_rad, float theta_elec_rad, uint32_t dt_us)
  {
    if (state_ != State::Capturing)
    {
      return;
    }
    if (++decimation_count_ < decimate_)
    {
      return;
    }
    decimation_count_ = 0;

    const uint16_t index = count_;
    int16_t* row = &samples_[static_cast<size_t>(index) *
                            protocol::xt_can::kSnapChannelCount];
    row[0] = ToMilli16(id_A);
    row[1] = ToMilli16(iq_A);
    row[2] = ToMilli16(i1_A);
    row[3] = ToMilli16(i2_A);
    row[4] = ToMilli16(i3_A);
    row[5] = ToMilli16(theta_mech_rad);
    row[6] = ToMilli16(theta_elec_rad);
    duration_us_ += dt_us * decimate_;
    dt_total_us_ += dt_us * decimate_;
    if (++count_ >= target_)
    {
      state_ = State::Ready;
    }
  }

  void PollSend(middleware::communication::BinaryLink* link)
  {
    if (link == nullptr)
    {
      return;
    }
    if (state_ == State::Ready)
    {
      protocol::xt_can::SnapMeta meta{};
      FillMeta(&meta);
      if (!link->SendSnapMeta(meta))
      {
        return;
      }
      send_index_ = 0;
      state_ = State::Sending;
    }
    if (state_ != State::Sending)
    {
      return;
    }

    for (int frame_count = 0; frame_count < 16; ++frame_count)
    {
      if (send_index_ >= count_)
      {
        state_ = State::Idle;
        return;
      }
      protocol::xt_can::SnapData frame{};
      FillDataFrame(&frame);
      if (!link->SendSnapData(frame))
      {
        return;
      }
      send_index_ = static_cast<uint16_t>(send_index_ + frame.n);
    }
  }

 private:
  static int16_t ToMilli16(float value)
  {
    const float milli = value * 1000.0f;
    if (milli > 32767.0f) return 32767;
    if (milli < -32768.0f) return -32768;
    return static_cast<int16_t>(milli);
  }

  void FillMeta(protocol::xt_can::SnapMeta* meta) const
  {
    meta->hdr.seq = sequence_;
    meta->n_samples = count_;
    if (count_ > 0 && dt_total_us_ > 0)
    {
      const uint32_t average_us = static_cast<uint32_t>(dt_total_us_ / count_);
      meta->sample_hz = average_us > 0 ? 1000000u / average_us
                                      : nominal_sample_hz_;
    }
    else
    {
      meta->sample_hz = nominal_sample_hz_ / decimate_;
    }
    meta->channel_mask = protocol::xt_can::kSnapChDefault;
    meta->channels = protocol::xt_can::kSnapChannelCount;
    meta->decimate = decimate_;
    meta->duration_us = duration_us_;
  }

  void FillDataFrame(protocol::xt_can::SnapData* frame) const
  {
    const uint16_t remaining = static_cast<uint16_t>(count_ - send_index_);
    const uint8_t frame_samples =
        remaining > protocol::xt_can::kSnapSamplesPerFrame
            ? protocol::xt_can::kSnapSamplesPerFrame
            : static_cast<uint8_t>(remaining);
    frame->hdr.seq = sequence_;
    frame->start_index = send_index_;
    frame->n = frame_samples;
    frame->reserved = 0;

    const size_t words = static_cast<size_t>(frame_samples) *
                         protocol::xt_can::kSnapChannelCount;
    const int16_t* source =
        &samples_[static_cast<size_t>(send_index_) *
                  protocol::xt_can::kSnapChannelCount];
    for (size_t i = 0; i < words; ++i)
    {
      frame->samples[i] = source[i];
    }
    for (size_t i = words;
         i < sizeof(frame->samples) / sizeof(frame->samples[0]); ++i)
    {
      frame->samples[i] = 0;
    }
  }

  volatile State state_ = State::Idle;
  uint8_t sequence_ = 0;
  uint16_t target_ = 0;
  uint16_t count_ = 0;
  uint16_t send_index_ = 0;
  uint8_t decimate_ = 1;
  uint8_t decimation_count_ = 0;
  uint16_t nominal_sample_hz_ = 15000;
  uint64_t dt_total_us_ = 0;
  uint32_t duration_us_ = 0;
  int16_t samples_[protocol::xt_can::kSnapMaxSamples *
                   protocol::xt_can::kSnapChannelCount] = {};
};

}  // namespace middleware::snapshot
