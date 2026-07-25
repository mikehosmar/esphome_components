#pragma once

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <cmath>

namespace esphome::pid {

struct PIDController {
  float update(float setpoint, float process_value);

  void reset_accumulated_integral() { accumulated_integral_ = 0; }
  void set_starting_integral_term(float in) { accumulated_integral_ = in; }

  bool in_deadband();

  /// Reset Smith predictor internal state (model output, delayed history, last control output).
  void reset_smith_predictor();

  /// Returns true when the Smith predictor has valid runtime configuration.
  bool smith_predictor_active() const {
    return smith_enabled_ && smith_time_constant_ > 0.0f && smith_dead_time_ >= 0.0f &&
           !std::isnan(smith_gain_) && smith_delay_buffer_.capacity() > 0;
  }

  friend class PIDClimate;

 private:
  /// Proportional gain K_p.
  float kp_ = 0;
  /// Integral gain K_i.
  float ki_ = 0;
  /// Differential gain K_d.
  float kd_ = 0;

  // smooth the derivative value using an average over X samples
  int derivative_samples_ = 1;

  /// smooth the output value using an average over X values
  int output_samples_ = 1;

  float threshold_low_ = 0.0f;
  float threshold_high_ = 0.0f;
  float kp_multiplier_ = 0.0f;
  float ki_multiplier_ = 0.0f;
  float kd_multiplier_ = 0.0f;
  int deadband_output_samples_ = 1;

  float min_integral_ = NAN;
  float max_integral_ = NAN;

  // Store computed values in struct so that values can be monitored through sensors
  float error_;
  float dt_;
  float proportional_term_;
  float integral_term_;
  float derivative_term_;

  // -----------------------------
  // Smith predictor (FOPDT) config
  // -----------------------------
  // When enabled, the PID error is computed from a predicted process value
  //   y_hat(t) = y_m(t) + (y(t) - y_m(t - L))
  // where y_m is the output of an internal FOPDT model driven by the previous
  // controller output, and L is the dead time.
  bool smith_enabled_ = false;
  float smith_gain_ = NAN;            // K   — steady-state process gain
  float smith_time_constant_ = 0.0f;  // tau — first-order time constant [s]
  float smith_dead_time_ = 0.0f;      // L   — pure dead time [s]

  // Runtime state for the predictor
  float smith_model_pv_ = NAN;         // y_m(t)      — undelayed model output
  float smith_model_pv_delayed_ = 0;   // y_m(t-L)    — delayed model output
  float smith_compensation_ = 0;       // y(t) - y_m(t-L)
  float smith_predicted_pv_ = NAN;     // y_hat(t)    — predicted process value
  float smith_last_output_ = 0.0f;     // u(t-1)      — controller output driving the model

  struct SmithSample {
    uint32_t timestamp_ms;
    float value;
  };
  // Ring buffer holding time-stamped no-delay model samples for dead-time lookup.
  FixedRingBuffer<SmithSample> smith_delay_buffer_;

  /// Advance the FOPDT model and compute compensation / predicted PV.
  /// Uses process_value as the current measurement y(t) and this->smith_last_output_
  /// as the previous controller output u(t-1).
  void advance_smith_predictor_(float process_value);

  void calculate_proportional_term_();
  void calculate_integral_term_();
  void calculate_derivative_term_(float setpoint);

  /// Ring buffer smoothing using FixedRingBuffer (single allocation at setup)
  float ring_buffer_average_(FixedRingBuffer<float> &buf, float new_value, int max_samples);

  float calculate_relative_time_();

  /// Error from previous update used for derivative term
  float previous_error_ = 0;
  float previous_setpoint_ = NAN;
  /// Accumulated integral value
  float accumulated_integral_ = 0;
  uint32_t last_time_ = 0;

  // Ring buffer for derivative smoothing
  FixedRingBuffer<float> derivative_window_;

  // Ring buffer for output smoothing (shared between normal and deadband modes)
  FixedRingBuffer<float> output_window_;

};  // Struct PIDController
}  // namespace esphome::pid
