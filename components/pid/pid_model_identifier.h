#pragma once

#include "esphome/core/component.h"
#include "esphome/core/optional.h"

#include <cstdint>
#include <string>
#include <vector>

namespace esphome::pid {

/**
 * PIDModelIdentifier
 * -------------------
 * Open-loop first-order-plus-dead-time (FOPDT) process identifier.
 *
 * Runs a "bump test":
 *   1. Hold the controller output at 0 for `baseline_duration_` seconds and
 *      collect samples to establish a baseline temperature (T0) and noise level.
 *   2. Step the output up to `step_output_` and record the process response
 *      until it plateaus or `max_test_duration_` elapses.
 *   3. Fit an FOPDT model using the classic two-point method:
 *          K   = (T_final - T0) / step_output
 *          tau = 1.5 * (t_63 - t_28)
 *          L   = t_63 - tau
 *      where t_28 and t_63 are the times to reach 28.3% and 63.2% of the
 *      total temperature rise from T0.
 *
 * While the identifier is running the PID output is fully overridden — the
 * regular PID error/integral/derivative calculations are skipped so they do
 * not fight the bump test.
 */
class PIDModelIdentifier {
 public:
  struct FOPDTResult {
    float gain;           ///< Process gain K (delta-PV per unit output)
    float time_constant;  ///< First-order time constant tau (seconds)
    float dead_time;      ///< Pure dead time L (seconds)
  };
  struct UpdateResult {
    float output;                       ///< Controller output to apply this step
    optional<FOPDTResult> result_params;///< Populated once identification completes
  };

  /// Called every PID update while identification is running.
  UpdateResult update(float process_variable);

  bool is_finished() const { return state_ == IDENT_SUCCEEDED || state_ == IDENT_FAILED; }
  bool succeeded() const { return state_ == IDENT_SUCCEEDED; }

  void dump_config();

  void set_identifier_id(std::string id) { id_ = std::move(id); }
  /// Bump size, in the same units the PID output uses (e.g. 0.30 = 30% heat).
  /// Positive drives a heater; negative drives a cooler.
  void set_step_output(float v) { step_output_ = v; }
  /// How long to hold output at 0 before applying the step, in seconds.
  void set_baseline_duration(float s) { baseline_duration_s_ = s; }
  /// Hard safety cap on total test time (baseline + step), in seconds.
  void set_max_test_duration(float s) { max_test_duration_s_ = s; }
  /// Percent-of-final-value slope threshold considered "settled" (0.0 .. 1.0
  /// per minute). If the response slope over the last plateau_window_
  /// seconds is below this, the step phase ends early. Default 0.01 = 1%/min.
  void set_plateau_slope(float slope_per_min) { plateau_slope_per_min_ = slope_per_min; }

 protected:
  enum State {
    IDENT_INIT,
    IDENT_BASELINE,
    IDENT_STEP,
    IDENT_SUCCEEDED,
    IDENT_FAILED,
  } state_ = IDENT_INIT;

  struct Sample {
    uint32_t t_ms;
    float pv;
  };

  void start_step_(uint32_t now_ms);
  bool analyze_();
  static float interpolate_time_at_(const std::vector<Sample> &data, uint32_t t0_ms, float target_pv, uint32_t t_start_ms);

  // Configuration
  float step_output_ = 0.3f;
  float baseline_duration_s_ = 30.0f;
  float max_test_duration_s_ = 600.0f;  // 10 minutes safety cap
  float plateau_slope_per_min_ = 0.01f; // 1%/minute

  // Runtime state
  std::string id_;
  uint32_t start_ms_ = 0;
  uint32_t step_start_ms_ = 0;
  float baseline_pv_ = NAN;
  float baseline_noise_ = 0.0f;
  std::vector<Sample> baseline_samples_;
  std::vector<Sample> step_samples_;

  // Results
  FOPDTResult result_{NAN, NAN, NAN};
  const char *failure_reason_ = nullptr;
};

}  // namespace esphome::pid
