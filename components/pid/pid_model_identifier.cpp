#include "pid_model_identifier.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cinttypes>

namespace esphome::pid {

static const char *const TAG = "pid.identify";

PIDModelIdentifier::UpdateResult PIDModelIdentifier::update(float process_variable) {
  UpdateResult res{0.0f, {}};
  const uint32_t now_ms = millis();

  if (state_ == IDENT_SUCCEEDED || state_ == IDENT_FAILED) {
    res.output = 0.0f;
    if (state_ == IDENT_SUCCEEDED)
      res.result_params = result_;
    return res;
  }

  if (state_ == IDENT_INIT) {
    start_ms_ = now_ms;
    baseline_samples_.clear();
    step_samples_.clear();
    baseline_samples_.reserve(128);
    step_samples_.reserve(1024);
    state_ = IDENT_BASELINE;
    ESP_LOGI(TAG,
             "%s: Model identification started.\n"
             "  Holding output at %.3f for %.1fs to establish baseline.\n"
             "  Then stepping to %.3f (delta=%+0.3f) for up to %.1fs.",
             id_.c_str(), baseline_output_, baseline_duration_s_, step_output_,
             step_output_ - baseline_output_, max_test_duration_s_);
  }

  const float elapsed_s = (now_ms - start_ms_) / 1000.0f;

  // Safety: overall time cap.
  if (elapsed_s > max_test_duration_s_ + baseline_duration_s_) {
    ESP_LOGW(TAG, "%s: Identification timed out after %.1fs.", id_.c_str(), elapsed_s);
    failure_reason_ = "test duration exceeded max_test_duration";
    state_ = IDENT_FAILED;
    res.output = 0.0f;
    return res;
  }

  if (state_ == IDENT_BASELINE) {
    if (!std::isnan(process_variable))
      baseline_samples_.push_back({now_ms, process_variable});
    res.output = baseline_output_;

    if (elapsed_s >= baseline_duration_s_) {
      if (baseline_samples_.size() < 3) {
        ESP_LOGW(TAG, "%s: Baseline phase collected only %zu samples — need at least 3.", id_.c_str(),
                 baseline_samples_.size());
        failure_reason_ = "insufficient baseline samples";
        state_ = IDENT_FAILED;
        return res;
      }
      // Compute baseline mean and noise stddev.
      float sum = 0.0f;
      for (const auto &s : baseline_samples_)
        sum += s.pv;
      baseline_pv_ = sum / baseline_samples_.size();
      float ss = 0.0f;
      for (const auto &s : baseline_samples_) {
        float d = s.pv - baseline_pv_;
        ss += d * d;
      }
      baseline_noise_ = std::sqrt(ss / baseline_samples_.size());
      start_step_(now_ms);
      ESP_LOGI(TAG, "%s: Baseline established: T0=%.3f, noise sigma=%.3f. Applying step.", id_.c_str(),
               baseline_pv_, baseline_noise_);
    }
    return res;
  }

  // IDENT_STEP
  res.output = step_output_;
  if (!std::isnan(process_variable))
    step_samples_.push_back({now_ms, process_variable});

  const float step_elapsed_s = (now_ms - step_start_ms_) / 1000.0f;

  // Early plateau detection: once we have a reasonable amount of data, check
  // whether the slope of the last 20% of samples (min 10s, max 60s window) is
  // below the configured threshold expressed as %/min of the current delta.
  bool early_plateau = false;
  if (step_samples_.size() >= 10 && step_elapsed_s > 20.0f) {
    const float window_s = std::clamp(step_elapsed_s * 0.2f, 10.0f, 60.0f);
    const uint32_t window_start_ms = now_ms - static_cast<uint32_t>(window_s * 1000.0f);
    float first_pv = NAN;
    uint32_t first_t = 0;
    for (const auto &s : step_samples_) {
      if (s.t_ms >= window_start_ms) {
        first_pv = s.pv;
        first_t = s.t_ms;
        break;
      }
    }
    if (!std::isnan(first_pv)) {
      const float last_pv = step_samples_.back().pv;
      const float dt_s = (step_samples_.back().t_ms - first_t) / 1000.0f;
      const float delta_from_baseline = std::fabs(last_pv - baseline_pv_);
      // Only trust plateau detection after we've seen a meaningful rise.
      if (dt_s > 5.0f && delta_from_baseline > 10.0f * std::max(baseline_noise_, 0.01f)) {
        const float slope_per_min = (last_pv - first_pv) / dt_s * 60.0f;
        const float relative_slope = std::fabs(slope_per_min) / std::max(delta_from_baseline, 0.01f);
        if (relative_slope < plateau_slope_per_min_) {
          ESP_LOGI(TAG, "%s: Response plateau detected after %.1fs (slope=%.4f/min).", id_.c_str(),
                   step_elapsed_s, slope_per_min);
          early_plateau = true;
        }
      }
    }
  }

  if (early_plateau || step_elapsed_s >= max_test_duration_s_) {
    if (analyze_()) {
      state_ = IDENT_SUCCEEDED;
      res.result_params = result_;
    } else {
      state_ = IDENT_FAILED;
    }
    dump_config();
    // Release sample memory now that we're done.
    baseline_samples_.clear();
    baseline_samples_.shrink_to_fit();
    step_samples_.clear();
    step_samples_.shrink_to_fit();
    res.output = 0.0f;
  }

  return res;
}

void PIDModelIdentifier::start_step_(uint32_t now_ms) {
  step_start_ms_ = now_ms;
  state_ = IDENT_STEP;
}

float PIDModelIdentifier::interpolate_time_at_(const std::vector<Sample> &data, uint32_t t0_ms, float target_pv,
                                               uint32_t t_start_ms) {
  // Return the (relative-to-t0_ms) time in seconds at which pv first crosses
  // target_pv, using linear interpolation between adjacent samples. Returns
  // NAN if never reached. Only considers samples with t_ms >= t_start_ms.
  for (size_t i = 1; i < data.size(); ++i) {
    if (data[i].t_ms < t_start_ms)
      continue;
    if (data[i].pv >= target_pv) {
      const Sample &a = data[i - 1];
      const Sample &b = data[i];
      float frac = 0.0f;
      if (b.pv != a.pv)
        frac = (target_pv - a.pv) / (b.pv - a.pv);
      frac = std::clamp(frac, 0.0f, 1.0f);
      float t_cross_ms = static_cast<float>(a.t_ms) + frac * static_cast<float>(b.t_ms - a.t_ms);
      return (t_cross_ms - static_cast<float>(t0_ms)) / 1000.0f;
    }
  }
  return NAN;
}

bool PIDModelIdentifier::analyze_() {
  if (step_samples_.size() < 10) {
    failure_reason_ = "not enough step samples to analyze";
    return false;
  }

  // Final value: mean of the last 5% of samples (min 3, max 20).
  const size_t n = step_samples_.size();
  size_t tail = std::clamp<size_t>(n / 20, 3, 20);
  tail = std::min(tail, n);
  float sum = 0.0f;
  for (size_t i = n - tail; i < n; ++i)
    sum += step_samples_[i].pv;
  const float final_pv = sum / tail;

  const float delta = final_pv - baseline_pv_;
  if (std::fabs(delta) < 5.0f * std::max(baseline_noise_, 0.01f)) {
    failure_reason_ = "process did not respond significantly to the step (delta too small vs noise)";
    return false;
  }
  const float delta_u = step_output_ - baseline_output_;
  if (delta_u == 0.0f) {
    failure_reason_ = "step_output equals baseline_output (delta_u is zero)";
    return false;
  }

  const float target_28 = baseline_pv_ + 0.283f * delta;
  const float target_63 = baseline_pv_ + 0.632f * delta;

  const float t28 = interpolate_time_at_(step_samples_, step_start_ms_, target_28, step_start_ms_);
  const float t63 = interpolate_time_at_(step_samples_, step_start_ms_, target_63, step_start_ms_);

  if (std::isnan(t28) || std::isnan(t63) || t63 <= t28) {
    failure_reason_ = "could not locate 28% / 63% response crossings";
    return false;
  }

  const float tau = 1.5f * (t63 - t28);
  const float dead_time = std::max(0.0f, t63 - tau);
  const float gain = delta / delta_u;

  if (!(tau > 0.0f) || !std::isfinite(gain)) {
    failure_reason_ = "computed model is not physically valid";
    return false;
  }

  result_.gain = gain;
  result_.time_constant = tau;
  result_.dead_time = dead_time;
  return true;
}

void PIDModelIdentifier::dump_config() {
  if (state_ == IDENT_SUCCEEDED) {
    ESP_LOGI(TAG,
             "%s: FOPDT Model Identification:\n"
             "  State: Succeeded!\n"
             "  Baseline output: %.3f  =>  Baseline PV: %.3f (noise sigma %.3f)\n"
             "  Step output: %.3f (delta_u=%+0.3f), duration %.1fs, %zu samples\n"
             "\n"
             "  Copy into your PID control_parameters block:\n"
             "\n"
             "  smith_predictor:\n"
             "    dead_time: %.2fs\n"
             "    time_constant: %.2fs\n"
             "    gain: %.4f\n",
             id_.c_str(), baseline_output_, baseline_pv_, baseline_noise_,
             step_output_, step_output_ - baseline_output_,
             (step_samples_.empty() ? 0.0f : (step_samples_.back().t_ms - step_start_ms_) / 1000.0f),
             step_samples_.size(), result_.dead_time, result_.time_constant, result_.gain);
    return;
  }
  if (state_ == IDENT_FAILED) {
    ESP_LOGW(TAG, "%s: FOPDT Model Identification failed: %s", id_.c_str(),
             failure_reason_ != nullptr ? failure_reason_ : "unknown reason");
    return;
  }
  ESP_LOGD(TAG,
           "%s: FOPDT Model Identification running.\n"
           "  Phase: %s, elapsed %.1fs\n"
           "  Baseline samples: %zu, step samples: %zu",
           id_.c_str(), state_ == IDENT_BASELINE ? "baseline" : "step",
           (millis() - start_ms_) / 1000.0f, baseline_samples_.size(), step_samples_.size());
}

}  // namespace esphome::pid
