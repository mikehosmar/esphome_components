#include "pid_climate_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::pid {

static const char *const TAG = "pid.sensor";

void PIDClimateSensor::setup() {
  this->parent_->add_on_pid_computed_callback([this]() { this->update_from_parent_(); });
  this->update_from_parent_();
}
void PIDClimateSensor::update_from_parent_() {
  float value;
  switch (this->type_) {
    case PID_SENSOR_TYPE_RESULT:
      value = this->parent_->get_output_value();
      break;
    case PID_SENSOR_TYPE_ERROR:
      value = this->parent_->get_error_value();
      break;
    case PID_SENSOR_TYPE_PROPORTIONAL:
      value = this->parent_->get_proportional_term();
      break;
    case PID_SENSOR_TYPE_INTEGRAL:
      value = this->parent_->get_integral_term();
      break;
    case PID_SENSOR_TYPE_DERIVATIVE:
      value = this->parent_->get_derivative_term();
      break;
    case PID_SENSOR_TYPE_HEAT:
      value = clamp(this->parent_->get_output_value(), 0.0f, 1.0f);
      break;
    case PID_SENSOR_TYPE_COOL:
      value = clamp(-this->parent_->get_output_value(), 0.0f, 1.0f);
      break;
    case PID_SENSOR_TYPE_KP:
      value = this->parent_->get_kp();
      this->publish_state(value);
      return;
    case PID_SENSOR_TYPE_KI:
      value = this->parent_->get_ki();
      this->publish_state(value);
      return;
    case PID_SENSOR_TYPE_KD:
      value = this->parent_->get_kd();
      this->publish_state(value);
      return;
    case PID_SENSOR_TYPE_SMITH_PREDICTED_PV:
      // Predicted process value is in temperature units — publish raw.
      this->publish_state(this->parent_->get_smith_predicted_pv());
      return;
    case PID_SENSOR_TYPE_SMITH_MODEL_PV:
      // Undelayed model output, temperature units — publish raw.
      this->publish_state(this->parent_->get_smith_model_pv());
      return;
    case PID_SENSOR_TYPE_SMITH_MODEL_PV_DELAYED:
      // Delayed model output, temperature units — publish raw.
      this->publish_state(this->parent_->get_smith_model_pv_delayed());
      return;
    case PID_SENSOR_TYPE_SMITH_COMPENSATION:
      // Compensation term (y - y_m_delayed), temperature units — publish raw.
      this->publish_state(this->parent_->get_smith_compensation());
      return;
    default:
      value = NAN;
      break;
  }
  this->publish_state(value * 100.0f);
}
void PIDClimateSensor::dump_config() { LOG_SENSOR("", "PID Climate Sensor", this); }

}  // namespace esphome::pid
