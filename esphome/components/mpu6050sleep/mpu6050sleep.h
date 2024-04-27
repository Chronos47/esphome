#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mpu6050sleep {

template<typename... Ts> class SleepAction;

template<typename... Ts> class WakeAction;

class MPU6050SleepComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;

  void update() override;

  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }

  uint8_t read_register8(uint8_t reg);
  void write_register_bit(uint8_t reg, uint8_t pos, bool state);
  void write_register8(uint8_t reg, uint8_t value);
  void low_power_accel(uint8_t frequency);

  void sleep();
  void wake();

 protected:
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
};

template<typename... Ts> class SleepAction : public Action<Ts...>, public Parented<MPU6050SleepComponent> {
 public:
  void play(Ts... x) override { this->parent_->sleep(); }
};

template<typename... Ts> class WakeAction : public Action<Ts...>, public Parented<MPU6050SleepComponent> {
 public:
  void play(Ts... x) override { this->parent_->wake(); }
};

}  // namespace mpu6050sleep
}  // namespace esphome
