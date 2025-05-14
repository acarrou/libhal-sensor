// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace hal::sensor {

/**
 * @brief Driver for the MPL3115A2 pressure/altitude/temperature sensor
 */
class mpl3115a2
{
public:
  /**
   * @brief Defines the operation mode of the sensor
   */
  enum class mode : hal::byte
  {
    /// Barometer mode for pressure readings
    barometer = 0,
    /// Altimeter mode for altitude readings
    altimeter = 1,
  };

  /**
   * @brief Stores the temperature data from temperature readings.
   */
  struct temperature_results
  {
    hal::celsius temperature;
  };

  /**
   * @brief Stores the pressure data from pressure readings.
   */
  struct pressure_results
  {
    /// Pressure is in units of Pascals
    float pressure;
  };

  /**
   * @brief Stores the altitude data from altitude readings.
   */
  struct altitude_results
  {
    hal::meters altitude;
  };

  /**
   * @brief Construct a mpl3115a2 driver
   *
   * @param p_i2c - The driver for the I2C bus the MPL3115A2 is connected to.
   *
   * @throws hal::no_such_device - when an invalid MPL3115A2 device is detected.
   * MPL3115A2 devices have a read-only ID register which allows a microcontroller
   * to determine what device it is connected to. This register will be read and
   * if it does not match the expected value, this exception is thrown.
   */
  explicit mpl3115a2(hal::i2c& p_i2c);

  /**
   * @brief Reads the temperature
   *
   * @returns the temperature in celsius.
   */
  [[nodiscard]] temperature_results read_temperature();

  /**
   * @brief Reads the pressure
   *
   * @returns the pressure in Pascals.
   */
  [[nodiscard]] pressure_results read_pressure();

  /**
   * @brief Reads the altitude
   *
   * @returns the altitude in meters.
   */
  [[nodiscard]] altitude_results read_altitude();

  /**
   * @brief Set sea level pressure (Barometric input for altitude calculations)
   *
   * @param p_sea_level_pressure - Sea level pressure in Pascals.
   *        Default value on startup is 101,326 Pa.
   */
  void set_sea_pressure(float p_sea_level_pressure);

  /**
   * @brief Set altitude offset
   *
   * @param p_offset - Offset value in meters, from -127 to 128
   */
  void set_altitude_offset(int8_t p_offset);

private:
  /// Maximum number of retries for polling operations
  static constexpr uint16_t default_max_polling_retries = 10000;
  /// The I2C peripheral used for communication with the device
  hal::i2c* m_i2c;
  /// Variable to track current sensor mode
  mode m_sensor_mode;
};

}  // namespace hal::sensor