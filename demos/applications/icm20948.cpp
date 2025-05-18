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

#include <cmath>
#include <numbers>

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

float compute_heading(float x, float y, float offset = 0.0)
{
  float angle = 360 - (atan2(y, x) * (180.0 / std::numbers::pi));
  angle += offset;  // Apply offset
  if (angle < 0) {
    angle += 360;
  } else if (angle >= 360) {
    angle -= 360;
  }
  return angle;
}

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "ICM20948 Application Starting...\n\n");
  hal::delay(clock, 200ms);
  
  // First scan for I2C devices
  hal::print(console, "Scanning I2C bus for devices...\n");
  
  for (hal::byte address = 0x08; address < 0x78; address++) {
    if (hal::probe(i2c, address)) {
      hal::print<64>(console, "  Found device at 0x%02X\n", address);
    }
  }
  
  // Try initializing the ICM20948
  try {
    hal::print(console, "\nInitializing ICM20948...\n");
    
    // Log extra debug info
    hal::print(console, "Looking for magnetometer at address 0x0C...\n");
    if (hal::probe(i2c, 0x0C)) {
      hal::print(console, "Found magnetometer at 0x0C\n");
    } else {
      hal::print(console, "WARNING: Magnetometer not detected at 0x0C\n");
    }
    
    hal::sensor::icm20948 icm_device(i2c);
    hal::print(console, "ICM20948 initialized successfully!\n");
    
    // Basic sensor test
    hal::print(console, "Reading initial values...\n");
    
    // Read each sensor
    try {
      auto accel = icm_device.read_acceleration();
      hal::print<64>(console, "Accel (g): x=%f, y=%f, z=%f\n", 
                     accel.x, accel.y, accel.z);
    }
    catch (const std::exception& e) {
      hal::print<64>(console, "Accel read error: %s\n", e.what());
    }
    
    try {
      auto gyro = icm_device.read_gyroscope();
      hal::print<64>(console, "Gyro (deg/s): x=%f, y=%f, z=%f\n", 
                     gyro.x, gyro.y, gyro.z);
    }
    catch (const std::exception& e) {
      hal::print<64>(console, "Gyro read error: %s\n", e.what());
    }
    
    try {
      auto temp = icm_device.read_temperature();
      hal::print<64>(console, "Temperature: %f °C\n", temp.temp);
    }
    catch (const std::exception& e) {
      hal::print<64>(console, "Temperature read error: %s\n", e.what());
    }
    
    try {
      auto mag = icm_device.read_magnetometer();
      hal::print<64>(console, "Mag: x=%f, y=%f, z=%f\n", 
                     mag.x, mag.y, mag.z);
                     
      float heading = compute_heading(mag.x, mag.y);
      hal::print<64>(console, "Heading: %f degrees\n", heading);
    }
    catch (const std::exception& e) {
      hal::print<64>(console, "Magnetometer read error: %s\n", e.what());
    }
    
    // Main loop
    hal::print(console, "\nEntering main loop...\n");
    
    while (true) {
      hal::print(console, "Reading sensors...\n");
      
      try {
        auto accel = icm_device.read_acceleration();
        auto gyro = icm_device.read_gyroscope();
        auto temp = icm_device.read_temperature();
        auto mag = icm_device.read_magnetometer();
        
        // Print with better formatting and rounding for readability
        hal::print<128>(console, "ACC[%6.3f, %6.3f, %6.3f] GYR[%7.2f, %7.2f, %7.2f]\n", 
                      accel.x, accel.y, accel.z,
                      gyro.x, gyro.y, gyro.z);
        
        hal::print<128>(console, "MAG[%6.2f, %6.2f, %6.2f] TEMP[%5.1f°C]\n",
                      mag.x, mag.y, mag.z,
                      temp.temp);
                      
        // Calculate heading if magnetometer readings are valid
        if (mag.x != 0 || mag.y != 0 || mag.z != 0) {
          float heading = compute_heading(mag.x, mag.y);
          hal::print<64>(console, "Heading: %5.1f degrees\n", heading);
        }
      }
      catch (const std::exception& e) {
        hal::print<64>(console, "Error: %s\n", e.what());
      }
      
      hal::delay(clock, 1s);
    }
  }
  catch (const hal::no_such_device& e) {
    hal::print(console, "Device not found or invalid WHO_AM_I value\n");
    hal::print(console, "Check connections and address settings\n");
  }
  catch (const std::exception& e) {
    hal::print<64>(console, "Error initializing ICM20948: %s\n", e.what());
  }
  
  // Keep the application alive if we reached here
  hal::print(console, "Demo halted due to errors. System will idle.\n");
  
  while (true) {
    hal::delay(clock, 1s);
  }
}