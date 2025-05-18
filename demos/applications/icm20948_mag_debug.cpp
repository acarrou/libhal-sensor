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
#include <array>

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

// Constants for magnetometer access
constexpr hal::byte AK09916_ADDR = 0x0C;        // Magnetometer I2C address
constexpr hal::byte AK09916_WIA1 = 0x00;        // Company ID register
constexpr hal::byte AK09916_WIA2 = 0x01;        // Device ID register
constexpr hal::byte AK09916_STATUS_1 = 0x10;    // Status 1 register
constexpr hal::byte AK09916_HXL = 0x11;         // X-axis data registers start
constexpr hal::byte AK09916_STATUS_2 = 0x18;    // Status 2 register
constexpr hal::byte AK09916_CNTL2 = 0x31;       // Control 2 register
constexpr hal::byte AK09916_CNTL3 = 0x32;       // Control 3 (reset) register

constexpr hal::byte ICM20948_ADDR = 0x69;       // Main IMU I2C address
constexpr hal::byte ICM_REG_BANK_SEL = 0x7F;    // Bank select register
constexpr hal::byte ICM_INT_PIN_CFG = 0x0F;     // INT_PIN_CFG register in bank 0
constexpr hal::byte ICM_BYPASS_EN = 0x02;       // I2C bypass enable bit

// Simple delay function
void busy_wait(hal::steady_clock& clock, std::chrono::milliseconds delay_time)
{
  hal::delay(clock, delay_time);
}

// Switch to a specific bank
void select_bank(hal::i2c& i2c, hal::byte bank) 
{
  // Bank value is in bits [4:5]
  hal::byte bank_value = (bank & 0x03) << 4;
  
  hal::write(i2c, 
            ICM20948_ADDR, 
            std::array<hal::byte, 2>{ICM_REG_BANK_SEL, bank_value}, 
            hal::never_timeout());
}

// Enable I2C bypass mode to directly access magnetometer
void enable_bypass_mode(hal::i2c& i2c, hal::serial& console)
{
  // Make sure we're in bank 0
  select_bank(i2c, 0);
  
  hal::print(console, "Reading INT_PIN_CFG register...\n");
  
  // Read current INT_PIN_CFG value
  auto result = hal::write_then_read<1>(i2c, 
                                      ICM20948_ADDR, 
                                      std::array<hal::byte, 1>{ICM_INT_PIN_CFG}, 
                                      hal::never_timeout());
  
  hal::print<64>(console, "INT_PIN_CFG = 0x%02X\n", result[0]);
  
  // Set bypass enable bit
  hal::byte new_value = result[0] | ICM_BYPASS_EN;
  
  hal::print<64>(console, "Writing INT_PIN_CFG = 0x%02X to enable bypass\n", new_value);
  
  // Write updated value
  hal::write(i2c, 
            ICM20948_ADDR, 
            std::array<hal::byte, 2>{ICM_INT_PIN_CFG, new_value}, 
            hal::never_timeout());
}

// Reset magnetometer
void reset_magnetometer(hal::i2c& i2c, hal::serial& console, hal::steady_clock& clock)
{
  hal::print(console, "Resetting magnetometer...\n");
  
  try {
    // Write 0x01 to CNTL3 register to reset
    hal::write(i2c, 
              AK09916_ADDR, 
              std::array<hal::byte, 2>{AK09916_CNTL3, 0x01}, 
              hal::never_timeout());
    
    hal::print(console, "Reset command sent. Waiting 100ms...\n");
  } 
  catch (const std::exception& e) {
    hal::print<64>(console, "Error sending reset: %s\n", e.what());
  }
  
  // Wait for reset to complete
  busy_wait(clock, std::chrono::milliseconds(100));
}

// Set magnetometer mode
void set_magnetometer_mode(hal::i2c& i2c, hal::serial& console, hal::byte mode)
{
  hal::print<64>(console, "Setting magnetometer mode to 0x%02X...\n", mode);
  
  try {
    // Write mode to CNTL2 register
    hal::write(i2c, 
              AK09916_ADDR, 
              std::array<hal::byte, 2>{AK09916_CNTL2, mode}, 
              hal::never_timeout());
    
    hal::print(console, "Mode set successfully\n");
  } 
  catch (const std::exception& e) {
    hal::print<64>(console, "Error setting mode: %s\n", e.what());
  }
}

// Read magnetometer data
void read_magnetometer_data(hal::i2c& i2c, hal::serial& console)
{
  hal::print(console, "Reading magnetometer status and data...\n");
  
  try {
    // Read STATUS1 register
    auto status1 = hal::write_then_read<1>(i2c, 
                                         AK09916_ADDR, 
                                         std::array<hal::byte, 1>{AK09916_STATUS_1}, 
                                         hal::never_timeout());
    
    hal::print<64>(console, "STATUS1 = 0x%02X (data ready: %s)\n", 
                 status1[0], (status1[0] & 0x01) ? "YES" : "NO");
    
    if (!(status1[0] & 0x01)) {
      hal::print(console, "Data not ready, no readings to report\n");
      return;
    }
    
    // Read all 6 bytes of magnetometer data
    auto data = hal::write_then_read<6>(i2c, 
                                      AK09916_ADDR, 
                                      std::array<hal::byte, 1>{AK09916_HXL}, 
                                      hal::never_timeout());
    
    // Read STATUS2 register to check for overflow
    auto status2 = hal::write_then_read<1>(i2c, 
                                         AK09916_ADDR, 
                                         std::array<hal::byte, 1>{AK09916_STATUS_2}, 
                                         hal::never_timeout());
    
    hal::print<64>(console, "STATUS2 = 0x%02X (overflow: %s)\n", 
                 status2[0], (status2[0] & 0x08) ? "YES" : "NO");
    
    // Convert readings - magnetometer is little-endian
    int16_t mag_x = static_cast<int16_t>((data[1] << 8) | data[0]);
    int16_t mag_y = static_cast<int16_t>((data[3] << 8) | data[2]);
    int16_t mag_z = static_cast<int16_t>((data[5] << 8) | data[4]);
    
    hal::print<64>(console, "Raw readings: X=%d, Y=%d, Z=%d\n", 
                  mag_x, mag_y, mag_z);
    
    // Convert to microtesla (0.15 uT per LSB)
    constexpr float mag_scale = 0.15f;
    float mag_x_ut = mag_x * mag_scale;
    float mag_y_ut = mag_y * mag_scale;
    float mag_z_ut = mag_z * mag_scale;
    
    hal::print<64>(console, "Scaled (uT): X=%6.2f, Y=%6.2f, Z=%6.2f\n", 
                  mag_x_ut, mag_y_ut, mag_z_ut);
    
    // Calculate heading if data is valid
    if (mag_x != 0 || mag_y != 0) {
      float heading = atan2(mag_y_ut, mag_x_ut) * (180.0f / std::numbers::pi);
      if (heading < 0) {
        heading += 360.0f;
      }
      
      hal::print<64>(console, "Heading: %6.2f degrees\n", heading);
    } else {
      hal::print(console, "Cannot calculate heading (no valid data)\n");
    }
  } 
  catch (const std::exception& e) {
    hal::print<64>(console, "Error reading magnetometer: %s\n", e.what());
  }
}

// Check magnetometer identity
void check_magnetometer_id(hal::i2c& i2c, hal::serial& console)
{
  hal::print(console, "Checking magnetometer identity...\n");
  
  try {
    // Read WIA1 and WIA2 registers
    auto wia1 = hal::write_then_read<1>(i2c, 
                                       AK09916_ADDR, 
                                       std::array<hal::byte, 1>{AK09916_WIA1}, 
                                       hal::never_timeout());
    
    auto wia2 = hal::write_then_read<1>(i2c, 
                                       AK09916_ADDR, 
                                       std::array<hal::byte, 1>{AK09916_WIA2}, 
                                       hal::never_timeout());
    
    hal::print<64>(console, "WIA1 = 0x%02X (expected 0x48)\n", wia1[0]);
    hal::print<64>(console, "WIA2 = 0x%02X (expected 0x09)\n", wia2[0]);
    
    if (wia1[0] == 0x48 && wia2[0] == 0x09) {
      hal::print(console, "Magnetometer identity verified successfully!\n");
    } else {
      hal::print(console, "WARNING: Unexpected magnetometer identity!\n");
    }
  } 
  catch (const std::exception& e) {
    hal::print<64>(console, "Error checking magnetometer ID: %s\n", e.what());
  }
}

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "ICM20948 Magnetometer Debug Application\n\n");
  busy_wait(clock, 500ms);
  
  // Scan for ICM20948 and magnetometer
  hal::print(console, "Step 1: Scanning I2C bus for devices...\n");
  bool found_icm = false;
  bool found_mag = false;
  
  for (hal::byte address = 0x08; address < 0x78; address++) {
    if (hal::probe(i2c, address)) {
      hal::print<64>(console, "  Found device at 0x%02X\n", address);
      if (address == ICM20948_ADDR) {
        found_icm = true;
      }
      if (address == AK09916_ADDR) {
        found_mag = true;
      }
    }
  }
  
  if (!found_icm) {
    hal::print(console, "WARNING: ICM20948 not found at address 0x69\n");
  }
  
  if (found_mag) {
    hal::print(console, "Magnetometer found at 0x0C during initial scan.\n");
    hal::print(console, "This suggests bypass mode is already enabled.\n");
  } else {
    hal::print(console, "Magnetometer not found at 0x0C during initial scan.\n");
    hal::print(console, "This is normal - need to enable bypass mode first.\n");
  }
  
  // Step 2: Enable bypass mode
  hal::print(console, "\nStep 2: Enabling I2C bypass mode...\n");
  try {
    enable_bypass_mode(i2c, console);
    busy_wait(clock, 100ms);
    
    // Check if magnetometer is now visible
    hal::print(console, "Checking if magnetometer is now accessible...\n");
    if (hal::probe(i2c, AK09916_ADDR)) {
      hal::print(console, "Success! Magnetometer is now accessible at 0x0C\n");
      found_mag = true;
    } else {
      hal::print(console, "ERROR: Cannot access magnetometer at 0x0C\n");
      found_mag = false;
    }
  } 
  catch (const std::exception& e) {
    hal::print<64>(console, "Error enabling bypass mode: %s\n", e.what());
  }
  
  if (!found_mag) {
    hal::print(console, "Cannot proceed without magnetometer access\n");
    while (true) {
      busy_wait(clock, 1s);
    }
  }
  
  // Step 3: Check magnetometer identity
  hal::print(console, "\nStep 3: Checking magnetometer identity...\n");
  check_magnetometer_id(i2c, console);
  
  // Step 4: Reset the magnetometer
  hal::print(console, "\nStep 4: Resetting magnetometer...\n");
  reset_magnetometer(i2c, console, clock);
  busy_wait(clock, 100ms);
  
  // Step 5: Set magnetometer to continuous measurement mode
  hal::print(console, "\nStep 5: Setting magnetometer to continuous mode (100Hz)...\n");
  set_magnetometer_mode(i2c, console, 0x08);  // 0x08 = 100Hz continuous measurement
  busy_wait(clock, 100ms);
  
  // Step 6: Monitor data and status in a loop
  hal::print(console, "\nStep 6: Starting magnetometer data monitoring...\n");
  
  int loop_count = 0;
  while (true) {
    hal::print<64>(console, "\n--- Magnetometer Reading #%d ---\n", ++loop_count);
    read_magnetometer_data(i2c, console);
    
    // If we've been running a while, try resetting and reinitializing
    if (loop_count % 10 == 0) {
      hal::print(console, "\nPerforming periodic magnetometer reset and reinit...\n");
      reset_magnetometer(i2c, console, clock);
      busy_wait(clock, 100ms);
      set_magnetometer_mode(i2c, console, 0x08);  // 100Hz continuous
      busy_wait(clock, 100ms);
    }
    
    busy_wait(clock, 1s);
  }
}