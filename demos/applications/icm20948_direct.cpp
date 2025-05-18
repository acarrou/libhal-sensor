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
#include <array>
#include <numbers>

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

// ICM20948 Constants
constexpr hal::byte ICM_ADDR = 0x69;

// Bank select register (always accessible)
constexpr hal::byte REG_BANK_SEL = 0x7F;

// Bank 0 Registers
constexpr hal::byte WHO_AM_I = 0x00;     // Should return 0xEA
constexpr hal::byte PWR_MGMT_1 = 0x06;   // Power management 1
constexpr hal::byte PWR_MGMT_2 = 0x07;   // Power management 2
constexpr hal::byte INT_PIN_CFG = 0x0F;  // Interrupt config
constexpr hal::byte ACCEL_OUT = 0x2D;    // Accel data start (6 bytes)
constexpr hal::byte GYRO_OUT = 0x33;     // Gyro data start (6 bytes)
constexpr hal::byte TEMP_OUT = 0x39;     // Temp data start (2 bytes)

// Bank 2 Registers
constexpr hal::byte GYRO_CONFIG_1 = 0x01;  // Gyro config 1
constexpr hal::byte ACCEL_CONFIG = 0x14;   // Accel config

// Magnetometer (AK09916) Constants and Registers
constexpr hal::byte MAG_ADDR = 0x0C;
constexpr hal::byte MAG_WIA1 = 0x00;        // Who I am 1
constexpr hal::byte MAG_WIA2 = 0x01;        // Who I am 2
constexpr hal::byte MAG_STATUS_1 = 0x10;    // Status 1
constexpr hal::byte MAG_HXL = 0x11;         // X-axis data (6 bytes total)
constexpr hal::byte MAG_STATUS_2 = 0x18;    // Status 2
constexpr hal::byte MAG_CNTL2 = 0x31;       // Control 2
constexpr hal::byte MAG_CNTL3 = 0x32;       // Control 3

// Control values
constexpr hal::byte ICM_RESET = 0x80;   // Reset bit for PWR_MGMT_1
constexpr hal::byte BYPASS_EN = 0x02;   // Enable I2C bypass

// Expected ID values
constexpr hal::byte ICM_ID = 0xEA;
constexpr hal::byte MAG_ID1 = 0x48;
constexpr hal::byte MAG_ID2 = 0x09;

// Scaling factors
constexpr float ACCEL_SCALE_2G = 16384.0f;
constexpr float GYRO_SCALE_250DPS = 131.0f;
constexpr float TEMP_SCALE = 333.87f;
constexpr float TEMP_OFFSET = 21.0f;
constexpr float MAG_SCALE = 0.15f;  // uT per LSB

// Struct for sensor data
struct imu_data_t {
  float accel_x, accel_y, accel_z;  // Acceleration in g
  float gyro_x, gyro_y, gyro_z;     // Gyro in degrees/s
  float mag_x, mag_y, mag_z;        // Magnetometer in uT
  float temp;                       // Temperature in °C
  float heading;                    // Heading in degrees
};

// Calculate heading from magnetometer data
float compute_heading(float x, float y)
{
  float heading = atan2(y, x) * (180.0f / std::numbers::pi);
  if (heading < 0) {
    heading += 360.0f;
  }
  return heading;
}

// Switch to a specific bank
void select_bank(hal::i2c& i2c, hal::byte bank)
{
  // Bank value is in bits [4:5]
  hal::byte bank_value = (bank & 0x03) << 4;
  hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{REG_BANK_SEL, bank_value}, 
           hal::never_timeout());
}

// Read 16-bit value (big-endian)
int16_t read_int16(const std::array<hal::byte, 2>& data)
{
  return static_cast<int16_t>((data[0] << 8) | data[1]);
}

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "ICM20948 Direct Implementation Demo\n\n");
  hal::delay(clock, 200ms);
  
  // First scan I2C bus to verify device presence
  hal::print(console, "Scanning for I2C devices...\n");
  bool found = false;
  
  for (hal::byte address = 0x08; address < 0x78; address++) {
    if (hal::probe(i2c, address)) {
      hal::print<64>(console, "  Found device at 0x%02X\n", address);
      if (address == ICM_ADDR) {
        found = true;
      }
    }
  }
  
  if (!found) {
    hal::print(console, "ERROR: ICM20948 not found at expected address!\n");
    while (true) { hal::delay(clock, 1s); }
  }
  
  // Initialize the ICM20948 directly
  try {
    // Step 1: Reset the device
    hal::print(console, "Resetting ICM20948...\n");
    select_bank(i2c, 0);
    hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{PWR_MGMT_1, ICM_RESET}, 
             hal::never_timeout());
    
    // Wait for reset to complete
    hal::delay(clock, 100ms);
    
    // Step 2: Wake up the device
    hal::print(console, "Waking up device...\n");
    hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{PWR_MGMT_1, 0x01}, 
             hal::never_timeout());
    
    // Step 3: Check WHO_AM_I register
    hal::print(console, "Checking device ID...\n");
    auto result = hal::write_then_read<1>(i2c, ICM_ADDR, 
                                        std::array<hal::byte, 1>{WHO_AM_I}, 
                                        hal::never_timeout());
    
    hal::print<64>(console, "WHO_AM_I = 0x%02X (expected 0x%02X)\n", result[0], ICM_ID);
    if (result[0] != ICM_ID) {
      hal::print(console, "ERROR: Unexpected device ID!\n");
      while (true) { hal::delay(clock, 1s); }
    }
    
    // Step 4: Enable accelerometer and gyroscope
    hal::print(console, "Enabling accelerometer and gyroscope...\n");
    hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{PWR_MGMT_2, 0x00}, 
             hal::never_timeout());
    
    // Step 5: Configure accelerometer (±2g range)
    hal::print(console, "Configuring accelerometer...\n");
    select_bank(i2c, 2);
    hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{ACCEL_CONFIG, 0x00}, 
             hal::never_timeout());
    
    // Step 6: Configure gyroscope (±250 dps range)
    hal::print(console, "Configuring gyroscope...\n");
    hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{GYRO_CONFIG_1, 0x00}, 
             hal::never_timeout());
    
    // Step 7: Configure and enable magnetometer
    hal::print(console, "Configuring magnetometer...\n");
    select_bank(i2c, 0);
    
    // Enable I2C bypass to access magnetometer directly
    hal::write(i2c, ICM_ADDR, std::array<hal::byte, 2>{INT_PIN_CFG, BYPASS_EN}, 
             hal::never_timeout());
    
    // Reset magnetometer
    hal::write(i2c, MAG_ADDR, std::array<hal::byte, 2>{MAG_CNTL3, 0x01}, 
             hal::never_timeout());
    hal::delay(clock, 100ms);
    
    // Set magnetometer to continuous measurement mode, 100Hz
    hal::write(i2c, MAG_ADDR, std::array<hal::byte, 2>{MAG_CNTL2, 0x08}, 
             hal::never_timeout());
    
    // Check magnetometer ID
    auto mag_id1 = hal::write_then_read<1>(i2c, MAG_ADDR, 
                                         std::array<hal::byte, 1>{MAG_WIA1}, 
                                         hal::never_timeout());
    auto mag_id2 = hal::write_then_read<1>(i2c, MAG_ADDR, 
                                         std::array<hal::byte, 1>{MAG_WIA2}, 
                                         hal::never_timeout());
                                         
    hal::print<64>(console, "MAG ID = 0x%02X%02X (expected 0x%02X%02X)\n", 
                  mag_id1[0], mag_id2[0], MAG_ID1, MAG_ID2);
    
    hal::print(console, "Initialization complete!\n");
    hal::delay(clock, 200ms);
    
    // Main loop - read and display sensor data
    hal::print(console, "Starting main loop...\n\n");
    
    while (true) {
      try {
        // Make sure we're in bank 0 for sensor reading
        select_bank(i2c, 0);
        
        // Read accelerometer data (6 bytes)
        auto accel_data = hal::write_then_read<6>(i2c, ICM_ADDR, 
                                               std::array<hal::byte, 1>{ACCEL_OUT}, 
                                               hal::never_timeout());
        
        // Read gyroscope data (6 bytes)
        auto gyro_data = hal::write_then_read<6>(i2c, ICM_ADDR, 
                                              std::array<hal::byte, 1>{GYRO_OUT}, 
                                              hal::never_timeout());
        
        // Read temperature data (2 bytes)
        auto temp_data = hal::write_then_read<2>(i2c, ICM_ADDR, 
                                              std::array<hal::byte, 1>{TEMP_OUT}, 
                                              hal::never_timeout());
        
        // Wait for magnetometer data ready
        hal::byte status1 = 0;
        int attempts = 0;
        while ((status1 & 0x01) == 0 && attempts < 10) {
          auto status = hal::write_then_read<1>(i2c, MAG_ADDR, 
                                             std::array<hal::byte, 1>{MAG_STATUS_1}, 
                                             hal::never_timeout());
          status1 = status[0];
          if ((status1 & 0x01) != 0) {
            break;
          }
          attempts++;
          hal::delay(clock, 1ms);
        }
        
        // Read magnetometer data (6 bytes)
        auto mag_data = hal::write_then_read<6>(i2c, MAG_ADDR, 
                                             std::array<hal::byte, 1>{MAG_HXL}, 
                                             hal::never_timeout());
        
        // Convert acceleration data (raw -> g)
        int16_t accel_x_raw = read_int16({accel_data[0], accel_data[1]});
        int16_t accel_y_raw = read_int16({accel_data[2], accel_data[3]});
        int16_t accel_z_raw = read_int16({accel_data[4], accel_data[5]});
        
        float accel_x = static_cast<float>(accel_x_raw) / ACCEL_SCALE_2G;
        float accel_y = static_cast<float>(accel_y_raw) / ACCEL_SCALE_2G;
        float accel_z = static_cast<float>(accel_z_raw) / ACCEL_SCALE_2G;
        
        // Convert gyroscope data (raw -> deg/s)
        int16_t gyro_x_raw = read_int16({gyro_data[0], gyro_data[1]});
        int16_t gyro_y_raw = read_int16({gyro_data[2], gyro_data[3]});
        int16_t gyro_z_raw = read_int16({gyro_data[4], gyro_data[5]});
        
        float gyro_x = static_cast<float>(gyro_x_raw) / GYRO_SCALE_250DPS;
        float gyro_y = static_cast<float>(gyro_y_raw) / GYRO_SCALE_250DPS;
        float gyro_z = static_cast<float>(gyro_z_raw) / GYRO_SCALE_250DPS;
        
        // Convert temperature data (raw -> °C)
        int16_t temp_raw = read_int16({temp_data[0], temp_data[1]});
        float temp = (static_cast<float>(temp_raw) / TEMP_SCALE) + TEMP_OFFSET;
        
        // Magnetometer data in the AK09916 has little-endian byte order
        int16_t mag_x_raw = static_cast<int16_t>((mag_data[1] << 8) | mag_data[0]);
        int16_t mag_y_raw = static_cast<int16_t>((mag_data[3] << 8) | mag_data[2]);
        int16_t mag_z_raw = static_cast<int16_t>((mag_data[5] << 8) | mag_data[4]);
        
        float mag_x = static_cast<float>(mag_x_raw) * MAG_SCALE;
        float mag_y = static_cast<float>(mag_y_raw) * MAG_SCALE;
        float mag_z = static_cast<float>(mag_z_raw) * MAG_SCALE;
        
        // Calculate heading
        float heading = compute_heading(mag_x, mag_y);
        
        // Display all data
        hal::print(console, "ICM20948 Sensor Readings:\n");
        
        hal::print<64>(console, "Accel (g):    X: %6.2f  Y: %6.2f  Z: %6.2f\n", 
                      accel_x, accel_y, accel_z);
                      
        hal::print<64>(console, "Gyro (deg/s): X: %6.2f  Y: %6.2f  Z: %6.2f\n", 
                      gyro_x, gyro_y, gyro_z);
                      
        hal::print<64>(console, "Mag (uT):     X: %6.2f  Y: %6.2f  Z: %6.2f\n", 
                      mag_x, mag_y, mag_z);
                      
        hal::print<64>(console, "Temperature:  %6.2f °C\n", temp);
        
        hal::print<64>(console, "Heading:      %6.2f degrees\n", heading);
        
        hal::print(console, "\n");
      }
      catch (const std::exception& e) {
        hal::print<64>(console, "Error reading sensor data: %s\n", e.what());
      }
      
      hal::delay(clock, 1s);
    }
  }
  catch (const std::exception& e) {
    hal::print<64>(console, "ERROR: %s\n", e.what());
  }
  
  // If we reach here, there was an error
  hal::print(console, "Demo halted due to errors.\n");
  
  while (true) {
    hal::delay(clock, 1s);
  }
}