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

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "ICM20948 Simple Demo Starting...\n\n");
  hal::delay(clock, 500ms);
  
  // Since we know the device is at address 0x69 from scan
  constexpr hal::byte icm20948_address = 0x69;
  constexpr hal::byte who_am_i_reg = 0x00;      // WHO_AM_I register in bank 0
  constexpr hal::byte bank_select_reg = 0x7F;   // Bank select register
  constexpr hal::byte pwr_mgmt_1_reg = 0x06;    // Power management register 1
  constexpr hal::byte expected_id = 0xEA;       // Expected WHO_AM_I value

  hal::print(console, "Step 1: Verifying device at address 0x69\n");
  
  if (!hal::probe(i2c, icm20948_address)) {
    hal::print(console, "ERROR: Device at 0x69 not responding!\n");
    while (true) { hal::delay(clock, 1s); }
  }
  
  hal::print(console, "Device found at 0x69\n");
  hal::delay(clock, 100ms);

  hal::print(console, "Step 2: Testing basic register access\n");
  
  try {
    // First select bank 0
    hal::write(i2c, 
              icm20948_address,
              std::array<hal::byte, 2>{bank_select_reg, 0x00}, 
              hal::never_timeout());
              
    hal::print(console, "Bank selection successful\n");
    hal::delay(clock, 100ms);
    
    // Read WHO_AM_I register
    auto result = hal::write_then_read<1>(
      i2c, 
      icm20948_address, 
      std::array<hal::byte, 1>{who_am_i_reg}, 
      hal::never_timeout());
    
    hal::print<64>(console, "WHO_AM_I = 0x%02X (expected 0x%02X)\n", 
                   result[0], expected_id);
                   
    if (result[0] != expected_id) {
      hal::print(console, "ERROR: Unexpected WHO_AM_I value!\n");
      while (true) { hal::delay(clock, 1s); }
    }
    
    hal::print(console, "WHO_AM_I verification successful\n");
    hal::delay(clock, 100ms);
    
    // Try resetting the device
    hal::print(console, "Step 3: Performing device reset\n");
    
    // Reset device (bit 7 of PWR_MGMT_1)
    hal::write(i2c, 
              icm20948_address,
              std::array<hal::byte, 2>{pwr_mgmt_1_reg, 0x80}, 
              hal::never_timeout());
              
    hal::print(console, "Reset command sent\n");
    // Wait for reset to complete
    hal::delay(clock, 100ms);
    
    // Clear sleep bit (bit 6 of PWR_MGMT_1)
    hal::write(i2c, 
              icm20948_address,
              std::array<hal::byte, 2>{pwr_mgmt_1_reg, 0x01}, 
              hal::never_timeout());
              
    hal::print(console, "Sleep mode disabled\n");
    hal::delay(clock, 100ms);
    
    // Step 4: Now try using the driver
    hal::print(console, "Step 4: Initializing driver\n");
    
    hal::sensor::icm20948 icm_device(i2c);
    
    hal::print(console, "Driver initialized successfully!\n");
    hal::delay(clock, 100ms);
    
    // Read acceleration
    hal::print(console, "Step 5: Reading acceleration\n");
    auto accel = icm_device.read_acceleration();
    hal::print<64>(console, "Accel (g): x=%f, y=%f, z=%f\n", 
                  accel.x, accel.y, accel.z);
    hal::delay(clock, 100ms);
    
    // Read gyroscope
    hal::print(console, "Step 6: Reading gyroscope\n");
    auto gyro = icm_device.read_gyroscope();
    hal::print<64>(console, "Gyro (deg/s): x=%f, y=%f, z=%f\n", 
                  gyro.x, gyro.y, gyro.z);
    hal::delay(clock, 100ms);
    
    // Read temperature
    hal::print(console, "Step 7: Reading temperature\n");
    auto temp = icm_device.read_temperature();
    hal::print<64>(console, "Temperature: %f °C\n", temp.temp);
    hal::delay(clock, 100ms);
    
    // Read magnetometer
    hal::print(console, "Step 8: Reading magnetometer\n");
    auto mag = icm_device.read_magnetometer();
    hal::print<64>(console, "Mag: x=%f, y=%f, z=%f\n", 
                  mag.x, mag.y, mag.z);
    
    hal::print(console, "All sensor readings successful!\n");
    
    // Main loop - just read values periodically
    hal::print(console, "\nEntering main loop...\n");
    while (true) {
      try {
        accel = icm_device.read_acceleration();
        gyro = icm_device.read_gyroscope();
        temp = icm_device.read_temperature();
        mag = icm_device.read_magnetometer();
        
        hal::print<64>(console, "A[%0.2f,%0.2f,%0.2f] G[%0.2f,%0.2f,%0.2f] T[%0.1f] M[%0.2f,%0.2f,%0.2f]\n",
                      accel.x, accel.y, accel.z,
                      gyro.x, gyro.y, gyro.z,
                      temp.temp,
                      mag.x, mag.y, mag.z);
      }
      catch (const std::exception& e) {
        hal::print<64>(console, "Error: %s\n", e.what());
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