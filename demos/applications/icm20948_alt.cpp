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

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

// Address can be either 0x68 or 0x69 depending on AD0 pin
// This version tries to communicate directly without the driver first

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "ICM20948 Debug Application Starting...\n\n");
  hal::delay(clock, 200ms);
  
  // First scan for I2C devices to verify the bus is working
  hal::print(console, "Scanning I2C bus for devices...\n");
  
  for (hal::byte address = 0x08; address < 0x78; address++) {
    if (hal::probe(i2c, address)) {
      hal::print<32>(console, "  Found device at 0x%02X\n", address);
    }
  }
  
  // Try to read WHO_AM_I register from both possible addresses
  constexpr std::array<hal::byte, 2> possible_addresses = { 0x68, 0x69 };
  constexpr hal::byte who_am_i_reg = 0x00;  // WHO_AM_I register in bank 0
  constexpr hal::byte bank_select_reg = 0x7F;
  constexpr hal::byte expected_id = 0xEA;  // Expected value for ICM20948
  
  hal::print(console, "\nTesting direct communication with ICM20948...\n");
  
  for (auto address : possible_addresses) {
    hal::print<32>(console, "Trying address 0x%02X:\n", address);
    
    // First select bank 0 to ensure we're in the right state
    try {
      // Set bank 0 (bank = 0 << 4 = 0x00)
      hal::write(i2c, address, std::array<hal::byte, 2>{bank_select_reg, 0x00}, hal::never_timeout());
      hal::print(console, "  - Bank select write succeeded\n");
      
      // Try to read WHO_AM_I register
      auto result = hal::write_then_read<1>(
        i2c, address, std::array<hal::byte, 1>{who_am_i_reg}, hal::never_timeout());
      
      hal::print<64>(console, "  - WHO_AM_I = 0x%02X (expected 0x%02X)\n", 
                     result[0], expected_id);
                     
      if (result[0] == expected_id) {
        hal::print(console, "  - Found valid ICM20948 at this address!\n");
      } else {
        hal::print(console, "  - Invalid device ID\n");
      }
    }
    catch (const std::exception& e) {
      hal::print<64>(console, "  - Communication error: %s\n", e.what());
    }
    
    hal::delay(clock, 100ms);
  }
  
  hal::print(console, "\nI2C diagnostics complete.\n");
  
  // Keep the application alive
  while (true) {
    hal::delay(clock, 1s);
  }
}