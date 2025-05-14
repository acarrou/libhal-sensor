#include <array>

#include <libhal-sensor/multi/mpl3115a2.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>

using namespace std::literals;
namespace hal::sensor {
namespace {

// Default 7-bit I2C device address for MPL3115A2
constexpr hal::byte device_address = 0x60;

// Helper functions for creating arrays containing the different register
// addresses in their payloads.
constexpr auto whoami_register()
{
  return std::to_array<hal::byte>({ 0x0C });
}

constexpr auto status_register()
{
  return std::to_array<hal::byte>({ 0x00 });
}

constexpr auto out_p_msb_register()
{
  return std::to_array<hal::byte>({ 0x01 });
}

constexpr auto out_t_msb_register()
{
  return std::to_array<hal::byte>({ 0x04 });
}

constexpr auto pt_data_cfg_register()
{
  return std::to_array<hal::byte>({ 0x13 });
}

constexpr auto bar_in_msb_register()
{
  return std::to_array<hal::byte>({ 0x14 });
}

constexpr auto ctrl_reg1_register()
{
  return std::to_array<hal::byte>({ 0x26 });
}

constexpr auto off_h_register()
{
  return std::to_array<hal::byte>({ 0x2D });
}

// MPL3115A2 Status Register Bits
constexpr hal::byte status_tdr = 0x02;  // Temperature new data ready
constexpr hal::byte status_pdr = 0x04;  // Pressure/Altitude new data ready

// MPL3115A2 PT DATA Register Bits
constexpr hal::byte pt_data_cfg_tdefe = 0x01;  // Data event flag enable for temperature
constexpr hal::byte pt_data_cfg_pdefe = 0x02;  // Data event flag enable for pressure/altitude
constexpr hal::byte pt_data_cfg_drem = 0x04;   // Data ready event mode

// MPL3115A2 Control Register Bits
constexpr hal::byte ctrl_reg1_rst = 0x04;   // Reset bit
constexpr hal::byte ctrl_reg1_ost = 0x02;   // One-Shot trigger bit
constexpr hal::byte ctrl_reg1_alt = 0x80;   // Altimeter-Barometer mode bit
constexpr hal::byte ctrl_reg1_os128 = 0x38; // Oversampling ratio 2^128

/**
 * @brief Set the sensor operating mode (barometer or altimeter)
 * 
 * @param p_i2c - I2C driver pointer
 * @param p_mode - Desired mode to set
 */
void set_mode(hal::i2c* p_i2c, mpl3115a2::mode p_mode)
{
  // Read current control register value
  auto ctrl_value = hal::write_then_read<1>(*p_i2c,
                                           device_address,
                                           ctrl_reg1_register(),
                                           hal::never_timeout())[0];

  // Check if mode change is needed
  if ((ctrl_value & ctrl_reg1_alt) != static_cast<int>(p_mode)) {
    // Set mode bit appropriately
    if (p_mode == mpl3115a2::mode::barometer) {
      ctrl_value &= ~ctrl_reg1_alt;  // Clear the bit for barometer mode
    } else {
      ctrl_value |= ctrl_reg1_alt;   // Set the bit for altimeter mode
    }

    // Write updated value back to register
    std::array<hal::byte, 2> ctrl_buffer = { ctrl_reg1_register()[0], ctrl_value };
    hal::write(*p_i2c, device_address, ctrl_buffer, hal::never_timeout());
  }
}

/**
 * @brief Set bits in a register without overwriting existing register state
 * 
 * @param p_i2c - I2C driver reference
 * @param p_reg_addr - Register address
 * @param p_bits_to_set - Bits to set in the register
 */
void modify_reg_bits(hal::i2c& p_i2c, hal::byte p_reg_addr, hal::byte p_bits_to_set)
{
  // Read current register value
  auto reg_value = hal::write_then_read<1>(p_i2c,
                                          device_address,
                                          std::array<hal::byte, 1>{ p_reg_addr },
                                          hal::never_timeout())[0];

  // Set specified bits while preserving existing values
  hal::byte updated_value = reg_value | p_bits_to_set;
  
  // Write updated value back to register
  std::array<hal::byte, 2> reg_buffer = { p_reg_addr, updated_value };
  hal::write(p_i2c, device_address, reg_buffer, hal::never_timeout());
}

/**
 * @brief Wait for reset bit to clear after a device reset
 * 
 * @param p_i2c - I2C driver pointer
 */
void poll_reset(hal::i2c* p_i2c)
{
  bool reset_active = true;
  uint16_t retries = 0;

  while (reset_active && (retries < mpl3115a2::default_max_polling_retries)) {
    try {
      // This may throw because the device is resetting
      auto ctrl_value = hal::write_then_read<1>(*p_i2c,
                                               device_address,
                                               ctrl_reg1_register(),
                                               hal::never_timeout())[0];
      
      // Check if reset bit is still set
      reset_active = ((ctrl_value & ctrl_reg1_rst) != 0);
    } catch (hal::no_such_device const&) {
      // Expected during reset, retry
      retries++;
    }
  }
}

/**
 * @brief Poll for a specific register flag to reach desired state
 * 
 * @param p_i2c - I2C driver reference
 * @param p_reg_addr - Register address to check
 * @param p_flag - Flag bit(s) to check
 * @param p_desired_state - Target state (true = flag set, false = flag clear)
 */
void poll_flag(hal::i2c& p_i2c, hal::byte p_reg_addr, hal::byte p_flag, bool p_desired_state)
{
  uint16_t retries = 0;
  bool flag_check = !p_desired_state;

  while (flag_check && (retries < mpl3115a2::default_max_polling_retries)) {
    auto reg_value = hal::write_then_read<1>(p_i2c,
                                            device_address,
                                            std::array<hal::byte, 1>{ p_reg_addr },
                                            hal::never_timeout())[0];
    
    // Check if flag matches desired state
    if (p_desired_state) {
      flag_check = ((reg_value & p_flag) == 0);
    } else {
      flag_check = ((reg_value & p_flag) != 0);
    }
    retries++;
  }
}

/**
 * @brief Trigger a one-shot measurement
 * 
 * @param p_i2c - I2C driver reference
 */
void initiate_one_shot(hal::i2c& p_i2c)
{
  // Wait for any previous one-shot to complete
  poll_flag(p_i2c, ctrl_reg1_register()[0], ctrl_reg1_ost, false);

  // Set one-shot bit to initiate new measurement
  modify_reg_bits(p_i2c, ctrl_reg1_register()[0], ctrl_reg1_ost);
}

}  // anonymous namespace

mpl3115a2::mpl3115a2(hal::i2c& p_i2c)
  : m_i2c(&p_i2c)
  , m_sensor_mode(mode::altimeter)
{
  // Verify connection by checking device ID
  static constexpr hal::byte expected_device_id = 0xC4;
  auto device_id = hal::write_then_read<1>(p_i2c,
                                          device_address,
                                          whoami_register(),
                                          hal::never_timeout())[0];

  if (device_id != expected_device_id) {
    hal::safe_throw(hal::no_such_device(device_id, this));
  }

  // Perform software reset
  modify_reg_bits(p_i2c, ctrl_reg1_register()[0], ctrl_reg1_rst);

  // Wait for reset to complete
  poll_reset(&p_i2c);

  // Set oversampling ratio to 2^128 and set altitude mode
  modify_reg_bits(p_i2c, ctrl_reg1_register()[0], ctrl_reg1_os128 | ctrl_reg1_alt);

  // Enable data ready events for pressure/altitude and temperature
  std::array<hal::byte, 2> pt_cfg_buffer = {
    pt_data_cfg_register()[0],
    pt_data_cfg_tdefe | pt_data_cfg_pdefe | pt_data_cfg_drem
  };
  hal::write(p_i2c, device_address, pt_cfg_buffer, hal::never_timeout());
}

mpl3115a2::temperature_results mpl3115a2::read_temperature()
{
  constexpr float temp_conversion_factor = 256.0f;

  // Trigger a new measurement
  initiate_one_shot(*m_i2c);

  // Wait for temperature data to be ready
  poll_flag(*m_i2c, status_register()[0], status_tdr, true);

  // Read temperature data from registers
  auto temp_buffer = hal::write_then_read<2>(*m_i2c,
                                            device_address,
                                            out_t_msb_register(),
                                            hal::never_timeout());

  // Convert raw data to temperature
  auto temp_raw = (temp_buffer[0] << 8) | temp_buffer[1];
  auto temp_celsius = static_cast<float>(temp_raw) / temp_conversion_factor;

  return temperature_results{ temp_celsius };
}

mpl3115a2::pressure_results mpl3115a2::read_pressure()
{
  constexpr float pressure_conversion_factor = 64.0f;

  // Switch to barometer mode if needed
  if (m_sensor_mode != mode::barometer) {
    set_mode(m_i2c, mode::barometer);
    m_sensor_mode = mode::barometer;
  }

  // Trigger a new measurement
  initiate_one_shot(*m_i2c);

  // Wait for pressure data to be ready
  poll_flag(*m_i2c, status_register()[0], status_pdr, true);

  // Read pressure data from registers
  auto pressure_buffer = hal::write_then_read<3>(*m_i2c,
                                               device_address,
                                               out_p_msb_register(),
                                               hal::never_timeout());

  // Convert raw data to pressure
  uint32_t pressure_raw = uint32_t(pressure_buffer[0]) << 16 |
                          uint32_t(pressure_buffer[1]) << 8 |
                          uint32_t(pressure_buffer[2]);
  
  float pressure = static_cast<float>(pressure_raw) / pressure_conversion_factor;

  return pressure_results{ pressure };
}

mpl3115a2::altitude_results mpl3115a2::read_altitude()
{
  constexpr float altitude_conversion_factor = 65536.0f;

  // Switch to altimeter mode if needed
  if (m_sensor_mode != mode::altimeter) {
    set_mode(m_i2c, mode::altimeter);
    m_sensor_mode = mode::altimeter;
  }

  // Trigger a new measurement
  initiate_one_shot(*m_i2c);

  // Wait for altitude data to be ready
  poll_flag(*m_i2c, status_register()[0], status_pdr, true);

  // Read altitude data from registers
  auto altitude_buffer = hal::write_then_read<3>(*m_i2c,
                                               device_address,
                                               out_p_msb_register(),
                                               hal::never_timeout());

  // Convert raw data to altitude
  int32_t altitude_raw = int32_t(altitude_buffer[0]) << 24 |
                         int32_t(altitude_buffer[1]) << 16 |
                         int32_t(altitude_buffer[2]) << 8;
  
  float altitude = static_cast<float>(altitude_raw) / altitude_conversion_factor;

  return altitude_results{ altitude };
}

void mpl3115a2::set_sea_pressure(float p_sea_level_pressure)
{
  // Convert to 2Pa per LSB
  auto two_pa = static_cast<std::uint16_t>(p_sea_level_pressure / 2.0f);
  auto two_pa_hi = static_cast<hal::byte>((two_pa & 0xFF00) >> 8);
  auto two_pa_lo = static_cast<hal::byte>(two_pa & 0x00FF);

  // Write value to registers
  std::array<hal::byte, 3> slp_buffer = {
    bar_in_msb_register()[0],
    two_pa_hi,
    two_pa_lo
  };

  hal::write(*m_i2c, device_address, slp_buffer, hal::never_timeout());
}

void mpl3115a2::set_altitude_offset(int8_t p_offset)
{
  // Write offset to register
  std::array<hal::byte, 2> offset_buffer = {
    off_h_register()[0],
    static_cast<hal::byte>(p_offset)
  };

  hal::write(*m_i2c, device_address, offset_buffer, hal::never_timeout());
}

}  // namespace hal::sensor