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

#include <array>
#include <cmath>

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>

namespace hal::sensor {

namespace {
static constexpr hal::byte icm20948_address = 0x69;
static constexpr hal::byte ak09916_address = 0x0C;

/* Registers ICM20948 USER BANK 0 */
constexpr hal::byte who_am_i = 0x00;
constexpr hal::byte pwr_mgmt_1 = 0x06;
constexpr hal::byte pwr_mgmt_2 = 0x07;
constexpr hal::byte int_pin_cfg = 0x0F;
constexpr hal::byte accel_out = 0x2D;  // accel data registers begin
constexpr hal::byte gyro_out = 0x33;   // gyro data registers begin
constexpr hal::byte temp_out = 0x39;

/* Registers ICM20948 USER BANK 2 */
constexpr hal::byte gyro_smplrt_div = 0x00;
constexpr hal::byte gyro_config_1 = 0x01;
constexpr hal::byte gyro_config_2 = 0x02;
constexpr hal::byte odr_align_en = 0x09;
constexpr hal::byte accel_smplrt_div_1 = 0x10;
constexpr hal::byte accel_config = 0x14;
constexpr hal::byte accel_config_2 = 0x15;
constexpr hal::byte temp_config = 0x53;

/* Registers ICM20948 USER BANK 3 */
constexpr hal::byte i2c_slv0_addr = 0x03;
constexpr hal::byte i2c_slv0_reg = 0x04;
constexpr hal::byte i2c_slv0_ctrl = 0x05;
constexpr hal::byte i2c_slv0_do = 0x06;

/* Registers ICM20948 ALL BANKS */
constexpr hal::byte reg_bank_sel = 0x7F;

/* Registers AK09916 */
constexpr hal::byte ak09916_wia_1 = 0x00;  // Who I am, Company ID
constexpr hal::byte ak09916_wia_2 = 0x01;  // Who I am, Device ID
constexpr hal::byte ak09916_status_1 = 0x10;
constexpr hal::byte ak09916_hxl = 0x11;
[[maybe_unused]] constexpr hal::byte ak09916_hxh = 0x12;
[[maybe_unused]] constexpr hal::byte ak09916_hyl = 0x13;
[[maybe_unused]] constexpr hal::byte ak09916_hyh = 0x14;
[[maybe_unused]] constexpr hal::byte ak09916_hzl = 0x15;
[[maybe_unused]] constexpr hal::byte ak09916_hzh = 0x16;
constexpr hal::byte ak09916_status_2 = 0x18;
constexpr hal::byte ak09916_cntl_2 = 0x31;
constexpr hal::byte ak09916_cntl_3 = 0x32;

/* Register Bits */
constexpr hal::byte icm_reset = 0x41;
constexpr hal::byte icm_sleep = 0x40;
constexpr hal::byte bypass_en = 0x02;
constexpr hal::byte gyro_en = 0x07;
constexpr hal::byte acc_en = 0x38;
constexpr hal::byte ak09916_read = 0x80;

constexpr hal::byte who_am_i_content = 0xEA;
constexpr auto t_sensitivity = 333.87f;
}  // namespace

using namespace std::literals;

icm20948::icm20948(hal::i2c& p_i2c)
  : m_i2c(&p_i2c)
{
  m_current_bank = 0;
  reset_icm20948();
  reset_mag();

  // Check device ID
  if (auto id = whoami(); id != who_am_i_content) {
    hal::safe_throw(hal::no_such_device(id, this));
  }

  // Initialize offset and scaling factors
  m_acc_offset_val.x = 0.0;
  m_acc_offset_val.y = 0.0;
  m_acc_offset_val.z = 0.0;
  m_acc_corr_factor.x = 1.0;
  m_acc_corr_factor.y = 1.0;
  m_acc_corr_factor.z = 1.0;
  m_acc_range_factor = 1.0;
  m_gyro_offset_val.x = 0.0;
  m_gyro_offset_val.y = 0.0;
  m_gyro_offset_val.z = 0.0;
  m_gyro_range_factor = 1.0;

  // Power up and configure core sensors
  sleep(false);
  set_clock_auto_select();
  enable_acc(true);
  enable_gyro(true);

  // Configure sensor ranges
  set_acc_range(acc_range_2g);    // Initialize with 2g range
  set_gyro_range(gyro_range_250); // Initialize with 250 dps range
  set_acc_dlpf(dlpf_6);          // Set low noise filter
  set_gyro_dlpf(dlpf_6);         // Set low noise filter

  // Enable ODR (Output Data Rate) alignment
  write_register8({ .bank = 2, .reg = odr_align_en, .val = 1 });  // aligns ODR

  // Initialize magnetometer in 20Hz continuous mode
  init_mag();
}

void icm20948::auto_offsets()
{
  set_gyro_dlpf(dlpf_6);           // lowest noise
  set_gyro_range(gyro_range_250);  // highest resolution
  set_acc_range(acc_range_2g);
  set_acc_dlpf(dlpf_6);
  set_temp_dlpf(dlpf_6);
}

void icm20948::set_acceleration_offsets(
  acceleration_offset_t const& acc_offsets)
{
  m_acc_offset_val.x = (acc_offsets.xmax + acc_offsets.xmin) * 0.5f;
  m_acc_offset_val.y = (acc_offsets.ymax + acc_offsets.ymin) * 0.5f;
  m_acc_offset_val.z = (acc_offsets.zmax + acc_offsets.zmin) * 0.5f;
  m_acc_corr_factor.x = (acc_offsets.xmax + abs(acc_offsets.xmin)) / 32768.0f;
  m_acc_corr_factor.y = (acc_offsets.ymax + abs(acc_offsets.ymin)) / 32768.0f;
  m_acc_corr_factor.z = (acc_offsets.zmax + abs(acc_offsets.zmin)) / 32768.0f;
}

void icm20948::set_gyro_offsets(gyro_offset_t const& gyr_offsets)
{
  m_gyro_offset_val.x = gyr_offsets.x_offset;
  m_gyro_offset_val.y = gyr_offsets.y_offset;
  m_gyro_offset_val.z = gyr_offsets.z_offset;
}

hal::byte icm20948::whoami()
{
  return read_register8({ .bank = 0, .reg = who_am_i });
}

void icm20948::enable_acc(bool p_en_acc)
{
  m_reg_val = read_register8({ .bank = 0, .reg = pwr_mgmt_2 });

  if (p_en_acc) {
    m_reg_val &= ~acc_en;
  } else {
    m_reg_val |= acc_en;
  }

  write_register8({ .bank = 0, .reg = pwr_mgmt_2, .val = m_reg_val });
}

void icm20948::set_acc_range(acc_range p_acc_range)
{
  m_reg_val = read_register8({ .bank = 2, .reg = accel_config });
  m_reg_val &= ~(0x06);
  m_reg_val |= (p_acc_range << 1);
  write_register8({ .bank = 2, .reg = accel_config, .val = m_reg_val });

  // Update the acc_range_factor based on selected range
  switch(p_acc_range) {
    case acc_range_2g:
      m_acc_range_factor = 1;
      break;
    case acc_range_4g:
      m_acc_range_factor = 2;
      break;
    case acc_range_8g:
      m_acc_range_factor = 4;
      break;
    case acc_range_16g:
      m_acc_range_factor = 8;
      break;
  }
}

void icm20948::set_acc_dlpf(digital_lowpass_filter p_dlpf)
{
  m_reg_val = read_register8({ .bank = 2, .reg = accel_config });

  if (p_dlpf == dlpf_off) {
    m_reg_val &= 0xFE;
    write_register8({ .bank = 2, .reg = accel_config, .val = m_reg_val });
    return;
  } else {
    m_reg_val |= 0x01;
    m_reg_val &= 0xC7;
    m_reg_val |= (p_dlpf << 3);
  }
  write_register8({ .bank = 2, .reg = accel_config, .val = m_reg_val });
}


void icm20948::enable_gyro(bool p_enable_gyro)
{
  m_reg_val = read_register8({ .bank = 0, .reg = pwr_mgmt_2 });
  if (p_enable_gyro) {
    m_reg_val &= ~gyro_en;
  } else {
    m_reg_val |= gyro_en;
  }
  write_register8({ .bank = 0, .reg = pwr_mgmt_2, .val = m_reg_val });
}

void icm20948::set_gyro_range(gyro_range p_gyro_range)
{
  m_reg_val = read_register8({ .bank = 2, .reg = gyro_config_1 });
  m_reg_val &= ~(0x06);
  m_reg_val |= (static_cast<hal::byte>(p_gyro_range) << 1);
  write_register8({ .bank = 2, .reg = gyro_config_1, .val = m_reg_val });

  // Update the gyro_range_factor based on selected range
  switch(p_gyro_range) {
    case gyro_range_250:
      m_gyro_range_factor = 1;
      break;
    case gyro_range_500:
      m_gyro_range_factor = 2;
      break;
    case gyro_range_1000:
      m_gyro_range_factor = 4;
      break;
    case gyro_range_2000:
      m_gyro_range_factor = 8;
      break;
  }
}

void icm20948::set_gyro_dlpf(digital_lowpass_filter p_dlpf)
{
  m_reg_val = read_register8({ .bank = 2, .reg = gyro_config_1 });

  if (p_dlpf == dlpf_off) {
    m_reg_val &= 0xFE;
    write_register8({ .bank = 2, .reg = gyro_config_1, .val = m_reg_val });
    return;
  } else {
    m_reg_val |= 0x01;
    m_reg_val &= 0xC7;
    m_reg_val |= (p_dlpf << 3);
  }
  write_register8({ .bank = 2, .reg = gyro_config_1, .val = m_reg_val });
}


void icm20948::set_temp_dlpf(digital_lowpass_filter p_dlpf)
{
  write_register8({ .bank = 2, .reg = temp_config, .val = p_dlpf });
}

/************** Read Functions **************/

icm20948::accel_read_t icm20948::read_acceleration()
{
  accel_read_t accel_read = { 0, 0, 0 }, accel_read_raw{};
  switch_bank(0);
  auto data = hal::write_then_read<6>(*m_i2c,
                                      icm20948_address,
                                      std::to_array({ accel_out }),
                                      hal::never_timeout());

  accel_read_raw.x = static_cast<int16_t>((data[0] << 8) | data[1]);
  accel_read_raw.y = static_cast<int16_t>((data[2] << 8) | data[3]);
  accel_read_raw.z = static_cast<int16_t>((data[4] << 8) | data[5]);

  auto const acc_range_factor = static_cast<float>(m_acc_range_factor);
  constexpr auto max_range = 16384.0f;

  accel_read.x = (accel_read_raw.x * acc_range_factor) / max_range;
  accel_read.y = (accel_read_raw.y * acc_range_factor) / max_range;
  accel_read.z = (accel_read_raw.z * acc_range_factor) / max_range;

  accel_read.x = (accel_read.x - (m_acc_offset_val.x / acc_range_factor)) /
                 m_acc_corr_factor.x;
  accel_read.y = (accel_read.y - (m_acc_offset_val.y / acc_range_factor)) /
                 m_acc_corr_factor.y;
  accel_read.z = (accel_read.z - (m_acc_offset_val.z / acc_range_factor)) /
                 m_acc_corr_factor.z;

  return accel_read;
}

icm20948::gyro_read_t icm20948::read_gyroscope()
{
  gyro_read_t gyro_read = { 0, 0, 0 }, gyro_read_raw{};

  switch_bank(0);

  auto data = hal::write_then_read<6>(*m_i2c,
                                      icm20948_address,
                                      std::array<hal::byte, 1>{ gyro_out },
                                      hal::never_timeout());

  gyro_read_raw.x = static_cast<int16_t>((data[0] << 8) | data[1]);
  gyro_read_raw.y = static_cast<int16_t>((data[2] << 8) | data[3]);
  gyro_read_raw.z = static_cast<int16_t>((data[4] << 8) | data[5]);

  auto const gyro_range_factor = static_cast<float>(m_gyro_range_factor);
  constexpr auto max_resolution = 32768.0f;
  constexpr auto scale = 250.0f;

  gyro_read.x =
    ((gyro_read_raw.x * gyro_range_factor) * scale) / max_resolution;
  gyro_read.y =
    ((gyro_read_raw.y * gyro_range_factor) * scale) / max_resolution;
  gyro_read.z =
    ((gyro_read_raw.z * gyro_range_factor) * scale) / max_resolution;

  gyro_read.x -= (m_gyro_offset_val.x / gyro_range_factor);
  gyro_read.y -= (m_gyro_offset_val.y / gyro_range_factor);
  gyro_read.z -= (m_gyro_offset_val.z / gyro_range_factor);

  return gyro_read;
}

icm20948::mag_read_t icm20948::read_magnetometer()
{
  constexpr int max_polling_attempts = 1000;

  mag_read_t mag_read{};
  int polling_attempts = 0;

  while (true) {
    auto status =
      hal::write_then_read<1>(*m_i2c,
                              ak09916_address,
                              std::array<hal::byte, 1>{ ak09916_status_1 },
                              hal::never_timeout());

    if (status[0] & 0x01) {  // Check if data ready bit is set
      break;
    }

    if (++polling_attempts > max_polling_attempts) {
      hal::safe_throw(hal::timed_out(this));
    }
  }

  // Read Mag Data
  auto const data =
    hal::write_then_read<6>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_hxl },
                            hal::never_timeout());

  mag_read.x = static_cast<int16_t>((data[1] << 8) | data[0]);
  mag_read.y = static_cast<int16_t>((data[3] << 8) | data[2]);
  mag_read.z = static_cast<int16_t>((data[5] << 8) | data[4]);

  // Check magnetometer status silently

  return mag_read;
}

icm20948::temp_read_t icm20948::read_temperature()
{
  temp_read_t temp_read;

  switch_bank(0);
  auto const data =
    hal::write_then_read<2>(*m_i2c,
                            icm20948_address,
                            std::array<hal::byte, 1>{ temp_out },
                            hal::never_timeout());

  auto const raw_temp = static_cast<int16_t>((data[0] << 8) | data[1]);
  auto const float_temp = static_cast<float>(raw_temp);
  temp_read.temp = (float_temp / t_sensitivity) + 21.0f;
  return temp_read;
}

/********* Power, Sleep, Standby *********/



void icm20948::sleep(bool p_sleep)
{
  if (p_sleep) {
    m_reg_val |= icm_sleep;
  } else {
    m_reg_val &= ~icm_sleep;
  }
  write_register8({ .bank = 0, .reg = pwr_mgmt_1, .val = m_reg_val });
}

/************** Magnetometer **************/

void icm20948::init_mag()
{
  // Enable bypass mode to directly communicate with the magnetometer
  enable_bypass_mode();
  // Set magnetometer to continuous mode at 20Hz
  set_mag_op_mode(ak09916_cont_mode_20hz);
}

void icm20948::set_mag_op_mode(ak09916_op_mode p_op_mode)
{
  // Set magnetometer operation mode directly
  hal::write(*m_i2c,
             ak09916_address,
             std::array<hal::byte, 2>{ ak09916_cntl_2, static_cast<hal::byte>(p_op_mode) },
             hal::never_timeout());

  if (p_op_mode != ak09916_pwr_down) {
    enable_mag_data_read(ak09916_hxl, 0x08);
  }
}


void icm20948::reset_mag()
{
  enable_bypass_mode();

  hal::write(*m_i2c,
             ak09916_address,
             std::array<hal::byte, 2>{ ak09916_cntl_3, 0x01 },  // Soft Reset
             hal::never_timeout());
}


// Private implementation functions

hal::byte icm20948::mag_status1()
{
  auto const status =
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_status_1 },
                            hal::never_timeout());

  return status[0];
}

hal::byte icm20948::mag_status2()
{
  auto const status =
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_status_2 },
                            hal::never_timeout());

  return status[0];
}

hal::byte icm20948::whoami_ak09916_wia1_direct()
{
  auto const result =
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_wia_1 },
                            hal::never_timeout());
  return result[0];
}

hal::byte icm20948::whoami_ak09916_wia2_direct()
{
  auto const result =
    hal::write_then_read<1>(*m_i2c,
                            ak09916_address,
                            std::array<hal::byte, 1>{ ak09916_wia_2 },
                            hal::never_timeout());
  return result[0];
}

/************************************************
     Private Functions
*************************************************/

void icm20948::set_clock_auto_select()
{
  m_reg_val = read_register8({ .bank = 0, .reg = pwr_mgmt_1 });
  m_reg_val |= 0x01;
  write_register8({ .bank = 0, .reg = pwr_mgmt_1, .val = m_reg_val });
}

void icm20948::switch_bank(hal::byte p_new_bank)
{
  if (p_new_bank != m_current_bank) {
    m_current_bank = p_new_bank;
    hal::byte bank_value = m_current_bank << 4;

    // Write to bank select register with correct values
    hal::write(*m_i2c,
               icm20948_address,
               std::array<hal::byte, 2>{ reg_bank_sel, bank_value },
               hal::never_timeout());
  }
}

void icm20948::write_register8(write8_param p_param)
{
  switch_bank(p_param.bank);
  hal::write(*m_i2c,
             icm20948_address,
             std::array<hal::byte, 2>{ p_param.reg, p_param.val },
             hal::never_timeout());
}

void icm20948::write_register16(write16_param p_param)
{
  switch_bank(p_param.bank);
  auto msb = static_cast<hal::byte>((p_param.val >> 8) & 0xFF);
  hal::byte lsb = p_param.val & 0xFF;

  hal::write(*m_i2c,
             icm20948_address,
             std::array<hal::byte, 3>{ p_param.reg, msb, lsb },
             hal::never_timeout());
}

hal::byte icm20948::read_register8(read_param p_param)
{
  switch_bank(p_param.bank);
  auto ctrl_buffer =
    hal::write_then_read<1>(*m_i2c,
                            icm20948_address,
                            std::array<hal::byte, 1>{ p_param.reg },
                            hal::never_timeout());
  return ctrl_buffer[0];
}

std::uint16_t icm20948::read_register16(read_param p_param)
{
  switch_bank(p_param.bank);

  // Read both bytes at once since they should be consecutive registers
  auto data = hal::write_then_read<2>(*m_i2c,
                                     icm20948_address,
                                     std::array<hal::byte, 1>{ p_param.reg },
                                     hal::never_timeout());

  std::uint16_t reg_16_value = (data[0] << 8) | data[1];
  return reg_16_value;
}

void icm20948::reset_icm20948()
{
  write_register8({ .bank = 0, .reg = pwr_mgmt_1, .val = icm_reset });
}

void icm20948::enable_bypass_mode()
{
  write_register8({ .bank = 0, .reg = int_pin_cfg, .val = bypass_en });
}

void icm20948::enable_mag_data_read(hal::byte p_reg,   // NOLINT
                                    hal::byte p_bytes  // NOLINT
)
{
  // read AK09916
  write_register8(
    { .bank = 3, .reg = i2c_slv0_addr, .val = ak09916_address | ak09916_read });
  // define AK09916 register to be read
  write_register8({ .bank = 3, .reg = i2c_slv0_reg, .val = p_reg });
  // enable read | number of byte
  hal::byte const enable_and_bytes = 0x80 | p_bytes;
  write_register8({ .bank = 3, .reg = i2c_slv0_ctrl, .val = enable_and_bytes });
}
}  // namespace hal::sensor
