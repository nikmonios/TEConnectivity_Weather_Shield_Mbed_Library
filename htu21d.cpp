#include "mbed.h"
#include "htu21d.h"

// HTU21 device address
#define HTU21_ADDR 0x40 // 0b1000000
// HTU21 device commands
#define HTU21_RESET_COMMAND 0xFE
#define HTU21_READ_TEMPERATURE_W_HOLD_COMMAND 0xE3
#define HTU21_READ_TEMPERATURE_WO_HOLD_COMMAND 0xF3
#define HTU21_READ_HUMIDITY_W_HOLD_COMMAND 0xE5
#define HTU21_READ_HUMIDITY_WO_HOLD_COMMAND 0xF5
#define HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND_MSB 0xFA
#define HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND_LSB 0x0F
#define HTU21_READ_SERIAL_LAST_6BYTES_COMMAND_MSB 0xFC
#define HTU21_READ_SERIAL_LAST_6BYTES_COMMAND_LSB 0xC9
#define HTU21_WRITE_USER_REG_COMMAND 0xE6
#define HTU21_READ_USER_REG_COMMAND 0xE7

#define RESET_TIME 15 // ms value
#define TEMPERATURE_TIME 50
#define HUMIDITY_TIME 50
// Processing constants
#define HTU21_TEMPERATURE_COEFFICIENT (float)(-0.15)
#define HTU21_CONSTANT_A (float)(8.1332)
#define HTU21_CONSTANT_B (float)(1762.39)
#define HTU21_CONSTANT_C (float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL (175.72)
#define TEMPERATURE_COEFF_ADD (-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL (125)
#define HUMIDITY_COEFF_ADD (-6)

// Conversion timings
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b 50
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_13b_RH_10b 25
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_12b_RH_8b 13
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_11b_RH_11b 7
#define HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b 16
#define HTU21_HUMIDITY_CONVERSION_TIME_T_13b_RH_10b 5
#define HTU21_HUMIDITY_CONVERSION_TIME_T_12b_RH_8b 3
#define HTU21_HUMIDITY_CONVERSION_TIME_T_11b_RH_11b 8

// HTU21 User Register masks and bit position
#define HTU21_USER_REG_RESOLUTION_MASK 0x81
#define HTU21_USER_REG_END_OF_BATTERY_MASK 0x40
#define HTU21_USER_REG_ENABLE_ONCHIP_HEATER_MASK 0x4
#define HTU21_USER_REG_DISABLE_OTP_RELOAD_MASK 0x2
#define HTU21_USER_REG_RESERVED_MASK                                           \
  (~(HTU21_USER_REG_RESOLUTION_MASK | HTU21_USER_REG_END_OF_BATTERY_MASK |     \
     HTU21_USER_REG_ENABLE_ONCHIP_HEATER_MASK |                                \
     HTU21_USER_REG_DISABLE_OTP_RELOAD_MASK))

// HTU User Register values
// Resolution
#define HTU21_USER_REG_RESOLUTION_T_14b_RH_12b 0x00
#define HTU21_USER_REG_RESOLUTION_T_13b_RH_10b 0x80
#define HTU21_USER_REG_RESOLUTION_T_12b_RH_8b 0x01
#define HTU21_USER_REG_RESOLUTION_T_11b_RH_11b 0x81

// End of battery status
#define HTU21_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V 0x00
#define HTU21_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V 0x40
// Enable on chip heater
#define HTU21_USER_REG_ONCHIP_HEATER_ENABLE 0x04
#define HTU21_USER_REG_OTP_RELOAD_DISABLE 0x02

uint32_t HTU21_temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
uint32_t HTU21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;
enum HTU21_i2c_master_mode i2c_master_mode;
 
/**
* \brief Class constructor
*
*/
HTU21D::HTU21D(PinName sda, PinName scl) 
{
  i2c_master_mode = HTU21_i2c_no_hold;
  i2c_ = new I2C(sda, scl);
  //400KHz, as specified by the datasheet.
  i2c_->frequency(400000);
}
 

/**
 * \brief Set I2C master mode.
 *        This determines whether the program will hold while ADC is accessed or
 * will wait some time
 *
 * \param[in] HTU21_i2c_master_mode : I2C mode
 *
 */
void HTU21D::set_i2c_master_mode(enum HTU21_i2c_master_mode mode)
 {
  i2c_master_mode = mode;
}

/**
 * \brief Reads the HTU21 user register.
 *
 * \param[out] uint8_t* : Storage of user register value
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::read_user_register(uint8_t *value) 
{
  char buffer[1];
  char tx[1];
  
  tx[0] = HTU21_READ_USER_REG_COMMAND;

  // Read DATA
  i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 1);

  i2c_->read((HTU21_ADDR << 1) | 0x01, buffer, 1);

  *value = buffer[0];
  
  return HTU21_status_ok;
}

/**
 * \brief Writes the HTU21 user register with value
 *        Will read and keep the unreserved bits of the register
 *
 * \param[in] uint8_t : Register value to be set.
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::write_user_register(uint8_t value)
{
  enum HTU21_status status;
  uint8_t reg;
  char tx[2];

  status = read_user_register(&reg);
  if (status != HTU21_status_ok)
    return status;

  // Clear bits of reg that are not reserved
  reg &= HTU21_USER_REG_RESERVED_MASK;
  // Set bits from value that are not reserved
  reg |= (value & ~HTU21_USER_REG_RESERVED_MASK);
  
  tx[0] = HTU21_WRITE_USER_REG_COMMAND;
  tx[1] = reg;
  
  i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 2);

  return HTU21_status_ok;
}

/**
 * \brief Get heater status
 *
 * \param[in] HTU21_heater_status* : Return heater status (above or below 2.5V)
 *                      - HTU21_heater_off,
 *                      - HTU21_heater_on
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::get_heater_status(enum HTU21_heater_status *heater) 
{
  enum HTU21_status status;
  uint8_t reg_value;

  status = read_user_register(&reg_value);
  if (status != HTU21_status_ok)
    return status;

  // Get the heater enable bit in reg_value
  if (reg_value & HTU21_USER_REG_ONCHIP_HEATER_ENABLE)
    *heater = HTU21_heater_on;
  else
    *heater = HTU21_heater_off;

  return status;
}

/**
 * \brief Enable heater
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::enable_heater(void) 
{
  enum HTU21_status status;
  uint8_t reg_value;

  status = read_user_register(&reg_value);
  if (status != HTU21_status_ok)
    return status;

  // Clear the resolution bits
  reg_value |= HTU21_USER_REG_ONCHIP_HEATER_ENABLE;

  status = write_user_register(reg_value);

  return status;
}

/**
 * \brief Disable heater
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::disable_heater(void) 
{
  enum HTU21_status status;
  uint8_t reg_value;

  status = read_user_register(&reg_value);
  if (status != HTU21_status_ok)
    return status;

  // Clear the resolution bits
  reg_value &= ~HTU21_USER_REG_ONCHIP_HEATER_ENABLE;

  status = write_user_register(reg_value);

  return status;
}

/**
 * \brief Provide battery status
 *
 * \param[out] HTU21_battery_status* : Battery status
 *                      - HTU21_battery_ok,
 *                      - HTU21_battery_low
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::get_battery_status(enum HTU21_battery_status *bat) 
{
  enum HTU21_status status;
  uint8_t reg_value;

  status = read_user_register(&reg_value);
  if (status != HTU21_status_ok)
    return status;

  if (reg_value & HTU21_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V)
    *bat = HTU21_battery_low;
  else
    *bat = HTU21_battery_ok;

  return status;
}

/**
 * \brief Writes the HTU21 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::write_command(uint8_t cmd) 
{
  char tx[1];
  
  tx[0] = cmd;
  
  i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 1);

  return HTU21_status_ok;
}

/**
 * \brief Reset the HTU21 device
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum HTU21_status HTU21D::reset(void) 
{
  enum HTU21_status status;

  status = write_command(HTU21_RESET_COMMAND);
  if (status != HTU21_status_ok)
    return status;

  HTU21_temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
  HTU21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;

  wait_ms(RESET_TIME);

  return HTU21_status_ok;
}

/**
 * \brief Check CRC
 *
 * \param[in] uint16_t : variable on which to check CRC
 * \param[in] uint8_t : CRC value
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : CRC check is OK
 *       - HTU21_status_crc_error : CRC check error
 */
enum HTU21_status HTU21D::crc_check(uint16_t value, uint8_t crc) 
{
  uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
  uint32_t msb = 0x800000;
  uint32_t mask = 0xFF8000;
  uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

  while (msb != 0x80) 
  {

    // Check if msb of current value is 1 and apply XOR mask
    if (result & msb)
      result = ((result ^ polynom) & mask) | (result & ~mask);

    // Shift by one
    msb >>= 1;
    mask >>= 1;
    polynom >>= 1;
  }
  if (result == crc)
    return HTU21_status_ok;
  else
    return HTU21_status_crc_error;
}

/**
 * \brief Reads the temperature ADC value
 *
 * \param[out] uint16_t* : Temperature ADC value.
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - HTU21_status_crc_error : CRC check error
 */
enum HTU21_status HTU21D::temperature_conversion_and_read_adc(uint16_t *adc) 
{
  enum HTU21_status status = HTU21_status_ok;

  uint8_t i2c_status;
  uint16_t _adc;
  char buffer[3];
  char tx[1];

  /* Command */
  if (i2c_master_mode == HTU21_i2c_hold) 
  {
    tx[0] = HTU21_READ_TEMPERATURE_W_HOLD_COMMAND;
    i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 1);
  } 
  else 
  {
    tx[0] = HTU21_READ_TEMPERATURE_WO_HOLD_COMMAND;
    i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 1);
    wait_ms((uint8_t)TEMPERATURE_TIME);
  }

  /* Read data */
  i2c_->read((HTU21_ADDR << 1) | 0x01, buffer, 3);

  _adc = (buffer[0] << 8) | buffer[1];

  // compute CRC
  status = crc_check(_adc, buffer[2]);
  if (status != HTU21_status_ok)
    return status;

  *adc = _adc;

  return status;
}

/**
 * \brief Reads the relative humidity ADC value
 *
 * \param[out] uint16_t* : Relative humidity ADC value.
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - HTU21_status_crc_error : CRC check error
 */
enum HTU21_status HTU21D::humidity_conversion_and_read_adc(uint16_t *adc) 
{
  enum HTU21_status status = HTU21_status_ok;
  enum HTU21_i2c_master_mode mode;
  uint8_t i2c_status;
  uint16_t _adc;
  char buffer[3];
  char tx[1];

  /* Read data */
  if (i2c_master_mode == HTU21_i2c_hold) 
  {
    tx[0] = HTU21_READ_HUMIDITY_W_HOLD_COMMAND;
    i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 1);
  } 
  else 
  {
    tx[0] = HTU21_READ_HUMIDITY_WO_HOLD_COMMAND;
    i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 1);
    wait_ms((uint8_t)HUMIDITY_TIME);
  }
  
  i2c_->read((HTU21_ADDR << 1) | 0x01, buffer, 3);

  _adc = (buffer[0] << 8) | buffer[1];
  // compute CRC
  if (status != HTU21_status_ok)
    return status;
  *adc = _adc;

  return status;
}

/**
 * \brief Reads the relative humidity value.
 *
 * \param[out] float* : Celsius Degree temperature value
 * \param[out] float* : %RH Relative Humidity value
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - HTU21_status_crc_error : CRC check error
 */
enum HTU21_status HTU21D::read_temperature_and_relative_humidity(float *temperature, float *humidity) 
{
  enum HTU21_status status;
  uint16_t adc;

  status = temperature_conversion_and_read_adc(&adc);
  if (status != HTU21_status_ok)
    return status;

  // Perform conversion function
  *temperature = (float)adc * TEMPERATURE_COEFF_MUL / (1UL << 16) + TEMPERATURE_COEFF_ADD;

  status = humidity_conversion_and_read_adc(&adc);
  if (status != HTU21_status_ok)
    return status;

  // Perform conversion function
  *humidity = (float)adc * HUMIDITY_COEFF_MUL / (1UL << 16) + HUMIDITY_COEFF_ADD;

  return status;
}

/**
 * \brief Set temperature & humidity ADC resolution.
 *
 * \param[in] HTU21_resolution : Resolution requested
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - HTU21_status_crc_error : CRC check error
 */
enum HTU21_status HTU21D::set_resolution(enum HTU21_resolution res) 
{
  enum HTU21_status status;
  uint8_t reg_value, tmp = 0;
  uint32_t temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
  uint32_t humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;

  if (res == HTU21_resolution_t_14b_rh_12b) 
  {
    tmp = HTU21_USER_REG_RESOLUTION_T_14b_RH_12b;
    temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
    humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;
  } 
  else if (res == HTU21_resolution_t_13b_rh_10b) 
  {
    tmp = HTU21_USER_REG_RESOLUTION_T_13b_RH_10b;
    temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_13b_RH_10b;
    humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_13b_RH_10b;
  } 
  else if (res == HTU21_resolution_t_12b_rh_8b) 
  {
    tmp = HTU21_USER_REG_RESOLUTION_T_12b_RH_8b;
    temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_12b_RH_8b;
    humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_12b_RH_8b;
  } 
  else if (res == HTU21_resolution_t_11b_rh_11b) 
  {
    tmp = HTU21_USER_REG_RESOLUTION_T_11b_RH_11b;
    temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_11b_RH_11b;
    humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_11b_RH_11b;
  }

  status = read_user_register(&reg_value);
  if (status != HTU21_status_ok)
    return status;

  // Clear the resolution bits
  reg_value &= ~HTU21_USER_REG_RESOLUTION_MASK;
  reg_value |= tmp & HTU21_USER_REG_RESOLUTION_MASK;

  HTU21_temperature_conversion_time = temperature_conversion_time;
  HTU21_humidity_conversion_time = humidity_conversion_time;

  status = write_user_register(reg_value);

  return status;
}

/**
 * \brief Returns result of compensated humidity
 *
 * \param[in] float - Actual temperature measured (degC)
 * \param[in] float - Actual relative humidity measured (%RH)
 *
 * \return float - Compensated humidity (%RH).
 */
float HTU21D::compute_compensated_humidity(float temperature, float relative_humidity) 
{
  return (relative_humidity + (25 - temperature) * HTU21_TEMPERATURE_COEFFICIENT);
}

/**
 * \brief Returns the computed dew point
 *
 * \param[in] float - Actual temperature measured (degC)
 * \param[in] float - Actual relative humidity measured (%RH)
 *
 * \return float - Dew point temperature (DegC).
 */
float HTU21D::compute_dew_point(float temperature, float relative_humidity) 
{
  double partial_pressure;
  double dew_point;

  // Missing power of 10
  partial_pressure = pow(10, HTU21_CONSTANT_A - HTU21_CONSTANT_B / (temperature + HTU21_CONSTANT_C));

  dew_point = -HTU21_CONSTANT_B / (log10(relative_humidity * partial_pressure / 100) - HTU21_CONSTANT_A) - HTU21_CONSTANT_C;

  return (float)dew_point;
}

/**
 * \brief Reads the HTU21 serial number.
 *
 * \param[out] uint64_t* : Serial number
 *
 * \return HTU21_status : status of HTU21
 *       - HTU21_status_ok : I2C transfer completed successfully
 *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
 *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - HTU21_status_crc_error : CRC check error
 */
enum HTU21_status HTU21D::read_serial_number(uint64_t *serial_number) 
{
  enum HTU21_status status;
  enum HTU21_status_code i2c_status;
  char rcv_data[14];
  char first_data[8];
  char second_data[6];
  char tx[2];

  // Read the first 8 bytes
  tx[0] = HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND_MSB;
  tx[1] = HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND_LSB;
  
  i2c_->write((HTU21_ADDR << 1) & 0xFE, tx, 2);
  
  i2c_->read((HTU21_ADDR << 1) | 0x01, first_data, 8);

  // Read the last 6 bytes
  tx[0] = HTU21_READ_SERIAL_LAST_6BYTES_COMMAND_MSB;
  tx[1] = HTU21_READ_SERIAL_LAST_6BYTES_COMMAND_LSB;
  
  i2c_->write((0x40 << 1) & 0xFE, tx, 2);

  i2c_->read((HTU21_ADDR << 1) | 0x01, second_data, 6);

  for(uint16_t i = 0; i < 8; i++)
  {
      rcv_data[i] = first_data[i];
  }
  for(uint16_t i = 0; i < 6; i++)
  {
      rcv_data[i+8] = second_data[i];
  }

  *serial_number =
      ((uint64_t)rcv_data[0] << 56) | ((uint64_t)rcv_data[2] << 48) |
      ((uint64_t)rcv_data[4] << 40) | ((uint64_t)rcv_data[6] << 32) |
      ((uint64_t)rcv_data[8] << 24) | ((uint64_t)rcv_data[9] << 16) |
      ((uint64_t)rcv_data[12] << 8) | ((uint64_t)rcv_data[11] << 0);

  return HTU21_status_ok;
}