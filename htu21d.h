#ifndef HTU21D_H
#define HTU21D_H

#include "mbed.h"
 
// Enum
enum HTU21_status 
{
  HTU21_status_ok,
  HTU21_status_i2c_transfer_error,
  HTU21_status_no_i2c_acknowledge,
  HTU21_status_crc_error
};

enum HTU21_i2c_master_mode { HTU21_i2c_no_hold, HTU21_i2c_hold };

enum HTU21_status_code 
{
  HTU21_STATUS_OK = 0,
  HTU21_STATUS_ERR_OVERFLOW = 1,
  HTU21_STATUS_ERR_TIMEOUT = 4
};

enum HTU21_heater_status { HTU21_heater_off, HTU21_heater_on };

enum HTU21_battery_status { HTU21_battery_ok, HTU21_battery_low };

enum HTU21_resolution 
{
  HTU21_resolution_t_14b_rh_12b = 0,
  HTU21_resolution_t_12b_rh_8b,
  HTU21_resolution_t_13b_rh_10b,
  HTU21_resolution_t_11b_rh_11b
};

// Functions
class HTU21D 
{

public:
  HTU21D(PinName sda, PinName scl);

  /**
   * \brief Get heater status
   *
   * \param[in] HTU21_heater_status* : Return heater status (above or below
   *2.5V)
   *                        - HTU21_heater_off,
   *                      - HTU21_heater_on
   *
   * \return HTU21_status : status of HTU21
   *       - HTU21_status_ok : I2C transfer completed successfully
   *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
   *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum HTU21_status get_heater_status(enum HTU21_heater_status *heater);

  /**
   * \brief Enable heater
   *
   * \return HTU21_status : status of HTU21
   *       - HTU21_status_ok : I2C transfer completed successfully
   *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
   *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum HTU21_status enable_heater(void);

  /**
   * \brief Disable heater
   *
   * \return HTU21_status : status of HTU21
   *       - HTU21_status_ok : I2C transfer completed successfully
   *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
   *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum HTU21_status disable_heater(void);

  /**
   * \brief Provide battery status
   *
   * \param[out] HTU21_battery_status* : Battery status
   *                      - HTU21_battery_ok,
   *                      - HTU21_battery_low
   *
   * \return status of HTU21
   *       - HTU21_status_ok : I2C transfer completed successfully
   *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
   *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum HTU21_status get_battery_status(enum HTU21_battery_status *);

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
  enum HTU21_status reset(void);

  /**
   * \brief Reads the relative humidity and temperature value.
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
  enum HTU21_status read_temperature_and_relative_humidity(float *temperature, float *humidity);

  /**
   * \brief Set temperature and humidity ADC resolution.
   *
   * \param[in] HTU21_resolution : Resolution requested
   *
   * \return HTU21_status : status of HTU21
   *       - HTU21_status_ok : I2C transfer completed successfully
   *       - HTU21_status_i2c_transfer_error : Problem with i2c transfer
   *       - HTU21_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - HTU21_status_crc_error : CRC check error
   */
  enum HTU21_status set_resolution(enum HTU21_resolution res);

  /**
   * \brief Returns result of compensated humidity
   *
   * \param[in] float - Actual temperature measured (degC)
   * \param[in] float - Actual relative humidity measured (%RH)
   *
   * \return float - Compensated humidity (%RH).
   */
  float compute_compensated_humidity(float temperature, float relative_humidity);

  /**
   * \brief Returns the computed dew point
   *
   * \param[in] float - Actual temperature measured (degC)
   * \param[in] float - Actual relative humidity measured (%RH)
   *
   * \return float - Dew point temperature (DegC).
   */
  float compute_dew_point(float temperature, float relative_humidity);

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
  enum HTU21_status read_serial_number(uint64_t *serial_number);

  /**
   * \brief Set I2C master mode.
   *        This determines whether the program will hold while ADC is accessed
   * or will wait some time
   *
   * \param[in] HTU21_i2c_master_mode : I2C mode
   *
   */
  void set_i2c_master_mode(enum HTU21_i2c_master_mode mode);

private:
  enum HTU21_status read_user_register(uint8_t *value);
  enum HTU21_status write_user_register(uint8_t value);
  enum HTU21_status write_command(uint8_t cmd);
  enum HTU21_status temperature_conversion_and_read_adc(uint16_t *adc);
  enum HTU21_status humidity_conversion_and_read_adc(uint16_t *adc);
  enum HTU21_status crc_check(uint16_t value, uint8_t crc);
  
  I2C* i2c_;
};
 
#endif /* HTU21D_H */