#ifndef MS5637_H
#define MS5637_H
 
#include "mbed.h"

#define MS5637_COEFFICIENT_COUNT 7

#define MS5637_CONVERSION_TIME_OSR_256 1
#define MS5637_CONVERSION_TIME_OSR_512 2
#define MS5637_CONVERSION_TIME_OSR_1024 3
#define MS5637_CONVERSION_TIME_OSR_2048 5
#define MS5637_CONVERSION_TIME_OSR_4096 9
#define MS5637_CONVERSION_TIME_OSR_8192 17

// Enum
enum MS5637_resolution_osr 
{
  MS5637_resolution_osr_256 = 0,
  MS5637_resolution_osr_512,
  MS5637_resolution_osr_1024,
  MS5637_resolution_osr_2048,
  MS5637_resolution_osr_4096
};

enum MS5637_status 
{
  MS5637_status_ok,
  MS5637_status_no_i2c_acknowledge,
  MS5637_status_i2c_transfer_error,
  MS5637_status_crc_error
};

enum MS5637_status_code 
{
  MS5637_STATUS_OK = 0,
  MS5637_STATUS_ERR_OVERFLOW = 1,
  MS5637_STATUS_ERR_TIMEOUT = 4
};

// Functions
class MS5637 
{

public:
  MS5637(PinName sda, PinName scl);

  /**
  * \brief Reset the MS5637 device
  *
  * \return MS5637_status : status of MS5637
  *       - MS5637_status_ok : I2C transfer completed successfully
  *       - MS5637_status_i2c_transfer_error : Problem with i2c transfer
  *       - MS5637_status_no_i2c_acknowledge : I2C did not acknowledge
  */
  enum MS5637_status reset(void);

  /**
  * \brief Set  ADC resolution.
  *
  * \param[in] MS5637_resolution_osr : Resolution requested
  *
  */
  void set_resolution(enum MS5637_resolution_osr res);

  /**
  * \brief Reads the temperature and pressure ADC value and compute the
  * compensated values.
  *
  * \param[out] float* : Celsius Degree temperature value
  * \param[out] float* : mbar pressure value
  *
  * \return MS5637_status : status of MS5637
  *       - MS5637_status_ok : I2C transfer completed successfully
  *       - MS5637_status_i2c_transfer_error : Problem with i2c transfer
  *       - MS5637_status_no_i2c_acknowledge : I2C did not acknowledge
  *       - MS5637_status_crc_error : CRC check error on on the PROM
  * coefficients
  */
  enum MS5637_status read_temperature_and_pressure(float *temperature,
                                                   float *pressure);

private:
  I2C* i2c_;
  enum MS5637_status write_command(uint8_t cmd);
  enum MS5637_status read_eeprom_coeff(uint8_t command, uint16_t *coeff);
  bool crc_check(uint16_t *n_prom, uint8_t crc);
  enum MS5637_status conversion_and_read_adc(uint8_t cmd, uint32_t *adc);
  enum MS5637_status read_eeprom(void);

  uint16_t eeprom_coeff[MS5637_COEFFICIENT_COUNT + 1];
  bool coeff_read;
  enum MS5637_status MS5637_write_command(uint8_t);
  enum MS5637_status MS5637_read_eeprom_coeff(uint8_t, uint16_t *);
  enum MS5637_status MS5637_read_eeprom(void);
  enum MS5637_status MS5637_conversion_and_read_adc(uint8_t, uint32_t *);

  enum MS5637_resolution_osr MS5637_resolution_osr;
  uint32_t* conversion_time;
};

#endif