#ifndef TSYS01_H
#define TSYS01_H
 
#include "mbed.h"

#define TSYS01_ADDR_CSB_1 0x76 // 0b1110110
#define TSYS01_ADDR_CSB_0 0x77 // 0b1110111

#define PROM_ELEMENTS_NUMBER 8

#define COEFF_MUL_0 (float)(-1.5)
#define COEFF_MUL_1 (float)(1)
#define COEFF_MUL_2 (float)(-2)
#define COEFF_MUL_3 (float)(4)
#define COEFF_MUL_4 (float)(-2)

// enum
enum TSYS01_status_code 
{
  TSYS01_STATUS_OK = 0x00,
  TSYS01_STATUS_ERR_OVERFLOW = 0x01,
  TSYS01_STATUS_ERR_TIMEOUT = 0x02,
};
enum TSYS01_address { TSYS01_i2c_address_csb_1, TSYS01_i2c_address_csb_0 };

enum TSYS01_status 
{
  TSYS01_status_ok,
  TSYS01_status_no_i2c_acknowledge,
  TSYS01_status_i2c_transfer_error,
  TSYS01_status_crc_error
};

class TSYS01 
{

public:
  TSYS01(PinName sda, PinName scl);

  /**
   * \brief Perform initial configuration. Has to be called once.
   */
  void begin();
  /*{
      TSYS01_i2c_address = TSYS01_ADDR_CSB_0;
  };*/


  /**
   * \brief Configures TSYS01 I2C address to be used depending on HW
   * configuration
   *
   * \param[in] address : TSYS01 I2C address
   *
   */
  void set_address(enum TSYS01_address address);

  /**
   * \brief Reset the TSYS01 device
   *
   * \return TSYS01_status : status of TSYS01
   *       - TSYS01_status_ok : I2C transfer completed successfully
   *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
   *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum TSYS01_status reset(void);

  /**
   * \brief Reads the temperature ADC value
   *
   * \param[out] uint32_t* : Temperature ADC value.
   *
   * \return TSYS01_status : status of TSYS01
   *       - TSYS01_status_ok : I2C transfer completed successfully
   *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
   *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - TSYS01_status_crc_error : CRC error on PROM coefficients
   */
  enum TSYS01_status read_temperature(float *temperature);

private:
  /**
   * \brief Reads the TSYS01 EEPROM coefficient stored at address provided.
   *
   * \param[in] uint8_t : Address of coefficient in EEPROM
   * \param[out] uint16_t* : Value read in EEPROM
   *
   * \return TSYS01_status : status of TSYS01
   *       - TSYS01_status_ok : I2C transfer completed successfully
   *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
   *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum TSYS01_status read_eeprom_coeff(uint8_t command, uint16_t *coeff);

  /**
   * \brief Reads the TSYS01 EEPROM coefficient stored at address provided.
   *
   * \param[in] uint8_t : Address of coefficient in EEPROM
   * \param[out] uint16_t* : Value read in EEPROM
   *
   * \return TSYS01_status : status of TSYS01
   *       - TSYS01_status_ok : I2C transfer completed successfully
   *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
   *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum TSYS01_status read_eeprom(void);

  /**
   * \brief CRC check
   *
   * \param[in] uint16_t *: List of EEPROM coefficients
   *
   * \return bool : TRUE if CRC is OK, FALSE if KO
   */
  bool crc_check(uint16_t *n_prom);

  /**
   * \brief Reads the temperature ADC value and compute the degree Celsius one.
   *
   * Perform algorithm computation based on adc 16 bits ( 24-bits value is
   * divided by 256)<BR>
   * \verbatim
   *  T (degC) =     (-2) * k4 * 10e-21 * adc^4 +
   *                    4 * k3 * 10e-16 * adc^3 +
   *                 (-2) * k2 * 10e-11 * adc^2 +
   *                    1 * k1 * 10e-6  * adc +
   *               (-1.5) * k0 * 10e-2
   * \endverbatim
   *
   * Factored into
   * \verbatim
   *  T (degC) = 10e-2.( a.k0 + 10e1 * 10e-5.adc.( b.k1 + 10e-5.adc ( c.k2 +
   * 10e-5.adc.( d.k3 + 10e-5.adc.e.k4 ) ) ) )
   * \endverbatim
   *
   * \param[out] float* : Celsius Degree temperature value
   *
   * \return TSYS01_status : status of TSYS01
   *       - TSYS01_status_ok : I2C transfer completed successfully
   *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
   *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - TSYS01_status_crc_error : CRC error on PROM coefficients
   */
  enum TSYS01_status conversion_and_read_adc(uint32_t *adc);

  /**
* \brief Writes the TSYS01 8-bits command with the value passed
*
* \param[in] uint8_t : Command value to be written.
*
* \return TSYS01_status : status of TSYS01
*       - TSYS01_status_ok : I2C transfer completed successfully
*       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
*       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
*/
  enum TSYS01_status write_command(uint8_t cmd);

  uint8_t TSYS01_i2c_address;// = TSYS01_ADDR_CSB_0;

  bool TSYS01_crc_check(uint16_t *n_prom);
  float* coeff_mul;
  uint16_t eeprom_coeff[PROM_ELEMENTS_NUMBER];

  bool TSYS01_coeff_read;
  
  I2C* i2c_;
};


#endif