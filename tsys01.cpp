#include "tsys01.h"
#include "mbed.h"

// TSYS01 device commands
#define TSYS01_RESET_COMMAND 0x1E
#define TSYS01_START_ADC_CONVERSION 0x48
#define TSYS01_READ_ADC_TEMPERATURE 0x00
#define PROM_ADDRESS_READ_ADDRESS_0 0xA0
#define PROM_ADDRESS_READ_ADDRESS_1 0xA2
#define PROM_ADDRESS_READ_ADDRESS_2 0xA4
#define PROM_ADDRESS_READ_ADDRESS_3 0xA6
#define PROM_ADDRESS_READ_ADDRESS_4 0xA8
#define PROM_ADDRESS_READ_ADDRESS_5 0xAA
#define PROM_ADDRESS_READ_ADDRESS_6 0xAC
#define PROM_ADDRESS_READ_ADDRESS_7 0xAE
#define PROM_ELEMENTS_NUMBER 8

#define TSYS01_CONVERSION_TIME 10

// function
TSYS01::TSYS01(PinName sda, PinName scl)
{
  i2c_ = new I2C(sda, scl);
  //400KHz, as specified by the datasheet.
  i2c_->frequency(400000);
  float tempArray[] = {COEFF_MUL_0, COEFF_MUL_1, COEFF_MUL_2, COEFF_MUL_3, COEFF_MUL_4};
  coeff_mul = new float[5];
  for(int idx = 0; idx < 5; idx++)
        coeff_mul[idx] = tempArray[idx]; // Copies each value into the tones_freq array
    delete[] tempArray; // free's the memory used by tempArray
  TSYS01_coeff_read = false;
}

/**
 * \brief Perform initial configuration. Has to be called once.
 */
void TSYS01::begin(void)
{ 
    TSYS01_i2c_address = TSYS01_ADDR_CSB_0;
}

/**
 * \brief Configures TSYS01 I2C address to be used depending on HW configuration
 *
 * \param[in] address : TSYS01 I2C address
 *
 */
void TSYS01::set_address(enum TSYS01_address address) 
{
  if (address == TSYS01_i2c_address_csb_1)
    TSYS01_i2c_address = TSYS01_ADDR_CSB_1;
  else
    TSYS01_i2c_address = TSYS01_ADDR_CSB_0;
}


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
enum TSYS01_status TSYS01::write_command(uint8_t cmd) 
{
  uint8_t i2c_status;
  char tx[1];
  
  tx[0] = cmd;

  i2c_->write((TSYS01_i2c_address << 1) & 0xFE, tx, 1);
  
  i2c_status = TSYS01_status_ok;

  if (i2c_status == TSYS01_STATUS_ERR_OVERFLOW)
    return TSYS01_status_no_i2c_acknowledge;
  if (i2c_status != TSYS01_STATUS_OK)
    return TSYS01_status_i2c_transfer_error;

  return TSYS01_status_ok;
}

/**
 * \brief Reset the TSYS01 device
 *
 * \return TSYS01_status : status of TSYS01
 *       - TSYS01_status_ok : I2C transfer completed successfully
 *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
 *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum TSYS01_status TSYS01::reset(void) 
{
  return write_command(TSYS01_RESET_COMMAND);
}

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
enum TSYS01_status TSYS01::read_eeprom_coeff(uint8_t command, uint16_t *coeff) 
{
  uint8_t i2c_status;
  char tx[1];
  char rx[2];

  tx[0] = command;
  
  i2c_->write((TSYS01_i2c_address << 1) & 0xFE, tx, 1);
  i2c_status = TSYS01_status_ok;
  
  i2c_->read((TSYS01_i2c_address << 1) | 0x01, rx, 2);
  wait_ms(1);

  // Send the conversion command

  if (i2c_status == TSYS01_STATUS_ERR_OVERFLOW)
    return TSYS01_status_no_i2c_acknowledge;
  if (i2c_status != TSYS01_STATUS_OK)
    return TSYS01_status_i2c_transfer_error;

  *coeff = (rx[0] << 8) | rx[1];

  return TSYS01_status_ok;
}

/**
 * \brief Reads the TSYS01 EEPROM coefficients to store them for computation.
 *
 * \return TSYS01_status : status of TSYS01
 *       - TSYS01_status_ok : I2C transfer completed successfully
 *       - TSYS01_status_i2c_transfer_error : Problem with i2c transfer
 *       - TSYS01_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - TSYS01_status_crc_error : CRC error on PROM coefficients
 */
enum TSYS01_status TSYS01::read_eeprom(void) 
{
  enum TSYS01_status status;
  uint8_t i;

  // Read all coefficients from EEPROM
  for (i = 0; i < PROM_ELEMENTS_NUMBER; i++) 
  {
    status = read_eeprom_coeff(PROM_ADDRESS_READ_ADDRESS_0 + i * 2, eeprom_coeff + i);
    if (status != TSYS01_status_ok)
      return status;
  }

  // CRC check
  if (crc_check(eeprom_coeff))
    return TSYS01_status_crc_error;

  TSYS01_coeff_read = true;

  return TSYS01_status_ok;
}

/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool TSYS01::crc_check(uint16_t *n_prom) 
{
  uint8_t cnt;
  uint16_t sum = 0;

  for (cnt = 0; cnt < PROM_ELEMENTS_NUMBER; cnt++)
    // Sum each byte of the coefficients
    sum += ((n_prom[cnt] >> 8) + (n_prom[cnt] & 0xFF));

  return (sum & 0xFF == 0);
}

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
enum TSYS01_status TSYS01::conversion_and_read_adc(uint32_t *adc) 
{
  enum TSYS01_status i2c_status;
  char tx[1];
  char rx[3];

  /* Read data */
  tx[0] = TSYS01_START_ADC_CONVERSION;
  i2c_->write((TSYS01_i2c_address << 1) & 0xFE, tx, 1);
  // Wait for conversion
  wait_ms(TSYS01_CONVERSION_TIME);

  // Read ADC
  tx[0] = TSYS01_READ_ADC_TEMPERATURE;
  i2c_->write((TSYS01_i2c_address << 1) & 0xFE, tx, 1);
  i2c_->read((TSYS01_i2c_address << 1) | 0x01, rx, 3);

  i2c_status = TSYS01_status_ok;
  
  
  if (i2c_status != TSYS01_status_ok)
    return i2c_status;

  // Send the read command
  if (i2c_status != TSYS01_status_ok)
    return i2c_status;

  *adc = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];

  return i2c_status;
}

/**
 * \brief Reads the temperature ADC value and compute the degree Celsius one.
 *
 * Perform algorithm computation based on adc 16 bits ( 24-bits value is divided
 * by 256)<BR>
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

enum TSYS01_status TSYS01::read_temperature(float *temperature) 
{
  enum TSYS01_status status = TSYS01_status_ok;
  uint32_t adc;
  uint8_t i;
  float temp = 0;

  // If first time temperature is requested, get EEPROM coefficients
  if (TSYS01_coeff_read == false)
    status = read_eeprom();
  if (status != TSYS01_status_ok)
    return status;

  status = conversion_and_read_adc(&adc);
  if (status != TSYS01_status_ok)
    return status;

  adc /= 256;

  for (i = 4; i > 0; i--) 
  {
    temp += coeff_mul[i] * eeprom_coeff[1 + (4 - i)]; // eeprom_coeff[1+(4-i)] equiv. ki
    temp *= (float)adc / 100000;
  }
  temp *= 10;
  temp += coeff_mul[0] * eeprom_coeff[5];
  temp /= 100;

  *temperature = temp;

  return status;
}