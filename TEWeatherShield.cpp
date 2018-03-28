#include "TEWeatherShield.h"

TEWeatherShield::TEWeatherShield(void): HTU21D(D14, D15), MS5637(D14, D15), MS8607(D14, D15), TSYS01(D14, D15), TSD305(D14, D15){} //constructor for sensors, pins are sda and scl


void TEWeatherShield::begin(void) 
{
  selectSensor(Sensor_NONE);
  
  TSYS01.begin();
}

void TEWeatherShield::selectSensor(enum TEWeatherShield_Sensor sensor) 
{
  DigitalOut myled0(D9);
  DigitalOut myled1(D10);
  DigitalOut myled2(D11);

  switch (sensor) 
  {
    case Sensor_HTU21D:
      myled0 = 0;
      myled1 = 0;
      myled2 = 0;
      wait_ms(5);
      break;
    case Sensor_MS5637:
      myled0 = 0;
      myled1 = 0;
      myled2 = 0;
      wait_ms(5);
      break;
    case Sensor_MS8607:
      myled0 = 0;
      myled1 = 0;
      myled2 = 1;
      wait_ms(5);
      break;
    case Sensor_TSYS01:
      myled0 = 0;
      myled1 = 1;
      myled2 = 0;
      wait_ms(5);
      break;
    case Sensor_TSD305:
      myled0 = 0;
      myled1 = 1;
      myled2 = 1;
      wait_ms(5);
      break;
    case Sensor_NONE:
      myled0 = 1;
      myled1 = 1;
      myled2 = 1;
      wait_ms(5);
      break;
  }
}

void TEWeatherShield::selectHTU21D() { selectSensor(Sensor_HTU21D); }

void TEWeatherShield::selectMS5637() { selectSensor(Sensor_MS5637); }

void TEWeatherShield::selectMS8607() { selectSensor(Sensor_MS8607); }

void TEWeatherShield::selectTSYS01() { selectSensor(Sensor_TSYS01); }

void TEWeatherShield::selectTSD305() { selectSensor(Sensor_TSD305); }