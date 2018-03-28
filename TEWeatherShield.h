#include "mbed.h"

#include "tsys01.h"
#include "tsd305.h"
#include "ms8607.h"
#include "ms5637.h"
#include "htu21d.h"


enum TEWeatherShield_Sensor
{
  Sensor_HTU21D,
  Sensor_MS5637,
  Sensor_MS8607,
  Sensor_TSYS01,
  Sensor_TSD305,
  Sensor_NONE
};

class TEWeatherShield 
{
  public:
    HTU21D HTU21D; //create the htu21d object
    MS5637 MS5637;
    MS8607 MS8607;
    TSYS01 TSYS01;
    TSD305 TSD305;
  
    TEWeatherShield();
  
    void begin();
    void selectSensor(enum TEWeatherShield_Sensor sensor);
    void selectHTU21D();
    void selectMS5637();
    void selectMS8607();
    void selectTSYS01();
    void selectTSD305();
};