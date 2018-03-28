/*********************************************
* TO DO:
* add new serial for ESP8266 communication
* 
**********************************************/

#include "TEWeatherShield.h"

Serial pc(USBTX,USBRX);     // UART tx, rx (to be sent to PC for debug, and later to esp8266

static TEWeatherShield weatherShield; // load the shield

void HTU21D(void);
void MS5367(void);
void MS8607(void);
void TSY01(void);
void TSD305(void);


int main(void)
{
    pc.baud(9600); // set UART baud rate
    
    weatherShield.begin(); // begin the shield
    
    pc.printf("---- weather station is starting ----\r\n");
    wait(1);
    
    while(1)
    {
        HTU21D(); //read htu21d and print the values
        wait(5);
        
        MS5367(); //read ms5637 and print the values    
        wait(5);
        
        MS8607(); //read ms8607 and print the values     
        wait(5);
        
        TSY01(); //read tsys01 and print the values
        wait(5);
        
        TSD305(); //read tsd305 and print the values
        wait(5);
    } // end while 
}

/***************************  FUNCTIONS ***************************/
void HTU21D(void)
{
    static float htu21d_Temperature = 0;              // variable for HTU21D temperature in degrees C
    static float htu21d_Humidity = 0;                 // variable for HTU21D humidity
    
    weatherShield.selectHTU21D(); //select the sensor
    
    pc.printf("============ HTU21D ============\r\n"); //for debug reasons
    
    weatherShield.HTU21D.set_i2c_master_mode(HTU21_i2c_no_hold); //set i2c master mode
          
    weatherShield.HTU21D.read_temperature_and_relative_humidity(&htu21d_Temperature, &htu21d_Humidity); //read the values
    
    pc.printf("temperatue from HTU21D is: %f \r\n", htu21d_Temperature); //print Temperature
    pc.printf("humidity from HTU21D is : %f \r\n",  htu21d_Humidity); //prin Humidity
}


void MS5367(void)
{
    // MS5367 temperature and pressure variables
    static float ms5637_Temp = 0;
    static float ms5637_Press = 0;
    
    weatherShield.selectMS5637(); //select the sensor
    
    pc.printf("============ MS5367 ============\r\n"); //for debug reasons
        
    weatherShield.MS5637.read_temperature_and_pressure(&ms5637_Temp, &ms5637_Press); //read the values
         
    pc.printf("temperatue from MS5637 is: %f \r\n", ms5637_Temp); //print Temperature
    pc.printf("pressure from MS5637 is : %f \r\n",  ms5637_Press); //prin Humidity
}


void MS8607(void)
{
    static float ms8607_temperature = 0;              // MS8607 temperature variable in degress C
    static float ms8607_pressure = 0;                 // MS8607 pressure variable
    static float ms8607_humidity = 0;                 // MS8607 humidity variable
    
    weatherShield.selectMS8607();  //select the sensor
    
    pc.printf("============ MS8607 ============\r\n"); //for debug reasons
    
    weatherShield.MS8607.set_humidity_i2c_master_mode(ms8607_i2c_hold); //set i2c master mode
    
    weatherShield.MS8607.read_temperature_pressure_humidity( &ms8607_temperature, &ms8607_pressure, &ms8607_humidity); //read the values
     
    pc.printf("temperatue from MS8607 is: %f \r\n", ms8607_temperature); //print Temperature
    pc.printf("pressure from MS8607 is : %f \r\n",  ms8607_pressure); //prin Pressure
    pc.printf("humidity from MS8607 is : %f \r\n",  ms8607_humidity); //prin Humidity
}


void TSY01(void)
{
    static float tsys01_temperature = 0; // tsys01 temperature variable
    
    weatherShield.selectTSYS01(); //select the sensor
    
    pc.printf("============ TSY01 ============\r\n"); //for debug reasons
    
    weatherShield.TSYS01.read_temperature(&tsys01_temperature); //read the values
    
    pc.printf("temperatue from TSYS01 is: %f \r\n", tsys01_temperature); //print Temperature
}

void TSD305(void)
{
    static float tsd305_temperature = 0;
    static float tsd305_object_temperature = 0;
    
    weatherShield.selectTSD305(); //select the sensor
    
    pc.printf("============ TSD305 ============\r\n"); //for debug reasons
    
    weatherShield.TSD305.read_temperature_and_object_temperature(&tsd305_temperature, &tsd305_object_temperature); //read the values
    
    pc.printf("temperatue from TSD305 is: %f \r\n", tsd305_temperature); //print Temperature
    pc.printf("Objective temperatue from TSD305 is: %f \r\n", tsd305_object_temperature); //print objective temperature
}