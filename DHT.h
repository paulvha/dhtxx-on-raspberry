/****************************************************************
 * Based on the original Arduino library for DHT devicess
 *  DHT library

    MIT license
    written by Adafruit Industries
 *
 * 
 ******************************************************************
 * October 2018 : Changed, enhanced and extended for Raspberry Pi
 * by Paul van Haastrecht (paulvha@hotmail.com)
 * 
 * version 1.0 : initial Raspberry Pi
 *   
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * *****************************************************************
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **********************************************************************/
#ifndef DHT_H
#define DHT_H

# include <bcm2835.h>
# include <getopt.h>
# include <signal.h>
# include <stdint.h>
# include <stdarg.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <unistd.h>
# include <time.h>
# include <math.h>
# include <sys/time.h>
# include <sched.h>

// Define types of sensors.
#define DHT11 11    // 1
#define DHT21 21    // 2
#define DHT22 22    // 3
#define AM2301 21   // 4

// default values for program
#define DEFAULT_GPIO 8
#define DEFAULT_TYPE 1
#define DEFAULT_LOOP 10
#define DEFAULT_DELAY 5

class DHT {
  public:
  
    /******************************************* 
     * @brief Constructor 
     *******************************************/
     DHT();

    /******************************************* 
     * @brief Begin
     * @param pin : GPIO to use
     * @param type : type of sensor
     * 
     * @return : true is OK, false is error  
     *******************************************/
     bool begin(uint8_t pin, uint8_t type);

    /*********************************************************
     * @brief read humidity
     * @param force : force read independent on time-out
     * 
     * @return value if OK, NAN in case of error
     **********************************************************/
     float readHumidity(bool force=false);

    /*********************************************************
     * @brief read temperature
     * @param S == Scale.  True == Fahrenheit; False == Celcius
     * @param force : force read independent on time-out
     * 
     * @return value if OK, NAN in case of error
     **********************************************************/
     float readTemperature(bool S=false, bool force=false);

    /*********************************************************
     * @brief convert Celsius to Fahrenheit
     * @param c : Temperature in Celsius
     * 
     * @return temperaure in Fahrenheit
     *********************************************************/
     float convertCtoF(float);

    /*********************************************************
     * @brief convert Fahrenheit to Celsius
     * @param c : Temperature in Fahrenheit
     * 
     * @return temperaure in Celsius
     *********************************************************/
     float convertFtoC(float);

    /******************************************************************
     * @brief calculate heatindex
     * @param temperature : current temperature
     * @param percentHumidity : current humidity
     * @param isFahrenheit: True == Temperature is in Fahrenheit; False == Celcius
     * 
     * Using both Rothfusz and Steadman's equations
     * http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
     * @return heatindex
     ******************************************************************/ 
     float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit=true);

    /************************************************************
     * @brief calculate dew point
     * @param temp : current temperature
     * @param hum : current humidity
     * @param Fahrenheit (true) or celsius (false)
     *   
     * using the Augst-Roche-Magnus Approximation.
     *   
     * @return dewpoint
     ************************************************************/  
     float calc_dewpoint(float temp, float hum, bool isFahrenheit=true);

    /************************************************************
     * @brief Set for debugging the driver
     *
     * @param val : action to be performed
     * 0  = disable debug messages
     * >0 = enable  debug messages
     *
     * This can be called BEFORE performing the begin() call.
     ************************************************************/
     void setDebug(int val);
     
    /*********************************************************************
     * @brief close the BCM2835 library for the raspberry Pi
     *********************************************************************/
     void close();
     
 private:
 
       /*! read 40 bits from device */
       bool read(bool force=false);

       /*!  check for level changes */
       uint32_t expectPulse(bool level);

       /*! to optimize read results */
       void init_stats();
       bool checkCycles(uint32_t highCycles,uint32_t lowCycles, int pos);
       void addCycles  (uint32_t highCycles,uint32_t lowCycles, int pos);

       uint8_t data[5];
       uint8_t _pin, _type;
       uint32_t _lastreadtime, _maxcycles;
       bool _lastresult;

       /*! enable printing out debug messages. */
       int DHT_DEBUG;

       /*! needed to optimize read result*/
       uint32_t _max_low, _min_low, _max_high, _min_high;
       bool _guessed, _res[40];
       int _one[40], _zero[40];
       int _sampleCount;
};

/*=======================================================================
    to display in color 
  -----------------------------------------------------------------------*/
void p_printf (int level, char *format, ...);

/*! color display enable */
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define WHITE   5

#define REDSTR "\e[1;31m%s\e[00m"
#define GRNSTR "\e[1;92m%s\e[00m"
#define YLWSTR "\e[1;93m%s\e[00m"
#define BLUSTR "\e[1;34m%s\e[00m"

/*! set to disable color output */
extern bool NoColor;

#endif
