/****************************************************************
 * based on the original Arduino library for DHT devicess
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
 * 
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

#include "DHT.h"

/* used as part of p_printf() */
bool NoColor=false;

// needed for millis()
struct timeval tv, tv_s;

/* 2 Sec delay between reading */
# define MIN_INTERVAL 2000

/* number of samples for optimize reading */
#define MIN_SAMPLES 10
#define MAX_SAMPLES 250

/**********************************************
 *  define macro's to handle line 
 **********************************************/
#define PIN_LOW()   (SET_L(_pin,LOW))      
#define PIN_HIGH()  (SET_L(_pin,HIGH))
#define PIN_READ()  (bcm2835_gpio_lev(_pin))         

void SET_L (unsigned char line, bool level) {
   
    if (!level)     // set low
    {
        bcm2835_gpio_fsel(line, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_write(line, LOW);
    }
    else            // enable high
    {
        bcm2835_gpio_fsel(line, BCM2835_GPIO_FSEL_INPT);
    }
}

/********************************************************************* 
 * @brief get milli-seconds since start of program *
 * @return Milli-seconds
 *********************************************************************/
static unsigned long millis() {
    
    gettimeofday(&tv, NULL);
    
    /* seconds to milliseconds, microseconds to milli seconds */
    unsigned long calc = ((tv.tv_sec - tv_s.tv_sec) * 1000) + ((tv.tv_usec - tv_s.tv_usec) / 1000);
    return(calc);
}

/*********************************************************************
 * @brief delay for requested milli seconds                         * 
 * @param milliseconds to wait                                      *
 *********************************************************************/
void delay_msec(uint32_t ms){
    struct timespec sleeper;
    
    sleeper.tv_sec  = (time_t)(ms / 1000);
    sleeper.tv_nsec = (long) (ms % 1000) * 1000000; 
    nanosleep(&sleeper, NULL);
}

/******************************************************
 * set maximum priority on current program
 * 
 * source : Adafruit Python DHT library 
 *****************************************************/
void set_max_priority(void)
{
    struct sched_param sched;
    
    memset(&sched, 0, sizeof(sched));
    
    // FIFO scheduler with the highest priority
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
    
    sched_setscheduler(0, SCHED_FIFO, &sched);
}

/******************************************************
 * set default priority on current program
 * 
 * source : Adafruit Python DHT library 
 *****************************************************/
void set_default_priority(void)
{
    struct sched_param sched;
    
    memset(&sched, 0, sizeof(sched));
    
    sched.sched_priority = 0;
    
    sched_setscheduler(0, SCHED_OTHER, &sched);
} 

/******************************************* 
 * @brief Constructor 
 *******************************************/
DHT::DHT() {
    
    init_stats();
    DHT_DEBUG = 0;
}

/**************************************************************** 
 * @brief reset statistics for improving read results 
 ****************************************************************/
void DHT::init_stats()
{
    int i;
 
    //printf("reset counters\nminlc %d maxlc %d, minhc %d maxhc %d \n",
    //         _min_low, _max_low, _min_high, _max_high);
    
    /* reset bounderies */
    _max_low = _max_high = 0;
    _min_low = _min_high = 5000;
    
    /* no samples yet */
    _sampleCount = 0;
    
    /* no quess provided yet */
    _guessed= false;
    
    /* reset the counters */
    for (i = 0; i < 40; i++)
    {
        _zero[i] = _one[i] = 0;
    }
    
}
/******************************************* 
 * @brief Begin
 * @param pin : GPIO to use
 * @param type : type of sensor
 * 
 * @return : true is OK, false is error  
 *******************************************/
bool DHT::begin(uint8_t pin, uint8_t type) {
   
  _pin = pin;
  _type = type; 
  
  if (!bcm2835_init()) 
  {
    p_printf(RED,(char *)"Can't init bcm2835!\n");
    return(false);
  }
  
  // set up the pin
  PIN_HIGH();

  /* Using this value makes sure that millis() - lastreadtime will be
   * >= MIN_INTERVAL right away. Note that this assignment wraps around,
   * but so will the subtraction.
   */
  _lastreadtime = -MIN_INTERVAL;
 
   /* set start time for millis() */
   gettimeofday(&tv_s, NULL);
   
   /* timeout value to wait for ~1mS */
   _maxcycles=5000;
   
   return(true);
}

/*********************************************************************
 * @brief close the BCM2835 library for the raspberry Pi
 *********************************************************************/
void DHT::close()
{
    /* release memory */
    bcm2835_close();
}

/*********************************************************
 * @brief read temperature
 * @param S == Scale.  True == Fahrenheit; False == Celcius
 * @param force : force read independent on time-out
 * 
 * @return value if OK, NAN in case of error
 **********************************************************/
float DHT::readTemperature(bool S, bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
        case DHT11:
          f = data[2];
          if(S) {
            f = convertCtoF(f);
          }
          break;
        case DHT22:
        case DHT21:             // same as AM2301
          f = data[2] & 0x7F;
          f *= 256;
          f += data[3];
          f *= 0.1;
          if (data[2] & 0x80) {
            f *= -1;
          }
          if(S) {
            f = convertCtoF(f);
          }
          break;
        }
  }
  return f;
}

/*********************************************************
 * convert celsius to Fahrenheit
 * @param c : Temperature in Celsius
 * 
 * @return temperaure in Fahrenheit
 *********************************************************/
float DHT::convertCtoF(float c) {
  return c * 1.8 + 32;
}

/*********************************************************
 * convert Fahrenheit to Celsius
 * @param c : Temperature in Fahrenheit
 * 
 * @return temperaure in Celsius
 *********************************************************/

float DHT::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

/*********************************************************
 * @brief read humidity
 * @param force : force read independent on time-out
 * 
 * @return value if OK, NAN in case of error
 **********************************************************/
float DHT::readHumidity(bool force) {
  float f = NAN;            // Not A Number
  
  if (read(force)) {
    switch (_type) {
        case DHT11:
          f = data[0];
          break;
        
        case DHT22:
        
        case DHT21:         // same as AM2301
          f = data[0];
          f *= 256;
          f += data[1];
          f *= 0.1;
          break;
    }
  }
  return f;
}
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
float DHT::computeHeatIndex(float in_temperature, float percentHumidity, bool isFahrenheit) {

  float hi, temperature;
 
  /* if Celsius turn to Fahrenheit */
  if (!isFahrenheit) temperature = convertCtoF(in_temperature);
  else temperature = in_temperature;
  
  /* calculate */  
  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }
  
   /* if celsius was input, convert */
  return isFahrenheit ? hi : convertFtoC(hi);
}

/************************************************************
 *   @brief calculate dew point
 *   @param temp : current temperature
 *   @param hum : current humidity
 *   @param Fahrenheit (true) or celsius (false)
 *   
 *   using the Augst-Roche-Magnus Approximation.
 *   
 *   @return dewpoint
 ************************************************************/   
float DHT::calc_dewpoint(float in_temperature, float hum, bool isFahrenheit) {
    
    float td, H, temp;
    
    /* if Fahrenheit turn to Celsius */
    if (isFahrenheit)  temp = convertFtoC(in_temperature);
    else temp = in_temperature;
 
    /* calculate */
    H = log(hum/100) + ((17.625 * temp) / (243.12 + temp));
    td = 243.04 * H / (17.625 - H);
    
    /* if Fahrenheit was input, convert */
    return isFahrenheit ? convertCtoF(td) : td;
}

/************************************************************
 * @brief Set for debugging the driver
 *
 * @param val : action to be performed
 * 0  = disable debug messages
 * >0 = enable  debug messages
 *
 * This can be called BEFORE performing the begin() call.
 ************************************************************/
void DHT::setDebug(int val) {
     DHT_DEBUG = val;
}

/*********************************************************
 * @brief read values from DHT device
 * @param force : force read independent of time-delay
 * 
 * @return true if OK, false if error
 **********************************************************/
bool DHT::read(bool force) {

  int   retry = 5;
  uint32_t lowCycles, highCycles;
  
  /* buffer to hold the values read from GPIO */
  uint32_t cycles[80];
  
  /* Check if sensor was read less than two seconds ago and return early
   * to use last reading. (unless forced was set)
   */
  uint32_t currenttime = millis(); 
  
  if (!force && ((currenttime - _lastreadtime) < MIN_INTERVAL)) {
    return _lastresult; // return last status
  }
  
  _lastreadtime = currenttime;

  _lastresult = false;
  
  /* Linux is not real-time OS and receiving of the data
   * from the DHT sensor is time critical. To prevent many error
   * messages a retry count has been implemented before failing.
   *  
   * enable debug messages will show the need for retry */
  
  /* set maximum priority on current process */
  set_max_priority();
   
  while(retry-- > 0 && _lastresult == false)
  {
    // Reset 40 bits of received data to zero.
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    
    /* Send start signal.  See DHT datasheet for full signal diagram:
     * http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf
     * 
     * Go into high impedence state to let pull-up raise data line level 
     * and start the reading process.
     */
     
    PIN_HIGH();
    delay_msec(250);
        
    /* First set data line low for 20 milliseconds. */
    PIN_LOW();
    delay_msec(20);
    
    /* End the start signal by enabling data line high for 40 microseconds. */
    PIN_HIGH();
    delayMicroseconds(40);
    
    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      if (DHT_DEBUG) p_printf(RED,(char *) "Timeout waiting for start signal low pulse.\n");
      continue;
    }
    
    if (expectPulse(HIGH) == 0) {
      if (DHT_DEBUG) p_printf(RED,(char *) "Timeout waiting for start signal high pulse.\n");
      continue;
    }
    
    /* Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
     * microsecond low pulse followed by a variable length high pulse.
     * If the high pulse is ~28 microseconds then it's a 0 and if it's 
     * ~70 microseconds then it's a 1.  We measure the cycle count of 
     * the initial 50us low pulse and use that to compare to the cycle 
     * count of the high pulse to determine if the bit is a 0 (high 
     * state cycle count < low state cycle count), or a 1 (high state 
     * cycle count > low state cycle count). Note that for speed all
     * the pulses are read into a array and then examined in a later step.
     */
     
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }
    
    /// Timing critical code is now complete. ///
    
    /* Inspect pulses and determine which ones are 0 (high state cycle 
     * count < low state cycle count), or 1 (high state cycle count > 
     * low state cycle count).
     */

    // reset boundery exception
    _guessed = false;

    for (int i=0; i<40; ++i) {
        
        lowCycles  = cycles[2*i];
        highCycles = cycles[2*i+1];
        
        if ((lowCycles == 0) || (highCycles == 0)) {
          if (DHT_DEBUG) p_printf(RED,(char *) "Timeout waiting for pulse.\n");
          continue;
        }
        
        data[i/8] <<= 1;
        
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (checkCycles(highCycles, lowCycles, i))  data[i/8] |= 1;
        
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
    }

    /* the earlier continue only broke the for-next loop
     * if that was the situation, do not check CRC      */
    if ((lowCycles == 0) || (highCycles == 0)) continue;

    /* only display with succesfull reading */
    if (DHT_DEBUG) 
    {
       p_printf(GREEN,(char *) "Received: "
       "0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x =? 0x%2x\n",
       data[0], data[1], data[2], data[3], data[4],
       data[0]+data[1]+data[2]+data[3]);
    }    
    
    /* Check we read 40 bits and that the checksum matches.*/
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
    {
        /* Update timing for min and max, if not enough samples
         * & NOT an exception with corrected bit               */
        if (_sampleCount < MAX_SAMPLES && _guessed == false)
        {
            _sampleCount++;
 
            for (int i=0; i<40; ++i) {
                addCycles (cycles[2*i+1],cycles[2*i], i);
            }
        }

       _lastresult = true; // break retry loop
    }
    else 
        if (DHT_DEBUG) p_printf(RED,(char *) "Checksum failure!\n");
       
  } // end retry loop
  
  /* restore priority to default */
  set_default_priority();
    
  return _lastresult;
}

/*****************************************************************
 * As timing is critical for reading the DHT and Linux is NOT a 
 * real-time OS, the chance of missing a single bit is very likely. 
 * To improve the likelyhood of a good reading, not only a retry counter
 * has been added in read(), but also estimation/quessing routines.
 * 
 * In addCycles() we are determining the bounderies for the minimum 
 * and maximum count for low and high cycle. Also we count the number 
 * of times there as a 0 or 1 on a bit position. If 1 was most
 * often found on this position a 1 is stored as quess else 0 is stored.
 * 
 * It will also check whether the bounderies still make sense by
 * checking the minimum count. It should not be less than 20% of average.
 * If not it will reset the gathering.
 *  
 * This update will only happen if CRC check was good, there was no 
 * exception in checkCycles() and continues till max_samples has been reached. 
 * 
 * The stored quess is used whenever in checkCycles() it is determined
 * that the count low or high cycle was out of bounds. Thas is marked 
 * as exception and the quess is returned instead of calculated by 
 * comparing the lowCycle adn highCycle count.
 * 
 * This process will only start if a min_samples has been reached. 
 *******************************************************************/
 
 /******************************************************************
 * 
 * @brief check bounderies and try to determine the bit value
 * 
 * @param highCycles : count of high Cycle time
 * @param lowCycles : count of low cycle time (50us leader)
 * @param pos : bit position ( 0 - 39)
 * 
 * @return best result for bit value ( true = 1)
 * 
 *******************************************************************/  
bool DHT::checkCycles(uint32_t highCycles,uint32_t lowCycles, int pos)
{
    if (_sampleCount > MIN_SAMPLES)
    {
        // if cycle count is out of bounds / return the best quess
        if(lowCycles < _min_low || lowCycles > _max_low ||
        highCycles < _min_high || highCycles > _max_high)
        {
            //printf(" outbounds lC %d minlc %d maxlc %d, hc %d minhc %d maxhc %d pos %d\n",
            //lowCycles, _min_low, _max_low, highCycles, _min_high, _max_high, pos);
            
            // indicate quessed / exception
            _guessed = true;
         
            // return quess
            return(_res[pos]);
        }
    }
    
    // High cycles are greater than the 50us low cycle count, must be a 1.
    if (highCycles > lowCycles) return (true);
    
    return(false);
}

 /******************************************************************
 * 
 * @brief set bounderies and try to determine the bit value
 * 
 * @param highCycles : count of high Cycle time
 * @param lowCycles : count of low cycle time (50us leader)
 * @param pos : bit position ( 0 - 39)
 * 
 *******************************************************************/  
void DHT::addCycles (uint32_t highCycles,uint32_t lowCycles, int pos)
{
    uint32_t avg;
    bool _reset=false;
  
    // update boundaries
    if (highCycles > _max_high) _max_high = highCycles;
    if (highCycles < _min_high) _min_high = highCycles;
    
    if (lowCycles > _max_low)   _max_low = lowCycles;
    if (lowCycles < _min_low)   _min_low = lowCycles;
    
    // count how often zero or 1 on position
    if (highCycles > lowCycles)  _one[pos]++;
    else _zero[pos]++;
    
    // set best quess for bit position
    if (_one[pos] > _zero[pos]) _res[pos] = true;
    else _res[pos] = false;

    // check for meaningfull minimum boundary
    if (_sampleCount > MIN_SAMPLES)
    {
        // if minimum is below 20% of average perform reset
        avg = (_min_low + _max_low) / 2;
        if (_min_low < avg / 5) _reset = true;

        avg = (_min_high + _max_high) / 2;
        if (_min_high < avg / 5) _reset = true;
                
        if (_reset)  init_stats();
    }
}
        
/**********************************************************************
 * Expect the signal line to be at the specified level for a period of 
 * time and return a count of loop cycles spent at that level (this 
 * cycle count can be used to compare the relative time of two pulses).
 * If more than a millisecond ellapses without the level changing then 
 * the call fails with a 0 response.
 * This is adapted from Arduino's pulseInLong function (which is only 
 * available in the very latest IDE versions):
 * 
 * https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
 *********************************************************************/
uint32_t DHT::expectPulse(bool level) {
  
  uint32_t count = 0;

    while (PIN_READ() == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }

  return count;
}

/*********************************************************************
 * @brief Display in color
 * @param format : Message to display and optional arguments
 *                 same as printf
 * @param level :  1 = RED, 2 = GREEN, 3 = YELLOW 4 = BLUE 5 = WHITE
 * 
 * if NoColor was set, output is always WHITE.
 *********************************************************************/
void p_printf(int level, char *format, ...) {
    
    char    *col;
    int     coll=level;
    va_list arg;
    
    //allocate memory
    col = (char *) malloc(strlen(format) + 20);
    
    if (NoColor) coll = WHITE;
                
    switch(coll)
    {
    case RED:
        sprintf(col,REDSTR, format);
        break;
    case GREEN:
        sprintf(col,GRNSTR, format);
        break;      
    case YELLOW:
        sprintf(col,YLWSTR, format);
        break;      
    case BLUE:
        sprintf(col,BLUSTR, format);
        break;
    default:
        sprintf(col,"%s",format);
    }

    va_start (arg, format);
    vfprintf (stdout, col, arg);
    va_end (arg);

    fflush(stdout);

    // release memory
    free(col);
}
