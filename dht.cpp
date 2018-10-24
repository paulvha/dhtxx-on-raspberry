 /******************************************************************
 * October 2018 : Created for Raspberry Pi
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
# include "DHT.h"

# define VERSION 1

typedef struct dht_par
{
    /* option sensor parameters */
    uint8_t type;
    uint8_t pin;

    /* option program variables */
    uint16_t loop_count;        // number of measurement
    uint16_t loop_delay;        // loop delay in between measurements
    bool timestamp;             // include timestamp in output
    bool tempCel;               // show temperature in Celsius or Fahrenheit
    int verbose;                // verbose level
    bool heatindex;             // include heatindex
    bool dewpoint;              // include dewpoint
 
} dht_par;

char progname[20];

/* global constructor */ 
DHT MySensor;
 
/*********************************************************************
*  @brief close hardware and program correctly
**********************************************************************/
void closeout()
{
    MySensor.close();
    
    exit(EXIT_SUCCESS);
}

/*********************************************************************
* @brief catch signals to close out correctly 
* @param  sig_num : signal that was raised
* 
**********************************************************************/
void signal_handler(int sig_num)
{
    switch(sig_num)
    {
        case SIGINT:
        case SIGKILL:
        case SIGABRT:
        case SIGTERM:
        default:
            printf("\nStopping DHT monitor\n");
            closeout();
            break;
    }
}

/*****************************************
 * @brief setup signals 
 *****************************************/
void set_signals()
{
    struct sigaction act;
    
    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);
    
    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGSEGV,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}

/*********************************************
 * @brief generate timestamp
 * 
 * @param buf : returned the timestamp
 *********************************************/  
void get_time_stamp(char * buf)
{
    time_t ltime;
    struct tm *tm ;
    
    ltime = time(NULL);
    tm = localtime(&ltime);
    
    static const char wday_name[][4] = {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    
    static const char mon_name[][4] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    sprintf(buf, "%.3s %.3s%3d %.2d:%.2d:%.2d %d: ",
    wday_name[tm->tm_wday],  mon_name[tm->tm_mon],
    tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
    1900 + tm->tm_year);
}

/************************************************
 * @brief  initialise the variables 
 * @param dht : pointer to parameters
 ************************************************/
void init_variables(struct dht_par *dht)
{
    /* option program variables */
    dht->pin = DEFAULT_GPIO;
    dht->type= DEFAULT_TYPE;
    dht->loop_count = DEFAULT_LOOP;  // number of measurement
    dht->loop_delay = DEFAULT_DELAY; // loop delay in between measurements
    dht->timestamp = false;         // NOT include timestamp in output
    dht->tempCel = true;            // display temperature in Celsius
    dht->heatindex = false;         // do not include heatindex
    dht->dewpoint = false;          // do not include dewpoint
    dht->verbose = 0;                // No verbose information
}

/**********************************************************
 * @brief lookup correct sensor value
 * @param type : sensor index value
 *********************************************************/
uint8_t look_up_type(uint8_t type)
{
    switch(type)
    {
        case 1:  return (DHT11);
        case 2:  return (DHT21);
        case 3:  return (DHT22);
        case 4:  return (AM2301);
        default: return (DHT11);
    }
}

/**********************************************************
 * @brief initialise the Raspberry PI 
 * @param dht : pointer to parameters
 *********************************************************/
void init_hw(struct dht_par *dht)
{
    /* progress & debug messages tell driver */
    if (dht->verbose > 1) MySensor.setDebug(dht->verbose);

    /* start hardware */
    if (MySensor.begin(dht->pin, look_up_type(dht->type)) == false)
    {
        exit(-1);
    }
}

/*****************************************************************
 * @brief output the results
 * 
 * @param dht : pointer to parameters
 ****************************************************************/
void do_output(struct dht_par *dht)
{
    char buf[30]={0x0}, t;
    float hum, temp, index, dew;
    
    /* include timestamp ? */
    if (dht->timestamp)  get_time_stamp(buf);
    
    hum = MySensor.readHumidity(true);
    
    if (dht->tempCel)   // celsius
    {
        //                             Celsius, not force
        //                                V       V
        temp = MySensor.readTemperature(false, false);
        
        //                                           temp = Celsius
        //                                            V
        index = MySensor.computeHeatIndex(temp, hum, false);
        
        //                                      temp = Celsius
        //                                        V
        dew = MySensor.calc_dewpoint(temp, hum, false);
        t='C';
    }   
    else                // fahrenheit
    {
        temp = MySensor.readTemperature(true, false);
        index = MySensor.computeHeatIndex(temp, hum, true);
        dew = MySensor.calc_dewpoint(temp, hum, true);
        t='F';
    }
    
    printf("%sTemperature: %3.2f *%c, Humidity: %3.2f %%RH", buf, temp,t,hum);  
    
    /* add heatindex ? */
    if (dht->heatindex) 
        printf(", \"feels like temp\": %3.2f *%c", index,t );
    
    /* add dewpoint ? */
    if (dht->dewpoint)
        printf(", dew-point: %3.2f *%c", dew,t );
    
    printf("\n");
}

/*****************************************************************
 * @brief Here the main of the program 
 * @param dht : pointer to parameters
 ****************************************************************/
void main_loop(struct dht_par *dht)
{
    int     loop_set;

    p_printf(GREEN,(char *) "Starting measurement:\n");
            
    /*  check for endless loop */
    if (dht->loop_count > 0 ) loop_set = dht->loop_count;
    else loop_set = 1;
    
    /* loop requested */
    while (loop_set > 0)
    {
        do_output(dht);

        /* delay for seconds */
        sleep(dht->loop_delay);
        
        /* check for endless loop */
        if (dht->loop_count > 0) loop_set--;
    }
}       

/*********************************************************************
* @brief usage information  
* @param dht : pointer to parameters
**********************************************************************/

void usage(struct dht_par *dht)
{
    p_printf(YELLOW, (char *)"%s [options]  (version %d) \n\n"
    
    "sensor settings: \n"
    "-t #       type of sensor                         (default %d)\n"
    "           1 = DHT11          2 = DHT21\n"
    "           3 = DHT22          4 = AM2301\n\n"
    "-p #       GPIO to use                            (default %d)\n"
    

    
    "\nprogram settings\n"
    "-B         Do not display output in color\n"
    "-l #       number of measurements (0 = endless)    (default %d)\n"
    "-w #       waittime (seconds) between measurements (default %d)\n"
    "-v #       verbose/ debug level (0 - 2)            (default %d)\n"
    "-T         add timestamp to output                 (default no stamp)\n"
    "-I         add heat index/ (\"feels like temp\")     (default not added)\n"
    "-D         add dew-point                           (default not added)\n"
    "-F         show temperatures in Fahrenheit\n"

   ,progname, VERSION, dht->type, dht->pin, dht->loop_count, dht->loop_delay, dht->verbose
   );
}

/*********************************************************************
 * Parse parameter input 
 * @param dht : pointer to parameters
 
 *********************************************************************/ 

void parse_cmdline(int opt, char *option, struct dht_par *dht)
{
    switch (opt) {
     
     case 'p':   // GPIO
        dht->pin = (uint8_t) strtod(option, NULL);

        if (dht->pin < 2 || dht->pin > 27)
        {
            p_printf (RED, (char *) "Incorrect GPIO. Must be between 2 and 27\n");
            exit(EXIT_FAILURE);
        }
   
     case 't':   // add type
        dht->type = (uint8_t) strtod(option, NULL);

        if (dht->type < 1 || dht->type > 4)
        {
            p_printf (RED, (char *) "Incorrect type. Must be between 1 and 4\n");
            exit(EXIT_FAILURE);
        }

    case 'I':   // compute heat index
        dht->heatindex = true;
        break; 
        

    case 'D':   // compute dewpoint
        dht->dewpoint = true;
        break; 
        
    case 'B':   // set for no color output
        NoColor = true;
        break; 
              
    case 'l':   // loop count
        dht->loop_count = (uint16_t) strtod(option, NULL);
        break;
          
    case 'w':   // loop delay in between measurements
        dht->loop_delay = (uint16_t) strtod(option, NULL);
        break;
    
    case 'T':  // Add timestamp to output
        dht->timestamp = true;
        break;

    case 'F':  // show temperature in Fahrenheit
        dht->tempCel = false;
        break;
                
    case 'v':   // set verbose / debug level
        dht->verbose = (int) strtod(option, NULL);

        // must be between 0 and 2
        if (dht->verbose < 0 || dht->verbose > 2)
        {
            p_printf (RED, (char *) "Incorrect verbose/debug. Must be  0,1, 2 \n");
            exit(EXIT_FAILURE);
        }
        break;

    case 'h':   // help  (No break)
    
    default: /* '?' */
        usage(dht);
        exit(EXIT_FAILURE);
    }
}

/***********************
 *  program starts here
 **********************/
 
int main(int argc, char *argv[])
{
    int opt;
    struct dht_par dht; // parameters
    
    /* set signals */
    set_signals(); 
 
    /* save name for (potential) usage display */
    strncpy(progname,argv[0],20);

    /* set the initial values */
    init_variables(&dht);

    /* parse commandline */
    while ((opt = getopt(argc, argv, "IBl:v:w:t:HThFD")) != -1)
    {
        parse_cmdline(opt, optarg, &dht);
    }

    /* initialise hardware */
    init_hw(&dht);

    /* main loop to read results */
    main_loop(&dht);
 
    closeout();
}
