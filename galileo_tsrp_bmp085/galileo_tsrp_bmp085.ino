/*
An implementation for Thing Sensor Reporting protocol for Intel Galileo board with bmp085 pressure sensor connected. 
Note that the temperature reported by the bmp085 is somewhat bogus; needs to be verified and calibrated.

This code also makes use of the Thermo sensors built into onboard Quark and ADC. 

References: 
http://thethingsystem.com/dev/Thing-Sensor-Reporting-Protocol.html

Uses bmp085 library
https://bmp085driver.googlecode.com/files/bmp085v0.4.zip

For non-Galileo boards one need to get multicast implementation  

*/
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h> 
#include <string.h>
#include <errno.h> 

//The actual stuff for the Humidity sensor
#include <BMP085.h>


#define _Q(x) #x
#define _S(x) #x ":"

#define PADDING "0123456789012345678901234567890123456789"

#define PARAM_REQUEST_ID "REQUEST_ID"
#define PARAM_DEVICE_PATH "/device/climate/bmp085/sensor" PADDING
#define PARAM_DEVICE_NAME "BMP085" PADDING
#define PARAM_PROPERTIES                    _S("pressure") _Q("Pa") ","         \
                                            _S("temperature") _Q("celsius") "," \
                                            _S("altitude") _Q("m") PADDING

#define PARAM_INSTANCE_NAME "BMP 085 on Galileo" PADDING
#define PARAM_DEV_SERIAL "ThisIsDeviceSerialNumber"
#define PARAM_DEV_UDN    "195a42b0-ef6b-11e2-99d0-1234"
#define PARAM_VALUES                 _S("pressure")   /*Add value*/  "," \
                                     _S("temperature")/*Add value*/ ","   \
                                     _S("altitude")   /*Add value*/  PADDING
									
#define PARAM_UPTIME "UPTIMEMS012345678"

/*MACRO MAGIC: BEGIN*/

#define LIST_PARAMS     \
                PARAM_OP( PARAM_REQUEST_ID ) \
                PARAM_OP( PARAM_DEVICE_PATH  ) \
                PARAM_OP( PARAM_DEVICE_NAME ) \
                PARAM_OP( PARAM_PROPERTIES ) \
                PARAM_OP( PARAM_INSTANCE_NAME ) \
                PARAM_OP( PARAM_DEV_SERIAL ) \
                PARAM_OP( PARAM_DEV_UDN ) \
                PARAM_OP( PARAM_VALUES ) \
                PARAM_OP( PARAM_UPTIME )


#define PARAM_OP(P) POS_ ## P,
enum PARAMETERS_ENUM {
    LIST_PARAMS
    PARAMETERS_COUNT
};

#undef PARAM_OP

typedef struct { const char *name; size_t offset; size_t len; } params_template_t;

static params_template_t params_template[] =
{
#define PARAM_OP(P)  { P, 0, 0 },
    LIST_PARAMS
};
#undef PARAM_OP

/*MACRO MAGIC: END*/

static const char *TSRP_PACKET_PROTO = "{" _S("path") _Q("/api/v1/thing/reporting") "," 
    _S("requestID")  PARAM_REQUEST_ID ","		  
    _S("things") "{"							
        PARAM_DEVICE_PATH  ":"  "{"
            _S("prototype") "{"
                _S("device") "{"
                    _S("name")  PARAM_DEVICE_NAME ","
                    _S("maker") _Q("Intel")
                "} ,"
                _S("name") _Q("true") ","
                _S("status") "[" _Q("present") "," _Q("absent") "," _Q("recent") "]" ","
                _S("properties") "{"
                    PARAM_PROPERTIES
                "}"
            "},"
        _S("instances")  "[{"
            _S("name") PARAM_INSTANCE_NAME ","
            _S("status") _Q("present") ","
            _S("unit")  "{"
                _S("serial") PARAM_DEV_SERIAL ","
                _S("udn") PARAM_DEV_UDN
            "}" ","
            _S("info") "{"
                PARAM_VALUES
            "}" ","
            _S("uptime") PARAM_UPTIME
        "}]"
        "}"
    "}"
"}";


//Goes through the msg_proto and fills in the list with offsets and parameter lengths
//Returns the maximum length of a parameter value or -1 if error
int fill_param_template(const char *msg_proto, params_template_t *list);
//Writes the parameter to buffer
void write_parameter(char *buffer, params_template_t *tpl, char *value);

//Retreives temperature from underlying linux
int getQuarkTemp();
//Retreives temperature from underlying linux
int getADCTemp();

//Blinks a led 
void blink_led(int times);
  
char buffer_bmp085[UDP_TX_PACKET_MAX_SIZE];
char buffer_galileo[UDP_TX_PACKET_MAX_SIZE];
char buffer_adc[UDP_TX_PACKET_MAX_SIZE];

// All TSRP transmissions are via UDP to port 22601 on multicast address '224.0.9.1'.
EthernetUDP udp;

//These are fixed for communicating with ThingSystem
IPAddress ip(224,0,9,1);
unsigned int port = 22601;
BMP085 dps = BMP085();    

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led_pin = 13;

unsigned long requestID = 1;

#define BLINK_PANIC blink_led(-1, 200, 150)

/****************************************************************************************/

void setup() {
  //Opening serail
  Serial.begin(9600);

  // initialize the digital pin as an output.
  pinMode(led_pin, OUTPUT); 

   
  //Setting up packet buffers
  if ( -1 == fill_param_template(TSRP_PACKET_PROTO, params_template) )
  {
    Serial.println("Invalid template. Exiting");
    BLINK_PANIC;
  }

  //Populating buffers with the same packet template
  strcpy(buffer_bmp085, TSRP_PACKET_PROTO);
  strcpy(buffer_galileo, TSRP_PACKET_PROTO);
  strcpy(buffer_adc, TSRP_PACKET_PROTO);

  //Filling the packet buffers with the known information

  //BMP085
  write_parameter(buffer_bmp085, &params_template[POS_PARAM_DEVICE_PATH], _Q( "/device/climate/galileo/bmp085") );
  write_parameter(buffer_bmp085, &params_template[POS_PARAM_DEVICE_NAME], _Q( "BMP085" ));
  write_parameter(buffer_bmp085, &params_template[POS_PARAM_PROPERTIES],_S("pressure") _Q("Pa") "," _S("temperature") _Q("celsius"));
  write_parameter(buffer_bmp085, &params_template[POS_PARAM_INSTANCE_NAME], _Q( "BMP085" ));
  write_parameter(buffer_bmp085, &params_template[POS_PARAM_DEV_SERIAL], _Q( "23410109120912" ));
  write_parameter(buffer_bmp085, &params_template[POS_PARAM_DEV_UDN], _Q( "321-456-789-10-11-12" ));

  //Quark
  write_parameter(buffer_galileo, &params_template[POS_PARAM_DEVICE_PATH], _Q( "/device/climate/galileo/quark") );
  write_parameter(buffer_galileo, &params_template[POS_PARAM_DEVICE_NAME], _Q( "Quark Temperature Sensor" ));
  write_parameter(buffer_galileo, &params_template[POS_PARAM_PROPERTIES],  _S("temperature") _Q("celsius"));
  write_parameter(buffer_galileo, &params_template[POS_PARAM_INSTANCE_NAME], _Q( "Quark" ));
  write_parameter(buffer_galileo, &params_template[POS_PARAM_DEV_SERIAL], _Q( "12010109120912" ));
  write_parameter(buffer_galileo, &params_template[POS_PARAM_DEV_UDN], _Q( "123-456-789-10-11-12" ));

  //ADC on Quark
  write_parameter(buffer_adc, &params_template[POS_PARAM_DEVICE_PATH], _Q( "/device/climate/galileo/adc") );
  write_parameter(buffer_adc, &params_template[POS_PARAM_DEVICE_NAME], _Q( "ADC Temperature Sensor" ));
  write_parameter(buffer_adc, &params_template[POS_PARAM_PROPERTIES],  _S("temperature") _Q("celsius"));
  write_parameter(buffer_adc, &params_template[POS_PARAM_INSTANCE_NAME], _Q( "ADC" ));
  write_parameter(buffer_adc, &params_template[POS_PARAM_DEV_SERIAL], _Q( "12110109120912" ));
  write_parameter(buffer_adc, &params_template[POS_PARAM_DEV_UDN], _Q( "123-457-789-10-11-12" ));


  while(!Serial) { }
  
  Serial.println("\nStarting...");

  //Starting I2C
  Wire.begin(); 

  //Note, if there is DHCP address problem, seems that one need to restart the board
  //Note, the mac parameter in the .begin is not used on Galileo
  Ethernet.begin(0);

  Serial.print("IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
  
  dps.init(MODE_STANDARD, 5000, true);  // 50 meters, true = using meter units
                  // this initialization is useful if current altitude is known,
                  // pressure will be calculated based on TruePressure and known altitude.

  // note: use zeroCal only after initialization.
  dps.zeroCal(101800, 0);    // set zero point

  //Friendly blink
  blink_led(3, 200, 150);

  //Preparing socket for multicast
  //NOTE: Nothing special needs to be done for Galileo (unlike other Arduinos)
  
  udp.begin(0); 
  
}
/****************************************************************************************/
void loop() {
  
    int32_t Temperature = 0, Pressure = 0, Altitude = 0;
    int quark_temp, adc_temp; 
    int ret;
    char buf[256];
    
    dps.getPressure(&Pressure); 
    dps.getAltitude(&Altitude); 
    dps.getTemperature(&Temperature);
    adc_temp=getADCTemp();
    quark_temp = getQuarkTemp();
    
    Serial.print("  Alt(m):"); 
    Serial.print(abs(Altitude/100)); 
    Serial.print("  Pressure(mm Hg):"); 
    Serial.print(Pressure/133.3); 
    Serial.print(" Temp:"); 
    Serial.println(Temperature*0.018); 

    Serial.print("  ADC Temp:"); 
    Serial.print(adc_temp); 
    
    Serial.print("  Quark Temp:"); 
    Serial.print(quark_temp); 
  
    /*Todo: One can sprintf directly into the buffer and use sprintf 
            return value for padding*/
    
    sprintf(buf,"\"pressure\":\"%2.4f\", \"temperature\":\"%2.4f\"", Pressure * 1.0, Temperature*0.018 );
    write_parameter(buffer_bmp085, &params_template[POS_PARAM_VALUES], buf);

    sprintf(buf,"\"temperature\":\"%2.4f\"", quark_temp * 1.0);
    write_parameter(buffer_galileo, &params_template[POS_PARAM_VALUES], buf);

    sprintf(buf,"\"temperature\":\"%2.4f\"", adc_temp * 1.0);
    write_parameter(buffer_adc, &params_template[POS_PARAM_VALUES], buf);

    
    if(! (requestID % 5))
    {
      Serial.println("BMP085 BUFFER:");
      Serial.println(buffer_bmp085); 
      Serial.println();
      Serial.println("GALILEO BUFFER:");
      Serial.println(buffer_galileo); 
      Serial.println();
    }

    {
        char *buffers[]={ buffer_bmp085, buffer_galileo, buffer_adc };
        int i;
        char buf1[24];
        sprintf(buf1,"\"%ul\"", millis());
        sprintf(buf,"\"%i\"",requestID);
        
        for(i = 0;i<3; i++)
        {
           //Common things for all: req id and uptime
           write_parameter(buffers[i],  &params_template[POS_PARAM_REQUEST_ID], buf);          
           write_parameter(buffers[i],  &params_template[POS_PARAM_UPTIME], buf);
          
          udp.beginPacket(ip,port);
          udp.write(buffers[i]);
          if ( 0 > udp.endPacket() )
          {
            Serial.print("ERROR: endPacket, errno = "); 
            Serial.print(errno);
            BLINK_PANIC;
          }
        }
    }
       
  
    requestID = requestID + 1;
    
    //delay(2500); - we'll blink instead
    blink_led(2, 1500, 1000);
  
}
/****************************************************************************************/
int getQuarkTemp(){
  char temp_raw[6];
  int temp;
  FILE *fp;
 
  fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r"); 
  fgets(temp_raw, 5, fp);
  fclose(fp);
  
  temp = atoi(temp_raw);
  temp /= 100;
  return temp;  
  
}
/****************************************************************************************/
int getADCTemp(){
  char scale[4];
  char raw[4];
  char offset[4];
  
  int raw_i;
  int scale_i;
  int offset_i;
  FILE *fp_raw;
  int temp;
  
  fp_raw = fopen("/sys/bus/iio/devices/iio:device0/in_temp0_raw", "r");     //read the values from scale, raw and offset files.
  fgets(raw, 4, fp_raw);                                                    //we need all three values, because the formula for
  fclose(fp_raw);                                                           //calulating the actual temperature in milli-degrees Celcius
                                                                            //is: TEMP = (RAW + OFFSET) * SCALE 
  FILE *fp_scale;
  fp_scale = fopen("/sys/bus/iio/devices/iio:device0/in_temp0_scale", "r");
  fgets(scale, 4, fp_scale);
  fclose(fp_scale);
  
  FILE *fp_offset;
  fp_offset = fopen("/sys/bus/iio/devices/iio:device0/in_temp0_offset", "r");
  fgets(offset, 4, fp_offset);
  fclose(fp_offset);
   
  raw_i = atoi(raw);         //we have the values now, but they are in ASCII form-                                                       
  scale_i = atoi(scale);     //we need them as integers so we can use them for calculations.
  offset_i = atoi(offset);
  
  temp = (raw_i + offset_i) * scale_i;  //Calculate temperature in milli-degrees celcius
  temp /= 1000;                         //divide by 1000 to convert to degrees celcius
  return temp;  
  
}
/****************************************************************************************/
//Goes through the msg_proto and fills in the list with offsets and parameter lengths
//Returns the maximum length of a parameter value or -1 if error
int fill_param_template(const char *msg_proto, params_template_t *list)
{
    const char *pos;
    int max_len = 0;
    size_t i;
    
    for(i=0; i<PARAMETERS_COUNT;i++)
    {
        pos = strstr(msg_proto, list[i].name);
        if(pos != 0)
            list[i].offset = (size_t)(pos - msg_proto);
        else
        {
            //Shit happened
            return -1;      
        }

        list[i].len = strlen(list[i].name);
        if(max_len < list[i].len)  max_len  = list[i].len;
    }
	
    return max_len;
}
/****************************************************************************************/
//Writes the parameter to buffer
void write_parameter(char *buffer, params_template_t *tpl, char *value)
{
    size_t n = strlen(value);
    strncpy( buffer + tpl->offset, value, n);
		
	for(;n<tpl->len;n++) *(buffer + tpl->offset + n) = ' ';
}
/****************************************************************************************/
//Does just that: blinks, waits time_on (ms), switches off waits time_off and does it 
// for 'times'  times or forever, when times < 0
void blink_led(int times, int time_on, int time_off) 
{
  while( times )
  {
    digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(time_on);               // wait for a second
    digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
    delay(time_off);               // wait for a 0.5 second  
    
    times --;
  }
}

