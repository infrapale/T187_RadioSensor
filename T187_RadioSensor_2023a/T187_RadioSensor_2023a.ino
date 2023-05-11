/*********************************************************************
 *  T187 Radio Sensor
 *  Read sensor values via UART transmit using RFM69
 *  
 *  - Serial sensor messages
 *  - <OT1:-12.3>
 *  - <OT1:-12.3>
 *  - <OT1:-12.3>
 ********************************************************************
 *  https://github.com/infrapale/T187_RadioSensor 
 *******************************************************************/
// Define message groups to be supported (Astrid.h)
#include <AstridAddrSpace.h>

#include <Arduino.h>
#include <Wire.h>
#include <RH_RF69.h>
#include <SPI.h>
//#include <Astrid.h>
#include <VillaAstridCommon.h>
#include <TaHa.h>
#include "avr_watchdog.h"


#define ZONE  "OD_1"
#define MINUTES_BTW_MSG   10

#define UNIT_ID           'O'
#define TYPE_TEMP         'T'
#define TYPE_HUMIDITY     'H'
#define TYPE_LIGHT        'L'


//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     100  //the same on all nodes that talk to each other
#define NODEID        10  
#define BROADCAST     255
#define MAX_MESSAGE_LEN   68
#define RECEIVER      BROADCAST    // The recipient of packets
//Match frequency to the hardware version of the radio on your Feather
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY      RF69_915MHZ
//#define ENCRYPTKEY     "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW    true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   9600

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ   434.0  //915.0
#define RFM69_CS      10
#define RFM69_INT     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9
#define ENCRYPTKEY     "VillaAstrid_2003" //exactly the same 16 characters/bytes on all nodes!

#define LED_PIN       13  // onboard blinky

AVR_Watchdog   watchdog(4);
int16_t packetnum = 0;  // packet counter, we increment per xmission
boolean msgReady;       //Serial.begin(SERIAL_BAUD
boolean SerialFlag;
RH_RF69 rf69(RFM69_CS, RFM69_INT);

//SimpleTimer timer;
TaHa run_1sec_handle;
TaHa run_led_off_handle;


char    radio_packet[MAX_MESSAGE_LEN];
uint8_t eKey[] ="VillaAstrid_2003"; //exactly the same 16 characters/bytes on all nodes!

void setup() {
    //while (!Serial); // wait until serial console is open, remove if not tethered to computer
    delay(2000);
    Serial.begin(9600);
    Serial.println("T187 RFM69HCW Radio Sensor");
  
    // Hard Reset the RFM module
    pinMode(LED_PIN, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    Serial.println("RFM69 TX Test!");
    digitalWrite(RFM69_RST, HIGH);    delay(100);
    digitalWrite(RFM69_RST, LOW);    delay(100);
    watchdog.set_timeout(600);

    if (!rf69.init()) 
    {
       Serial.println("RFM69 radio init failed");
       while (1);
    }
    Serial.println("RFM69 radio init OK!");
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69.setFrequency(RF69_FREQ)) 
    {
       Serial.println("setFrequency failed");
    }
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
 
    uint8_t key[] ="VillaAstrid_2003"; //exactly the same 16 characters/bytes on all nodes!   
    rf69.setEncryptionKey(key);
  
    
    Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

    //timer.setInterval(10, run_10ms);
    run_1sec_handle.set_interval(1000, RUN_RECURRING, run_1000ms);
    run_led_off_handle.set_interval(5000, RUN_ONCE, run_led_off);
}

void loop() {
    run_1sec_handle.run();
    byte i;

    if (Serial.available() > 0) 
    {    // read the incoming byte:
        char cin = Serial.read();
        parse_serial(cin);
    }
}

#define MAX_VALUE_LEN  16

void parse_serial(char c)
{
    static uint8_t  state = 0;
    static char     type = 0x00; 
    static char     indx = '0' ;
    static char     value_str[16] ={0};
    static uint8_t  value_pos = 0;
    String          float_string;

    //Serial.print("parse: "); Serial.print(state, DEC); Serial.print(" - "); Serial.println(c, DEC);
    switch(state)
    {
        case 0:
            if (c=='<') 
            {
               state++;
               digitalWrite(LED_PIN,HIGH);
               run_led_off_handle.delay_task(5000);
               memset(value_str,0x00, MAX_VALUE_LEN);
               value_pos = 0;
               watchdog.clear();
            }   
            break;
        case 1:
            if (c==UNIT_ID) 
            {
                state++;
            }
            else state=0; 
            break;
        case 2:
            if ((c==TYPE_TEMP) || (c==TYPE_HUMIDITY) || (c==TYPE_LIGHT))
            {
                type = c;
                state++;
            }
          else  state = 0;
            break;
        case 3:
            indx = c;
            state++;
            break;
        case 4:
            if (c==':') state++;
            else state=0; 
            break;
        case 5:
            if (c=='>') 
            {
                String float_string = String(value_str);
                float f_value = float_string.toFloat();
                // char sensor[8];
                switch(type)
                {
                    case TYPE_TEMP:
                        if (indx == '1')
                        {
                            if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Temp2",f_value,"") > 0 ) radiate_msg(radio_packet);
                        }
                        break;
                    case TYPE_HUMIDITY:
                        if (indx == '1')
                        {
                            if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Hum2",f_value,"") > 0 ) radiate_msg(radio_packet);
                        }
                        break;
                    case TYPE_LIGHT:
                        switch(indx)
                        {
                            case  '1': 
                                if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Light1",f_value,"") > 0 ) radiate_msg(radio_packet);
                                break;
                            case  '2': 
                                if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Light2",f_value,"") > 0 ) radiate_msg(radio_packet);
                                break;
                        }
                        
                        break;
                }
                state = 0;
            }
            else
             {
                if (value_pos < (MAX_VALUE_LEN-1) ) value_str[value_pos++] = c;
                else state = 0;
             };
            break;
        case 6:

            break;
    }
}
int ConvertFloatSensorToJsonRadioPacket(char *zone, char *sensor, float value, char *remark ){
    byte i;
    unsigned int json_len;
    //Serial.println("ConvertFloatSensorToJson");
    String JsonString; 
    JsonString = "{\"Z\":\"";
    JsonString += zone;
    JsonString += "\",";
    JsonString += "\"S\":\"";
    JsonString += sensor;  
    JsonString += "\",";
    JsonString += "\"V\":";
    JsonString += value;
    JsonString += ",";
    JsonString += "\"R\":\"";
    JsonString += remark;
    JsonString += "\"}";
    
    //Serial.println(JsonString);
    json_len = JsonString.length();
    if (json_len <= MAX_MESSAGE_LEN){
       for (i=0;i<MAX_MESSAGE_LEN;i++)radio_packet[i]=0;
       JsonString.toCharArray(radio_packet,MAX_MESSAGE_LEN);
       Serial.println(json_len);
       return( json_len );
    }
    else {
      Serial.print("JSON string was too long for the radio packet: "); 
      Serial.println(json_len);
      return(0);
    }
}

void radiate_msg( char *radio_msg ) {
     
    if (radio_msg[0] != 0){
       Serial.println(radio_msg);
       rf69.send((uint8_t *)radio_msg, strlen(radio_msg));
       rf69.waitPacketSent();
      
    }
}

void run_1000ms(void)
{
   // TODO
}

void run_led_off(void)
{
    digitalWrite(LED_PIN,LOW);
}


