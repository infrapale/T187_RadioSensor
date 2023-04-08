
// Define message groups to be supported (Astrid.h)
#include <AstridAddrSpace.h>

#include <Arduino.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <Astrid.h>
#include <VillaAstridCommon.h>
#include <SimpleTimer.h> 
#include <SmartLoop.h>
#include <DHT_U.h>

#define DHTPIN            8        // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
#define ZONE  "OD_1"
// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)

DHT_Unified dht(DHTPIN, DHTTYPE);

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
// Adafruit_PCD8544 display = Adafruit_PCD8544(5, 4, 3);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
//#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define TX_BUFF_LEN 61
#define DISPL_BUFF_LEN 10

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     100  //the same on all nodes that talk to each other
#define NODEID        10  
#define BROADCAST     255
#define RECEIVER      BROADCAST    // The recipient of packets
//Match frequency to the hardware version of the radio on your Feather
#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY      RF69_915MHZ
#define ENCRYPTKEY     "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW    true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   9600

#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0  // Pin 2 is IRQ 0!
#define RFM69_RST     9

#define LED           13  // onboard blinky
#define LDR1          A0
#define  LDR2         A1
unit_type_entry Me ={"MH1T1","Terminal","T171","T171","T171","17v01",'0'}; //len = 9,5,5,5,9,byte
time_type MyTime = {2017, 1,30,12,05,55}; 
int16_t packetnum = 0;  // packet counter, we increment per xmission
boolean msgReady;     //Serial.begin(SERIAL_BAUD
boolean SerialFlag;
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);
SFE_BMP180 bmp180;

SimpleTimer timer;
float Temp1;
float Hum1;
float Light1;
float Light2;
double Pressure_BMP180;
double Temp_BMP180;

char radio_packet[TX_BUFF_LEN] = "";
sensors_event_t Sensor1; 
byte rad_turn = 0;
byte read_turn = 0;


void setup() {
    //while (!Serial); // wait until serial console is open, remove if not tethered to computer
    delay(2000);
    Serial.begin(9600);
    //Smart.begin(9600);
    dht.begin();
    Serial.println("T187 RFM69HCW Radio Sensor");
  
    // Hard Reset the RFM module
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);
    delay(100);
    digitalWrite(RFM69_RST, LOW);
    delay(100);
  
    // Initialize radio
    radio.initialize(FREQUENCY,NODEID,NETWORKID);
    if (IS_RFM69HCW) {
       radio.setHighPower();    // Only for RFM69HCW & HW!
    }
    Serial.println("Set Power Level...");
    radio.setPowerLevel(5); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
    radio.encrypt(ENCRYPTKEY);
    if (bmp180.begin())
       Serial.println("BMP180 init success");
    else
    {
       Serial.println("BMP180 init fail (disconnected?)\n\n");
    }

  
    pinMode(LED, OUTPUT);

    Serial.print("\nListening at ");
    Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(" MHz");
    timer.setInterval(10, run_10ms);
    timer.setInterval(1000, run_1000ms);
    timer.setInterval(6000, run_10s);
 
 
   
    

}

void loop() {
    timer.run();
    byte i;
   
    //check if something was received (could be an interrupt from the radio)
    if (false) { //radio.receiveDone()) {
       //print message received to serial
       //Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
       //Serial.print((char*)radio.DATA);
       //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
       for (i=0; (char*)radio.DATA[i]; i++) radio_packet[i] = radio.DATA[i];
       Serial.println(radio_packet);
       //InterpretMsg(radio_packet);
       //RadioMsgHandler(radio_packet);
       
       //check if sender wanted an ACK
       if (radio.ACKRequested()) {
           radio.sendACK();
           Serial.println(" - ACK sent");
       }
       //Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks 
    }

    radio.receiveDone(); //put radio in RX mode
 
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
    if (json_len <= TX_BUFF_LEN){
       for (i=0;i<TX_BUFF_LEN;i++)radio_packet[i]=0;
       JsonString.toCharArray(radio_packet,TX_BUFF_LEN);
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
       radio.send(RECEIVER, radio_msg, strlen(radio_msg),false);
       radio.receiveDone(); //put radio in RX mode
       Serial.println(radio_msg);

       
    }
}

void ReadSensors(void){
  char status;
  if (++read_turn > 8) read_turn = 1;
  switch(read_turn){
     case 1:
        // Get temperature event and print its value.
        dht.temperature().getEvent(&Sensor1);
        if (isnan(Sensor1.temperature)) {
           //Serial.println("Error reading temperature!");
        }
        else {
           Temp1 = Sensor1.temperature;
           //Serial.print("Temperature: ");Serial.print(Sensor1.temperature);Serial.println(" *C");
        }
        break;
     case 2:  
        // Get humidity event and print its value.
        dht.humidity().getEvent(&Sensor1);
        if (isnan(Sensor1.relative_humidity)) {
           //Serial.println("Error reading humidity!");
        }
        else {
           Hum1 = Sensor1.relative_humidity;
           //Serial.print("Humidity: ");Serial.print(Sensor1.relative_humidity);Serial.println("%");
        }
        break;
      case 3:  
         Light1 = float(analogRead(LDR1))/1024; 
         break;
      case 4:  
         Light2 = float(analogRead(LDR2))/1024; 
         break;
      case 5:  
         status = bmp180.startTemperature();
         break;
      case 6:  
         status= bmp180.getTemperature(Temp_BMP180);
         break;
     case 7:  
         status = bmp180.startPressure(3);
         break;
      case 8:  
         status= bmp180.getPressure(Pressure_BMP180,Temp_BMP180);
         break;
    }
}

void Blink(byte PIN, byte DELAY_MS, byte loops){
    for (byte i=0; i<loops; i++) {
        digitalWrite(PIN,HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN,LOW);
        delay(DELAY_MS);
    }
}

void run_10s(void){
   ReadSensors();
}

void run_10ms(void){

}

void run_1000ms(void){
   Serial.println(MyTime.second);

   if (++MyTime.second > 59 ){
      if (++rad_turn > 6) rad_turn = 1;
      switch(rad_turn){
        case 1:
           if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Temp",Temp1,"") > 0 ) radiate_msg(radio_packet);
           break;
        case 2:
           if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Hum",Hum1,"") > 0 ) radiate_msg(radio_packet);
           break;
        case 3:
           if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Light1",Light1,"") > 0 ) radiate_msg(radio_packet);
           break;
        case 4:
           if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Light2",Light2,"") > 0 ) radiate_msg(radio_packet);
           break;
        case 5:   
           if (ConvertFloatSensorToJsonRadioPacket(ZONE,"Temp2",float(Temp_BMP180),"") > 0 ) radiate_msg(radio_packet);
           break;
        case 6:   
           if (ConvertFloatSensorToJsonRadioPacket(ZONE,"P_mb",float(Pressure_BMP180),"") > 0 ) radiate_msg(radio_packet);
           break;
 
      }
        
      MyTime.second = 0;
      if (++MyTime.minute > 59 ){
         MyTime.minute = 0;
         if (++MyTime.hour > 23){
            MyTime.hour = 0;
         }
      }   
   }
}



