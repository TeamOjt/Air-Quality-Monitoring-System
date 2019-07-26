 // Air-Quality-Monitoring-System

//take note of 128 and span of 15 secs

//Library
#include <DHT.h>
#include "Seeed_HM330X.h"
#include "SoftwareSerial.h"
#include <Wire.h> //this is for dust sensor
#include "LowPower.h" //for sleep mode
#include <EEPROM.h> //for saving

//Variable declaration
float d1_temperature=0;
float d1_humidity=0;
float d1_carbonmonoxide;
float d1_carbondioxide;
float d1_pm1,d1_pm2,d1_pm10;
String apiKey = "RM5O8WGSXCNVYN5M";
//String ssid="BITSTOP-MOD3 OJT";    // Wifi network SSID
//String password ="bitstop123ojt";  // Wifi network password
String ssid="Ichie18";    // Wifi network SSID
String password ="bitstopd5t3";  // Wifi network password
String server = "https://thingspeak.com/";

int time_out_count = 0;
unsigned char data[30];
unsigned short value;
unsigned short val[7];
boolean DEBUG=true;
boolean con=true;
boolean emp=false;

byte CO,CO2,TEMP,HUM,PM1,PM2,PM3;
int address=0;

//calibration factors
float RS_gas_mq135 = 0;
float ratio_mq135 = 0;
float sensor_volt_mq135 = 0;
float R0_mq135 = 4481.01; //change this

float RS_gas_mq7 = 0;
float ratio_mq7 = 0;
float sensor_volt_mq7 = 0;
float R0_mq7 = 7990.24; //change this

float ppm_d1_carbondioxide;
float ppm_d1_carbonmonoxide;

int concounter=0;
//PIN declaration
#define d1_mq7_pin A3
#define d1_mq135_pin A2
#define d1_dht_pin 4
#define d1_dht_type DHT11
#define d1_sda A4
#define d1_scl A5
#define HM3301 0x40         
#define CMD 0x88            

DHT d1_dht(4, d1_dht_type);
SoftwareSerial d1_esp(2, 3);// RX, TX
void showResponse(int waitTime)
{
    long t=millis();
    char c;
    while (t+waitTime>millis())
    {
      if (d1_esp.available())
      {
        c=d1_esp.read();
        if (DEBUG) Serial.print(c);
      }
    }                   
}

void resetwifi() 
{
    d1_esp.println("AT+RST");
    delay(1000);
    if(d1_esp.find("OK") ) Serial.println("Module Reset");
}
void connectWifi() 
{
    Serial.println("Initializing Connection for 5 minutes");
    String cmd = "AT+CWJAP=\"" +ssid+"\",\"" + password + "\"";    
    d1_esp.println(cmd);
    delay(3000);
    if(d1_esp.find("OK")) 
    {
        Serial.println("Connected!");
        con=true;
    }
    else 
    {
//        con=false;
//        Serial.println("Cannot connect to wifi");
        connectWifi();    
    }
}
void writeThingSpeak(void)
{
  Serial.println("Initiate Upload");
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += "184.106.153.149"; // api.thingspeak.com IP address
  cmd += "\",80";
  d1_esp.println(cmd);
  if( d1_esp.find("OK")) 
  {
      Serial.println("TCP connection ready");
  }
  delay(1000);
  
  String getStr = "GET /update?api_key=";   // prepare GET string
  getStr += apiKey;

  getStr +="&field1=";
  getStr += String(d1_pm1);
  getStr +="&field2=";
  getStr += String(d1_pm2);
  getStr +="&field3=";
  getStr += String(d1_pm10);
  getStr +="&field4=";
  getStr += String(d1_humidity);
  getStr +="&field5=";
  getStr += String(d1_temperature);
  getStr +="&field6=";
  getStr += String(ppm_d1_carbondioxide);
  getStr +="&field7=";
  getStr += String(ppm_d1_carbonmonoxide);
  getStr += "\r\n\r\n";

  // send data length
  String sendCmd = "AT+CIPSEND=";//determine the number of caracters to be sent.
  d1_esp.print(sendCmd);
  d1_esp.println(getStr.length() );
  delay(500);
  if(d1_esp.find(">")) 
  { 
      Serial.println("Sending.."); 
      d1_esp.print(getStr);
      if( d1_esp.find("SEND OK")) 
      { 
          Serial.println("Packet sent");
          while (d1_esp.available()) 
          {
              String tmpResp = d1_esp.readString();
              Serial.println(tmpResp);
          }
          // close the connection
          d1_esp.println("AT+CIPCLOSE");
      }
  }
}
void mins10()
{
    mins2();
    mins2();
    mins2();
    mins2();
    mins2();
}
void mins2()
{
    //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}
void readsensor()
{
  Serial.println("Initiate Sensor Reading");
  //dht11 program
    d1_humidity = d1_dht.readHumidity();
    d1_temperature = d1_dht.readTemperature();
    Serial.print("Humidity: ");
    Serial.print(d1_humidity);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(d1_temperature);
    Serial.println(" *C");
    
  //mq7 program CO Sensor
    d1_carbonmonoxide = analogRead(d1_mq7_pin); 
    sensor_volt_mq7 = d1_carbonmonoxide/1024*5.0;
    RS_gas_mq7 = (5.0-sensor_volt_mq7)/sensor_volt_mq7;
    ratio_mq7 = RS_gas_mq7/R0_mq7;
    float x_mq7 = 1538.46 * ratio_mq7;
    float ppm_mq7 = pow(x_mq7,-1.709);
    ppm_d1_carbonmonoxide=round(ppm_mq7);
    Serial.print("Carbon Monoxide: "); 
    Serial.print(ppm_d1_carbonmonoxide); 
    Serial.println(" PPM");
    delay(100);
    
  //mq135
    d1_carbondioxide = analogRead(d1_mq135_pin);
    sensor_volt_mq135 = d1_carbondioxide/1024*5.0;   
    RS_gas_mq135 = (5.0-sensor_volt_mq135)/sensor_volt_mq135; 
    ratio_mq135 = RS_gas_mq135/R0_mq135; 
    float x_mq135 = 1538.46 * ratio_mq135;
    float ppm_mq135 = pow(x_mq135,-1.709);
    ppm_d1_carbondioxide=round(ppm_mq135);
    Serial.print("Carbon Dioxide: ");
    Serial.print(ppm_d1_carbondioxide, DEC); 
    Serial.println(" PPM"); 
    
  //Dust sensor
    Wire.beginTransmission(HM3301);
    Wire.write(CMD);
    Wire.endTransmission();
    Wire.requestFrom(0x40, 29);
  
    while (29 != Wire.available()) 
    {
        time_out_count++;
        if (time_out_count > 10)
        delay(1);
    }
    for (int i = 0; i < 28; i++) 
    {
        data[i] = Wire.read();
    }
    value = 0;
    for (int i = 1; i < 7; i++) 
    {
        value = data[i * 2 + 2] << 8 | data[i * 2 + 3];
        val[i - 1] = value;
        
        if(i==1){d1_pm1=val[0];}
        else if(i==2){d1_pm2=val[1];}
        else if(i==3){d1_pm10=val[2];}
    }
    Serial.print("PM1: ");
    Serial.println(d1_pm1);
    Serial.print("PM2.5: ");
    Serial.println(d1_pm2);
    Serial.print("PM10: ");
    Serial.println(d1_pm10);
    delay(100);
}
void clearEEPROM()
{
  for (int ii = 0 ; ii < EEPROM.length() ; ii++) 
  {
    if(EEPROM.read(ii) != 0)                     //skip already "empty" addresses
    {
      EEPROM.write(ii, 0);                       //write 0 to address i
    }
  }
  Serial.println("EEPROM erased");
  address = 0;                                  //reset address counter
}

void setup() 
{
    Serial.begin(9600);  //Serial monitor Baud rate
    d1_esp.begin(9600);
    Wire.begin(); //for i2c connection of dust sensor
    d1_dht.begin();
    //sensor.init();
//    resetwifi();
    connectWifi();    
}

void loop() 
{
  //test if connected or not
    if(con==true) //it is connected 
    {
      Serial.println("Connection Establish");
      //check for EEPROM values
        if(EEPROM.read(0)!=0) //not empty
        {
            Serial.println("EEPROM not Empty");
            for(int i=0;i<EEPROM.length();i++)//iterate all addresses
            {
                byte value = EEPROM.read(i);
                if(value!=0)
                {
                  byte CO,CO2,TEMP,HUM,PM1,PM2,PM3;
                   Serial.println("Read from EEPROM then upload to thingspeak");
                  //read eeprom and assign to variable
                    PM1=EEPROM.read(i);
                    PM2=EEPROM.read(i+1);
                    PM3=EEPROM.read(i+2);
                    HUM=EEPROM.read(i+3);
                    TEMP=EEPROM.read(i+4);
                    CO2=EEPROM.read(i+5);
                    CO=EEPROM.read(i+6);
                    d1_pm1=float(PM1);
                    d1_pm2=float(PM2);
                    d1_pm10=float(PM3);
                    d1_humidity=float(HUM);
                    d1_temperature=float(TEMP);
                    d1_carbondioxide=float(CO2)+255;
                    d1_carbonmonoxide=float(CO)+255;
                  //this is for uploading
                    writeThingSpeak();
                  //this is for sleeping
                    delay(20000); //delay of 20 seconds
                    i=i+6;
                    Serial.println(HUM);
                    Serial.println(TEMP);
                    Serial.println(CO);
                    Serial.println(CO2);
                    Serial.println(PM1);
                    Serial.println(PM2);
                    Serial.println(PM3);
                }
            }
            //tapos na maupload yung mga nasave
          //Clear EEPROM
            clearEEPROM();
          //read sensors
            readsensor();
          //this is for uploading
            writeThingSpeak();
          //this is for sleeping
            mins10();
        }
        else //empty
        {
          Serial.println("EEPROM Empty");
          //read sensors
            readsensor();
          //this is for uploading
            writeThingSpeak();
          //this is for sleeping
            mins10();
        } 
    }
    else //not connected
    {
      Serial.println("Storing Data to EEPROM");
      //read sensors
        readsensor();
      //this is for saving
        PM1 = byte(d1_pm1);
          EEPROM.write(address, PM1);
          address++;
        PM2 = byte(d1_pm2);
          EEPROM.write(address, PM2);
          address++;
        PM3 = byte(d1_pm10);
          EEPROM.write(address, PM3);
          address++;
        HUM = byte(d1_humidity);
          EEPROM.write(address, HUM);
          address++;
        TEMP = byte(d1_temperature);
          EEPROM.write(address, TEMP);
          address++;
        CO2 = byte(d1_carbondioxide);
          EEPROM.write(address, CO2);
          address++;
        CO = byte(d1_carbonmonoxide);
          EEPROM.write(address, CO);
          address++;
          
        //reconnect to wifi
        Serial.println("Reconnecting to wifi");
          connectWifi();
//          mins10();
          mins2();
          mins2();
        //test if 24 hours data naba
        if(address == EEPROM.length()-128)  //check if address counter has reached the end of EEPROM
        {
            address = 0;              //if yes: reset address counter
        }
    }
}
