#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Ethernet.h"
#include "WebServer.h"
#include "RTClib.h"


const float gVersion = 2.9;
//Adding remote XBEE metrics gathering - use with XBEE_RECEIVE and XBEE_REMOTE_TEMP

//Adding some data persistence

//Connecting solar pump to SOLAR_PUMP_PIN and turning on SOLAR_PUMP_PIN whenever
//gTa > 105

//TODO - initialize these
float gMargin;          //distance in degrees from what the thermostat is set to that the spa will be allowed to vary
float gTt;              //thermostat
float gTa;              //temperature of the solar heater
float gTs;              //temperature of the spa
boolean gHo = false;    //Heater output - T or F
boolean gCo = false;    //Circulation pump output - T or F
boolean gJo = false;    //Jet output - T or F
boolean gPo = false;    //Solar Pump output - T or F

byte gCBToday = 0;           //Used to determine when we are starting a new 24hour period
int gCurrentMinute = 0;      //Used to determine when to update our CBs (every minute)
int gHeatIndexFraction = 0;  //Holds cumulative minutely temperature readings until they sum up to more than 1000
int gSolarPumpFraction = 0;  //Holds cumulative minutely gPo recordings until they add up to more than 4

String gXBeeRelayData = "";      //Holds latest data from I2C XBee relay


#define PREFIX "/spa"
WebServer gWebServer(PREFIX, 8084);
static uint8_t mac[] = { 0x90, 0xA2, 0xDA, 0x04, 0x00, 0xFC };

RTC_DS1307 gRTC;


int CIRC_PIN = 8;
int HEAT_PIN = 7;
int JET_PIN = 6;
int SOLAR_PUMP_PIN = 5;

int TA_ADDRESS = 0x48;
int TS_ADDRESS = 0x49;

//The Mega has 4096 Bytes of EEPROM - we will store our samples and preferences here
//The first 100 addresses will be reserved for preferences
int THERMOSTAT_EEPROM_ADDR = 1;        //thermostat setting stored here
int CB_TODAY_ADDR = 2;                 //Current day of month stored here (for day-change comparison)

//We will use address 100+ as circular buffers to store sample points of data.
//We would like to store 31 days of heater activity, degree heating data, and solar pump usage.
//A byte can hold values from 0 - 255

//Our arduino needs to have 99% uptime so we put it on a watchdog external power circuit
//which reboots the arduino every 60 minutes.  Thus we need to be able to handle interruptions
//without losing data.

//There is no way to tell if our EEPROM is full of "real data" or garbage so we need to reset it
//at least once and then trust it from that time on - we do that by defining RESET_ALL_CB.
//If RESET_ALL_CB is defined then the buffers will be cleared and the program will hang.
//Upload a new image without RESET_ALL_CB defined for normal operation.
//#define RESET_ALL_CB

//A day is a normal 24 hour 12:00am to 11:59pm period.
//The length of each circular buffer is 31 bytes or about a month of data.
int CB_SIZE = 31;      //no more than 100 unless you want to adjust your CB_START and lose your existing data

//the heater daily activity will be stored starting at HEATER_CB_START
//Every minute that the heater is on we will increment today's HEATER_CB by 1
//Maintaining our spa at 100 deg on cloudy days with low temps around 45 deg:
//Emperical evidence suggests that on our coldest day our heater will run a maximum of 10 cycles,
//each cycle lasting about 15 minutes for a total of 150 minutes every 24 hour period.
//As such 256 minutes should be more than enough to represent a days heater activity.
int HEATER_CB_START = 100;

//The daily cumulative heat index will be stored starting at HEAT_INDEX_CB_START.
//We are looking for a number representing the relative "hotness" of a day.
//Again we want this to fit into 8 bits.
//Like the heater activity, we will be taking samples of gTa every minute.
//We expect a maximum day would be (180 deg * 14 hrs * 60 min) + (110 deg * 10 hrs * 60 min) = 217200 / 1000 = 217
//217 is less than 256 so we are OK.  (1000 is our scaling factor)
//Every minute we add gTa to gHeatIndexFraction.  When gHeatIndexFraction > 1000 we subtract 1000 from
//gHeatIndexFraction and increment today's HEAT_INDEX_CB by 1.
int HEAT_INDEX_CB_START = 200;

//the solar pump daily activity will be stored starting at SOLAR_PUMP_CB_START
//on a hot summer day the solar pump could run for 14 hours straight:
//14 * 60 = 840 / 4 = 210.  210 < 256 so we are OK.  (4 is our scaling factor)
//Every minute we check if gPo is true; if so we add 1 to gSolarPumpFraction.  When gSolarPumpFraction > 4 we
//subtract 4 from gSolarPumpFraction and increment today's SOLAR_PUMP_CB by 1.
int SOLAR_PUMP_CB_START = 300;

//Summary of values stored in CBs:
//HEATER_CB holds daily heater usage in minutes
//HEAT_INDEX holds a unitless indication of a days solar energy - higher values == more heat
//SOLAR_PUMP holds daily solar pump usage in minutes / 4 (ex. if SOLAR_PUMP reads 8 then solar pump was on for 32 minutes that day)


void resetCircularBuffers() {
  for (int tAddress = HEATER_CB_START; tAddress <= (HEATER_CB_START + CB_SIZE); tAddress++) {
    EEPROM.write(tAddress, 0);
  }
  
  for (int tAddress = HEAT_INDEX_CB_START; tAddress <= (HEAT_INDEX_CB_START + CB_SIZE); tAddress++) {
    EEPROM.write(tAddress, 0);
  }
  
  for (int tAddress = SOLAR_PUMP_CB_START; tAddress <= (SOLAR_PUMP_CB_START + CB_SIZE); tAddress++) {
    EEPROM.write(tAddress, 0);
  }
  
  DateTime tNow = gRTC.now();
  EEPROM.write(CB_TODAY_ADDR, tNow.day());
    
  while (true) {}
}

void getState(WebServer &pServer, WebServer::ConnectionType pType, char *pURLTail, bool pTailComplete)
{
   pServer.httpSuccess();

  if (pType == WebServer::GET)
  {
      //print out a JSON response
      pServer.print("{");
      
      pServer.print("\"version\":");
      pServer.print(gVersion);
  
      pServer.print(", \"tAvailable\":");
      pServer.print(gTa);
    
      pServer.print(", \"tSpa\":");
      pServer.print(gTs);
      
      pServer.print(", \"tThermostat\":");
      pServer.print(gTt);
      
      pServer.print(", \"heaterOn\":");
      pServer.print(gHo);
  
      pServer.print(", \"circOn\":");
      pServer.print(gCo);
  
      pServer.print(", \"jetOn\":");
      pServer.print(gJo);
          
      pServer.print(", \"lightOn\":");    //legacy - removeme - here only for android app
      pServer.print(gPo);
          
      pServer.print(", \"solarPumpOn\":");
      pServer.print(gPo);
      
      pServer.print(", \"clockDay\":");
      pServer.print(gCBToday);
      
      pServer.print(", \"clockMinute\":");
      pServer.print(gCurrentMinute);

      pServer.print(", \"gHeatIndexFraction\":");
      pServer.print(gHeatIndexFraction);

      pServer.print(", \"gXBeeRelayData\":");
      pServer.print(gXBeeRelayData);
            
      
      pServer.print(", \"heaterCB\":[");
      for (int i = 1; i <= CB_SIZE; i++) {
        pServer.print(EEPROM.read(HEATER_CB_START + i));
        if (i < CB_SIZE)
          pServer.print(", ");
      }
      pServer.print("]");

      pServer.print(", \"heatIndexCB\":[");
      for (int i = 1; i <= CB_SIZE; i++) {
        pServer.print(EEPROM.read(HEAT_INDEX_CB_START + i));
        if (i < CB_SIZE)
          pServer.print(", ");
      }
      pServer.print("]");

      pServer.print(", \"solarPumpCB\":[");
      for (int i = 1; i <= CB_SIZE; i++) {
        pServer.print(EEPROM.read(SOLAR_PUMP_CB_START + i));
        if (i < CB_SIZE)
          pServer.print(", ");
      }
      pServer.print("]");
         
      pServer.print("}");
  }
}

void setPreferences(WebServer &pServer, WebServer::ConnectionType pType, char *pURLTail, bool pTailcomplete) {
  int PARAM_SIZE = 24;

  if (pType == WebServer::POST)
  {
    bool tRepeat;
    char tName[PARAM_SIZE], tValue[PARAM_SIZE];

    do
    {
      tRepeat = pServer.readPOSTparam(tName, PARAM_SIZE, tValue, PARAM_SIZE);

      if (strcmp(tName, "toggleJet") == 0)
        toggleJet();

      if (strcmp(tName, "thermostatUp") == 0)
        updateThermostat(1);

      if (strcmp(tName, "thermostatDown") == 0)
        updateThermostat(-1);
    } 
    while (tRepeat);
        
    return;
  }
}


void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  
  gRTC.begin();

  #ifdef RESET_ALL_CB
    resetCircularBuffers();
  #endif

  pinMode(CIRC_PIN, OUTPUT);
  digitalWrite(CIRC_PIN, HIGH);      //Safe to default circulation pump to off

  pinMode(HEAT_PIN, OUTPUT);
  digitalWrite(HEAT_PIN, HIGH);

  pinMode(JET_PIN, OUTPUT);
  digitalWrite(JET_PIN, HIGH);
  gJo = false;
  
  pinMode(SOLAR_PUMP_PIN, OUTPUT);
  digitalWrite(SOLAR_PUMP_PIN, HIGH);
  gPo = false;
  
  gTt = EEPROM.read(THERMOSTAT_EEPROM_ADDR);
  if ((gTt < 70) || (gTt > 104))
    gTt = 100;
    
  gMargin = 1;      //for now we will keep this hardcoded - we might want to write this to eeprom in the future if we want it flexible

  gCBToday = EEPROM.read(CB_TODAY_ADDR);                        //read in the last known day we recorded some data


  Ethernet.begin(mac);
  gWebServer.setDefaultCommand(&getState);
  gWebServer.addCommand("setPreferences", &setPreferences);

  gWebServer.begin();
}

void toggleJet() {
  gJo = !gJo;
  digitalWrite(JET_PIN, gJo ? LOW : HIGH);    //HIGH is off :0P
}

void solarPump() {
  if (gTa < 104)
    gPo = false;
  
  if (gTa > 105)
    gPo = true;
    
  digitalWrite(SOLAR_PUMP_PIN, gPo ? LOW : HIGH);    //HIGH is off :0P
}

void updateThermostat(int pDirection) {
  gTt += pDirection;
  EEPROM.write(THERMOSTAT_EEPROM_ADDR, gTt);
}

float getTemperature(int pAddress) {
  Wire.requestFrom(pAddress, 2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int tTemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float tCelsius = (float)tTemperatureSum * (float)0.0625;
  float tFarenheit = (1.8 * tCelsius) + 32;
  
  return tFarenheit;
}

float getTa() {
  int TA_ADDRESS = 0x48;
  
  return getTemperature(TA_ADDRESS);
}

float getTs() {
  int TS_ADDRESS = 0x49;
  
  return getTemperature(TS_ADDRESS);
}

void updateHeaterSettings() {
  gTa = getTa();      //temperature of the solar heater
  gTs = getTs();      //temperature of the spa
  
  //PHASE 1
  //Determine if we should turn the heater on or off
  if (gTs < (gTt - gMargin)) {
    //the spa is lower than our thermostat - margin, turn on the heater
    //(we don't want the spa to get any colder than thermostat - margin)
    //we hope that solar will maintain the temp, but as soon as our spa is cold (gTS < (gTt - gMargin)) we just
    //give up and turn on the heater until the spa > thermostat
    gHo = true;
  }
  
  else if (gTs > gTt) {
    //the spa is not too cold
    //if the spa is warmer than our thermostat then turn off the heater
    gHo = false;
  }
  
  
  //PHASE 2
  //determine if we should turn the circ pump on or off
  if (gHo) {
    //the heater is on so we must have the circ pump on as well
    gCo = true;
  }
  
  else if ((gTa > gTs) && (gTs < gTt)) {
    //the heater is off
    //the sun is hot and the spa is not too hot, turn on the circ pump to harvest some solar energy
    gCo = true;
  }
  
  else if (gTs > (gTt + gMargin)) {
    //the heater is not on and (there is not enough solar energy available to heat the spa, or else the spa is at least warm).
    //the spa is hot so turn off the circ pump.
    
      gCo = false;
  }
  
  else if (gTa < gTs) {
    //the sun is cold so we are not getting anything from it, turn off the circ pump.
    gCo = false;
  }
  
  else {
    //leave the circ pump doing whatever it was doing (hysteresis)
  }
  
  
  //PHASE 3
  //finally, update the heater settings
  //"LOW" is "on" is "true"
  digitalWrite(CIRC_PIN, gCo ? LOW : HIGH);
  digitalWrite(HEAT_PIN, gHo ? LOW : HIGH);
}

void updateDataLog() {
  DateTime tNow = gRTC.now();

  //check to see if this is a new day
  if (tNow.day() != gCBToday) {
    //new day, we need to reset today's bucket
    gCBToday = tNow.day();
    EEPROM.write(CB_TODAY_ADDR, gCBToday);
    /*EEPROM.write(HEATER_CB_START + gCBToday, 0);
    EEPROM.write(HEAT_INDEX_CB_START + gCBToday, 0);
    EEPROM.write(SOLAR_PUMP_CB_START + gCBToday, 0);*/
  }
  
  //Increment today's heater usage
  if (gHo) {
    byte tCurrentHeater = EEPROM.read(HEATER_CB_START + gCBToday);
    if (tCurrentHeater < 254)
      EEPROM.write(HEATER_CB_START + gCBToday, tCurrentHeater + 1);
  }
  
  //Increment today's Heat Index
  gHeatIndexFraction += gTa;
  while (gHeatIndexFraction > 1000) {
    gHeatIndexFraction -= 1000;
    byte tCurrentHI = EEPROM.read(HEAT_INDEX_CB_START + gCBToday);
    if (tCurrentHI < 254)
      EEPROM.write(HEAT_INDEX_CB_START + gCBToday, tCurrentHI + 1);
  }
  //EEPROM.write(HEAT_INDEX_FRACTION_ADDR, gHeatIndexFraction);      //TODO want to record for watchdog interrupt reload purposes but need more than a byte!
  
  //Increment today's solar pump usage
  if (gPo) {
    gSolarPumpFraction ++;
    if (gSolarPumpFraction > 4) {
      gSolarPumpFraction -= 4;
      byte tCurrentSolarPump = EEPROM.read(SOLAR_PUMP_CB_START + gCBToday);
      if (tCurrentSolarPump < 254)
        EEPROM.write(SOLAR_PUMP_CB_START + gCBToday, tCurrentSolarPump + 1);
    }
  }  
}

void checkXBeeRelay() {
  Wire.requestFrom(2, 100);    // request 100 bytes from slave device #2

  gXBeeRelayData = String();

  while(Wire.available())    // slave may send less than requested
  { 
    char tChar = Wire.read(); // receive a byte as character
    if (tChar <= 0)
      break;

    gXBeeRelayData += tChar;
  }

  if (gXBeeRelayData.length() == 0)
    gXBeeRelayData = "I2C XBEE Relay error";

  Serial.println(gXBeeRelayData);
}

void loop() {
  updateHeaterSettings();
  
  solarPump();

  //update only once per minute
  DateTime tNow = gRTC.now();
  if (tNow.minute() != gCurrentMinute) {
    gCurrentMinute = tNow.minute();
    
    updateDataLog();
    
    checkXBeeRelay();
  }  
      
  gWebServer.processConnection();
}

//End of Line.
