/******************************************************************************
LoRa_Node_BETA.ino
Logger software for Project Palmer (BETA development)
Primary Board = v0.0
Remote Board = v0.0
Control Board = LoRa 32u4 (BSFrance II)

Bobby Schulz @ Northern Widget LLC
11/20/2019
Hardware info located at: https://github.com/bschulz1701/Project-Palmer/Hardware

Designed to send data collection over LoRa at 1 second intervals, and update baseline values every 15 seconds along with auto-range light sensor

"The most rewarding things you do in life are often the ones that look like they cannot be done."
-Arnold Palmer

Distributed as-is; no warranty is given.
******************************************************************************/

/*********
Future Feature Set:
- 5v plug detection (check state of pullup)
- Report error code register (array generated, not distributed)
*********/
#include <Arduino.h>
#include "Adafruit_SGP30.h"
#include <BME.h>
#include <MCP3421.h>
#include <MCP3221.h>
#include <MCP23008.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <LoRa.h>
#include <avr/boot.h>

  #define SCK     15
  #define MISO    14
  #define MOSI    16
  #define SS      8
  #define RST     4
  #define DI0     7
  #define BAND    915E6  // 915E6
  #define PABOOST true 
  

#define NUM_DEPTH_NODES 3 //Number of sensor depth nodes
#define BASELINE_UPDATE_PERIOD 15 //How many samples between updates to baseline values
#define UPDATE_PERIOD 1000 //Number of milliseconds between primary sensor updates

//#define WHITE_RABBIT_OBJECT
#define SHERLOCK


///////////////////////////////////// Initialize PRIMARY board //////////////////////////////////////
MCP3421 adc(0x6B); //Initialize MCP3421
MCP23008 IO(0x20);

// Data wire is plugged into pin 9 on the LoRa32u4 II
#define ONE_WIRE_BUS 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress Therm;

//Define IO expander pins
#define ONE_WIRE_IO 0
#define S1 1
#define S0 2
#define IO_EN 3
#define SENSE_5V_EXT 4

float TempData[3] = {0}; //Register for storing soil tempurature data (lowest index, closest to surface)
float MoistData[3] = {0}; //Register for storing soil moisture data (lowest index, closest to surface)

////////////////////////////////////// Initialize REMOTE sensors /////////////////////////////////////
MCP3221 O2(0x4D); //Initialize MCP3421 with default address, 0x6A
Adafruit_SGP30 sgp;
BME RH;

const uint8_t ADR_ADC = 0x4D;
const float BitValue = 0.8056640625; //Precauculate bit value (3.3/4.096)
char serialNumber[30];
const uint8_t ADR_TCS = 0x39; //TCS3400 address
#define TCS_REG_EN 0x80
#define TCS_REG_IR 0x94
#define TCS_REG_RED 0x96
#define TCS_REG_GREEN 0x98
#define TCS_REG_BLUE 0x9A

#define TCS_REG_INT_TIME 0x81
#define TCS_REG_GAIN 0x8F

uint8_t TCS_GainIndex = 0; //Use 1x gain by default
uint8_t TCS_IntIndex = 0; //Use count = 1 by default
uint8_t TCS_IntTimes[5] = {0xFF, 0xF6, 0xDB, 0xC0, 0x00}; //Register values for integration times (increasing)
//Cycles = 256 - TCS_IntTimes[i] = 1, 10, 37, 64, 256
uint8_t TCS_GainVals[4] = {0b00, 0b01, 0b10, 0b11}; //Register values for various gain multiples (increasing)
//Gain = 4^TCS_GainVals[i] = 1, 4, 16, 64
long TCS_WaitTime[5] = {3, 28, 105, 180, 715}; //Wait times for each respective integration time [ms]

char nibbleToHex(uint8_t n);
void readSerialNumber();
boolean TCS_TestOverflow(); //FIX! Improve efficiency!
int ReadWord(uint8_t Adr, uint8_t Reg);
int WriteByte(uint8_t Adr, uint8_t Reg, uint8_t Val); //Write I2C value, given I2C address, register location, register value
void TCS_AutoRange();
int ReadWord(uint8_t Adr, uint8_t Reg);
int WriteByte(uint8_t Adr, uint8_t Reg, uint8_t Val); //Write I2C value, given I2C address, register location, register value
void ClearErrorFlags();
void GetArrayData(); //Get data from temp/soil moisture array
void GetRemoteData(boolean Baseline);
float GetVoltage(); //FIX! Replace with MCP3221 library
uint32_t getAbsoluteHumidity(float temperature, float humidity);
void InitRemote();
void InitPrimary();

uint16_t IR_Data = 0;
uint16_t Red_Data = 0;
uint16_t Green_Data = 0;
uint16_t Blue_Data = 0;

float RemoteData[16] = {0}; //Data array for sensor package
//IR, Red, Green, Blue, IntVal, GainVal, Pressure, Humidity, Temp, TVOC[ppb], eCO2[ppm], H2, Ethanol, eCOS_Base, TVOC_base, O2

#define NUM_FLAGS_REMOTE 4 //Number of sensor flags for remote unit
boolean RemoteErrorFlags[NUM_FLAGS_REMOTE] = {0}; //Set of error flags to keep track of status of remote sensor array, set on error
//TCS3400, BME280, SGP30, MCP3221


void setup() {
  readSerialNumber();
  LoRa.setPins(SS,RST,DI0);
  LoRa.begin(BAND,PABOOST );
  // pinMode(22, OUTPUT);
  // digitalWrite(22, HIGH); //Turn external power on for Margay
  InitPrimary(); //Initialize primary logger board
  InitRemote(); //Initalize remote sensor unit
  Serial1.begin(9600); //Used to echo to broken out serial lines
  #if defined(SHERLOCK)
    Serial.println("ID, IR, Red, Green, Blue, IntVal, GainVal, Pressure, Humidity, Temp, TVOC[ppb], eCO2[ppm], H2, Ethanol, eCOS_Base, TVOC_base, O2, Temp0, Moist0, Temp1, Moist1, Temp2, Moist2");
    Serial1.println("ID, IR, Red, Green, Blue, IntVal, GainVal, Pressure, Humidity, Temp, TVOC[ppb], eCO2[ppm], H2, Ethanol, eCOS_Base, TVOC_base, O2, Temp0, Moist0, Temp1, Moist1, Temp2, Moist2");
  #endif
  delay(25);
  //FIX! Seperate InitArray from InitPrimary
}

void loop() {
  static int UpdateCount = 0;
  unsigned long StartLog = millis(); //Keep start time to keep period
  if(UpdateCount == 0) {
    GetRemoteData(true); //Call data from remote unit, update baseline values
    UpdateCount = BASELINE_UPDATE_PERIOD; //Reset UpdateCount
  }
  else GetRemoteData(false); //For normal cycles, update remote sensor values, no baseline update

  GetArrayData(); //Call data from sensor array
  LoRa.beginPacket();
  //Print data
  #if defined(SHERLOCK)
    Serial.print(serialNumber);
    Serial.print(',');
    Serial1.print(serialNumber);
    Serial1.print(',');
  #endif
  LoRa.print(serialNumber);
  LoRa.print(",");
  for(int i = 0; i < 16; i++) {  //FIX! hardcode
    #if defined(SHERLOCK)
      Serial.print(RemoteData[i]);
      Serial.print(',');
      Serial1.print(RemoteData[i]);
      Serial1.print(',');
    #endif
    LoRa.print(RemoteData[i]);
    LoRa.print(",");
  }

  for(int i = 0; i < 3; i++) {
    #if defined(SHERLOCK)
      Serial.print(TempData[i]);
      Serial.print(',');
      Serial.print(MoistData[i]);
      Serial.print(',');
      Serial1.print(TempData[i]);
      Serial1.print(',');
      Serial1.print(MoistData[i]);
      Serial1.print(',');
    #endif
    LoRa.print(TempData[i]);
    LoRa.print(",");
    LoRa.print(MoistData[i]);
    LoRa.print(",");
  }
  #if defined(SHERLOCK)
    Serial.print("\n");
    Serial1.print("\n");
  #endif
  LoRa.print("\n");
  LoRa.endPacket();
  while(millis() - StartLog < UPDATE_PERIOD); //Wait until update
  UpdateCount--;
}

void InitPrimary()
{
//  pinMode(11, OUTPUT);
  Serial.begin(38400);
  Serial.println("Begin Palmer Remote Test...");
  IO.begin();
  adc.Begin();
  adc.SetResolution(12); //Set for low resolution, high read rate
  IO.PinMode(ONE_WIRE_IO, INPUT_PULLUP);
  IO.PinMode(S0, OUTPUT);
  IO.PinMode(S1, OUTPUT);
  IO.PinMode(IO_EN, OUTPUT);
  IO.PinMode(SENSE_5V_EXT, INPUT);

  IO.DigitalWrite(IO_EN, LOW); //Enable MUX
  IO.DigitalWrite(S0, LOW); //Set to port 0 by default
  IO.DigitalWrite(S1, LOW);
}

void InitRemote()
{
  sgp.begin();
  RH.begin();
  O2.Begin();
}

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

float GetVoltage() //FIX! Replace with MCP3221 library
{
  // uint8_t Vals[2] = {0}; //Temporary data register
  // Wire.requestFrom(ADR_ADC, 2);
  // if(Wire.available() == 2) //Get data bytes, 3 bytes of potential data and configuration register
  // {
  //  Vals[0] = Wire.read();
  //  Vals[1] = Wire.read();
  // }
  // int Result = ((Vals[0] & 0x0F) << 8) + Vals[1]; //Concatonate results
  // return float(Result)*(BitValue);
  return O2.GetVoltage(3.3);
}

void GetRemoteData(boolean Baseline)
{
  ClearErrorFlags(); //Clear error flags before reading data
  //Get TCS3400 Data
  WriteByte(ADR_TCS, TCS_REG_EN, 0x03); //Enable sensor and ADC
  if(Baseline) { //Only auto-range every baseline update time
    TCS_AutoRange(); //FIX! move to lower freq
    WriteByte(ADR_TCS, TCS_REG_INT_TIME, TCS_IntTimes[TCS_IntIndex]); //Update integration time
    WriteByte(ADR_TCS, TCS_REG_GAIN, TCS_GainVals[TCS_GainIndex]); //Update gain value
    delay(TCS_WaitTime[TCS_IntIndex]);
    delay(10);
  }
  boolean TempError = false;
  for(int i = 0; i < 4; i++) {

    uint8_t TempData1 = 0; //Temp data for stoage/concat
    uint8_t TempData2 = 0;
    Wire.beginTransmission(ADR_TCS); //Set pointer to begining of word
    Wire.write(TCS_REG_IR + 2*i);
    if(Wire.endTransmission() != 0) TempError = true;

    Wire.requestFrom(ADR_TCS, 2); //Get word
    TempData1 = Wire.read(); //Read low byte
    TempData2 = Wire.read(); //Read upper byte

    RemoteData[i] = (TempData2 << 8) | TempData1; //Concatonate values, store in global data register
  }
  RemoteErrorFlags[0] = TempError; //Load TCS error flag
  RemoteData[4] = TCS_IntIndex; //Store gain and integration time values
  RemoteData[5] = TCS_GainIndex;

  //Get BME280 Data
  RemoteData[6] = RH.GetPressure();
  RemoteData[7] = RH.GetHumidity();
  RemoteData[8] = RH.GetTemperature();
  // RemoteErrorFlags[1] = ??
  //FIX! Set error flag from BME

  //Get SGP30 Data
  //Set auxilary data from BME280
  float temperature = RH.GetTemperature(); // [°C]
  float humidity = RH.GetHumidity(); // [%RH]
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity)); //Set corrective humidity from BME //FIX! Use only if BME flag is false?

  boolean SPG_ReadError1 = ~sgp.IAQmeasure(); //Call for measurments, collect error info
  RemoteData[9] = sgp.TVOC; //Store TVOC data
  RemoteData[10] = sgp.eCO2; //Store eCO2 data

  boolean SGP_ReadError2 = ~ sgp.IAQmeasureRaw(); //Call for raw measurment, collect error info

  RemoteData[11] = sgp.rawH2; //Store H2 data
  RemoteData[12] = sgp.rawEthanol; //Store Ethanol data

  if(Baseline) { //If baseline data is requested
    uint16_t TVOC_base, eCO2_base;
    TempError = ~sgp.getIAQBaseline(&eCO2_base, &TVOC_base); //Store error flag in temp location
    RemoteData[13] = eCO2_base; //Store eCO2 baseline data
    RemoteData[14] = TVOC_base; //Store TVOC baseline data
  }
  else TempError = false; //If baseline data not collected, simply clear LocalError

  RemoteErrorFlags[2] = SPG_ReadError1 | SGP_ReadError2 | TempError; //Trigger SGP error flag if any source of triggers it

  //Get O2 Data
  RemoteData[15] = GetVoltage(); //FIX! Fix voltage call function, use library instead
  #if defined(WHITE_RABBIT_OBJECT) //Print error flags if set
    Serial.println("Error Flags:");
    for(int i = 0; i < NUM_FLAGS_REMOTE; i++) {
      Serial.println(RemoteErrorFlags[i]);
    }
  #endif
  // RemoteErrorFlags[3] = ?? //FIX! Add error flag
}

void GetArrayData() //Get data from temp/soil moisture array
{
  for(int i = 0; i < NUM_DEPTH_NODES; i++) {

    IO.PinMode(S0, OUTPUT); //Ensure proper IO expander state
    IO.PinMode(S1, OUTPUT);
    IO.PinMode(IO_EN, OUTPUT);
    //SET MUX
    boolean S0_State = (i & 0x01); //Pull off lowest bit
    boolean S1_State = (i >> 1) & 0x01; //Pull off next lowest bit
    IO.DigitalWrite(S0, S0_State); //Set S0 value
    IO.DigitalWrite(S1, S1_State); //Set S1 value
    IO.DigitalWrite(IO_EN, LOW); //Wait until after new state set to enable outputs

    sensors.begin(); //Initialize OneWire sensor on location
    sensors.getAddress(Therm, 0);
    sensors.setResolution(Therm, 9);
    sensors.requestTemperatures();

    TempData[i] = sensors.getTempC(Therm);
    MoistData[i] = adc.GetVoltage(true); //Wait for updated voltage
    IO.DigitalWrite(IO_EN, HIGH);  //Shutdown output until next state is set
  }
}

void ClearErrorFlags()
{
  for(int i = 0; i < NUM_FLAGS_REMOTE; i++) {
    RemoteErrorFlags[i] = false; //Clear all flags
  }
}

//Method: Squeel, increase until maxed, then back off
//Performance:
//Incrment integration values until maxed or overflowed
//If integration is maxed, increase gain, reset integration, step integration again
//If at any time output is overflowed, use previous values of integration and gain
void TCS_AutoRange()
{
  WriteByte(ADR_TCS, TCS_REG_EN, 0x03); //Enable sensor, turn on power, ADC
  WriteByte(ADR_TCS, TCS_REG_INT_TIME, TCS_IntTimes[0]); //Set default integration time (count = 1)
  WriteByte(ADR_TCS, TCS_REG_GAIN, 0x00); //Set default gain (1x)
  delay(TCS_WaitTime[TCS_IntIndex]); //Wait for new data
  TCS_GainIndex = 0; //Reset gain/int vals
  TCS_IntIndex = 0;

  if(TCS_TestOverflow()) { //Within default range, no need for auto-range
    TCS_GainIndex = 0;
    TCS_IntIndex = 0;
  }

  else {
    boolean InRange = false; //Used to keep track of range status
    boolean OverflowState = true; //Used to track overflow state to prevent requirment to perform multiple reads

    uint8_t PrevIntIndex = 0; //used to track the last set of gain/integration values used
    uint8_t PrevGainIndex = 0;
    unsigned long LocalTime = millis();
    while(!InRange && (millis() - LocalTime) < 10000) {


      OverflowState = TCS_TestOverflow(); //Check for overflow single time

      if(OverflowState) { //If overflowed, then use last set of values
        TCS_IntIndex = PrevIntIndex;
        TCS_GainIndex = PrevGainIndex;
        InRange = true; //Break from loop
      }
      if(!OverflowState && TCS_IntIndex < 4) { //If not in range and int time is less than max
        TCS_IntIndex += 1; //Incrment integration time, retry
      }

      if(!OverflowState && TCS_IntIndex >= 4 && TCS_GainIndex < 3) { //If no overflow, int at max, and gain not exceeded
        TCS_IntIndex = 0; //Reset to minimum
        TCS_GainIndex += 1; //Increment gain value
      }

      if(!OverflowState && TCS_IntIndex == 4 && TCS_GainIndex == 3) { //Max value
        InRange = true; //Exit loop
      }

      PrevIntIndex = TCS_IntIndex; //Store values after incrementing
      PrevGainIndex = TCS_GainIndex;
      #if defined(WHITE_RABBIT_OBJECT)
        Serial.print("Gain = ");
        Serial.print(TCS_GainIndex);
        Serial.print(" Int = ");
        Serial.println(TCS_IntIndex);
      #endif
    }
  }

  #if defined(WHITE_RABBIT_OBJECT)
    Serial.print("Gain = ");
    Serial.print(TCS_GainIndex);
    Serial.print(" Int = ");
    Serial.println(TCS_IntIndex);
  #endif

}

int WriteByte(uint8_t Adr, uint8_t Reg, uint8_t Val) //Write I2C value, given I2C address, register location, register value
{
  Wire.beginTransmission(Adr);
  Wire.write(Reg);
  Wire.write(Val);
  return Wire.endTransmission(); //Return error condition
}

int ReadWord(uint8_t Adr, uint8_t Reg)
{
  uint8_t TempData1 = 0; //Temp data for stoage/concat
  uint8_t TempData2 = 0;
  Wire.beginTransmission(Adr); //Set pointer to begining of word
  Wire.write(Reg);
  Wire.endTransmission();

  Wire.requestFrom(Adr, 2); //Get word
  TempData1 = Wire.read(); //Read low byte
  TempData2 = Wire.read(); //Read upper byte
  return (TempData2 << 8) | TempData1; //Concatonate values, retun
}

boolean TCS_TestOverflow() //FIX! Improve efficiency!
{
  WriteByte(ADR_TCS, TCS_REG_INT_TIME, TCS_IntTimes[TCS_IntIndex]); //Update integration time
  WriteByte(ADR_TCS, TCS_REG_GAIN, TCS_GainVals[TCS_GainIndex]); //Update gain value

  delay(TCS_WaitTime[TCS_IntIndex]);
  long OverflowCount = min((256 - long(TCS_IntTimes[TCS_IntIndex]))*1024, 65535); //Find the maximum count value for a given integration time
  #if defined(WHITE_RABBIT_OBJECT)
    Serial.print("Overflow val = "); Serial.println(OverflowCount);
  #endif
  // float EdgeRange = 0.9; //Acceptable range utilization
  long MaxCount = OverflowCount >> 1; //Maximum count is half of overflow value, this is maximum desired range to utilize
  for(int i = 0; i < 4; i++) {
    if(ReadWord(ADR_TCS, TCS_REG_IR + 2*i) > MaxCount) return true; //If there is overflow on any channel, return fail
  }
  return false; //Otherwise return no overflow
}

char nibbleToHex(uint8_t n)
{
  if (n <= 9) { return '0' + n; }
  else { return 'a' + (n - 10); }
}

void readSerialNumber()
{
  char * p = serialNumber;
  for(uint8_t i = 14; i < 24; i++)
  {
    uint8_t b = boot_signature_byte_get(i);
    *p++ = nibbleToHex(b >> 4);
    *p++ = nibbleToHex(b & 0xF);
    *p++ = '-';
  }
  *--p = 0;
}
