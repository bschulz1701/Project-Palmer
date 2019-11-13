//MargayDemo.ino
#include "Margay.h"
#include <MCP3421.h>
#include <MCP23008.h>
#include <OneWire.h>
#include <DallasTemperature.h>

MCP3421 adc(0x6B); //Initialize MCP3421 
MCP23008 IO(0x20);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 11

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

uint16_t Data[4] = {0}; //Bulk global data array


String Header = "TempGround [C],"; //Information header
uint8_t I2CVals[1] = {0x77}; 

unsigned long UpdateRate = 60; //Number of seconds between readings 

Margay Logger(Model_1v0, Build_B);  //Use build_b with correct ADC for board

void setup() {
  // Logger.PowerAux(ON);
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH); //Turn external power on
  pinMode(11, OUTPUT);
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

void loop() {
	for(int i = 0; i < 4; i++){
		//SET MUX
		IO.PinMode(S0, OUTPUT); //Ensure proper IO expander state
		IO.PinMode(S1, OUTPUT);
		IO.PinMode(IO_EN, OUTPUT);
		boolean S0_State = (i & 0x01); //Pull off lowest bit
		boolean S1_State = (i >> 1) & 0x01; //Pull off next lowest bit
		IO.DigitalWrite(S0, S0_State); //Set S0 value
		IO.DigitalWrite(S1, S1_State); //Set S1 value
		IO.DigitalWrite(IO_EN, LOW); //Wait until after new state set to enable outputs
		// IO.DigitalWrite(S0, LOW);
		// IO.DigitalWrite(S1, LOW);
		// delay(200);
		sensors.begin();
		sensors.getAddress(Therm, 0);
		delay(10);
		sensors.setResolution(Therm, 9);
		sensors.requestTemperatures(); 
		delay(10);
		// int OW_ADR = sensors.getAddress(Therm, 0);
		Serial.println(i);
		Serial.print("Temp = ");
		Serial.println(sensors.getTempC(Therm));
		Serial.print("Analog = ");
		Serial.println(adc.GetVoltage(true));
		IO.DigitalWrite(IO_EN, HIGH);  //Shutdown output until next state is set
	}
	Serial.print("\n\n");

	//Get O2 Data
	// Serial.println(GetVoltage());

	delay(1000);

}

float GetVoltage() 
{

}