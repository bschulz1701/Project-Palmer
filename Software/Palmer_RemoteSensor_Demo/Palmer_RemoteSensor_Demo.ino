//MargayDemo.ino
#include "Margay.h"
#include "Adafruit_SGP30.h"
#include "BME.h"
#include <MCP3421.h>

MCP3421 O2(0x4D); //Initialize MCP3421 with default address, 0x6A
Adafruit_SGP30 sgp;
BME RH;

const uint8_t ADR_ADC = 0x4D;
const float BitValue = 0.8056640625; //Precauculate bit value (3.3/4.096)

const uint8_t ADR_TCS = 0x39; //TCS3400 address
#define TCS_REG_EN 0x80
#define TCS_REG_IR 0x94
#define TCS_REG_RED 0x96
#define TCS_REG_GREEN 0x98
#define TCS_REG_BLUE 0x9A

uint16_t IR_Data = 0;
uint16_t Red_Data = 0;
uint16_t Green_Data = 0;
uint16_t Blue_Data = 0;

uint16_t Data[4] = {0}; //Bulk global data array


String Header = "TempGround [C],"; //Information header
uint8_t I2CVals[1] = {0x77}; 

unsigned long UpdateRate = 60; //Number of seconds between readings 

Margay Logger(Model_1v0, Build_B);  //Use build_b with correct ADC for board

void setup() {
  // Logger.PowerAux(ON);
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH); //Turn external power on
  Serial.begin(38400);
  Serial.println("Begin Palmer Remote Test...");
  sgp.begin();
  RH.begin();
  O2.Begin();
  O2.SetResolution(12); //Set for high resolution
}

int counter = 0;
void loop() {
	//Get TCS3400 Data
	Wire.beginTransmission(ADR_TCS); //Enable sensor
	Wire.write(TCS_REG_EN);
	Wire.write(0x03); //Turn on general power and ADC
	Wire.endTransmission();

	delay(10);

	for(int i = 0; i < 4; i++) {
		uint8_t TempData1 = 0; //Temp data for stoage/concat
		uint8_t TempData2 = 0;
		Wire.beginTransmission(ADR_TCS); //Set pointer to begining of word
		Wire.write(TCS_REG_IR + 2*i);
		Wire.endTransmission();

		Wire.requestFrom(ADR_TCS, 2); //Get word
		TempData1 = Wire.read(); //Read low byte
		TempData2 = Wire.read(); //Read upper byte

		Data[i] = (TempData2 << 8) | TempData1; //Concatonate values
	}

	for(int i = 0; i < 4; i++){
		Serial.println(Data[i]);
	}
	Serial.print("\n\n");


	//Get BME280 Data
	Serial.println(RH.GetPressure());
	Serial.println(RH.GetHumidity());
	Serial.println(RH.GetTemperature());

	//Get SGP30 Data
	//Set auxilary data from BME280
	float temperature = RH.GetTemperature(); // [°C]
	float humidity = RH.GetHumidity(); // [%RH]
	sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

	if (! sgp.IAQmeasure()) {
		Serial.println("Measurement failed");
		return;
	}
	Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
	Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");

	if (! sgp.IAQmeasureRaw()) {
		Serial.println("Raw Measurement failed");
		return;
	}
	Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
	Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");

	// delay(1000);

	counter++;
	if (counter == 30) {
		counter = 0;

		uint16_t TVOC_base, eCO2_base;
		if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
			Serial.println("Failed to get baseline readings");
			return;
		}
		Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
		Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
	}
	Serial.print("\n\n");
	// delay(1000);

	//Get O2 Data
	Serial.println(GetVoltage());

	delay(1000);

}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

float GetVoltage() 
{
	uint8_t Vals[2] = {0}; //Temporary data register
	Wire.requestFrom(ADR_ADC, 2);
	if(Wire.available() == 2) //Get data bytes, 3 bytes of potential data and configuration register 
	{
		Vals[0] = Wire.read();
		Vals[1] = Wire.read();
	}
	int Result = ((Vals[0] & 0x0F) << 8) + Vals[1]; //Concatonate results
	return float(Result)*(BitValue);
}