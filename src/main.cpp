#include <Arduino.h>

void zeroCrossingInterrupt();
void semicycleIntegralHeatingPowerAdjustment();

const uint16_t SEMICYCLE_PERIOD = 8000;
const uint8_t NUMBER_OF_HEATING_RESISTORS = 6;
const uint8_t HEATING_RESISTOR_PINS[NUMBER_OF_HEATING_RESISTORS] = {A0,A1,A2,A3,A4,A5};
const uint8_t ZERO_CROSSING_PIN = 2;

uint8_t semicyclesPerHeatingResistor[NUMBER_OF_HEATING_RESISTORS];
uint32_t lastTime;
uint32_t lastZeroCrossingActivationTime;

volatile uint32_t SemicyclesCounter = 0;  

void setup() {
	Serial.begin(9600);
  pinMode(ZERO_CROSSING_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSSING_PIN), zeroCrossingInterrupt, RISING);
	for(uint8_t i = 0; i < NUMBER_OF_HEATING_RESISTORS; ++i) {
		pinMode(HEATING_RESISTOR_PINS[i], OUTPUT);
		digitalWrite(HEATING_RESISTOR_PINS[i], LOW);
		semicyclesPerHeatingResistor[i] = (i + 1) * 2;
	}
	lastTime = millis();
	lastZeroCrossingActivationTime = micros();
}

void loop() {
	if(millis() - lastTime >= 1000){
		Serial.println(SemicyclesCounter);
		SemicyclesCounter = 0;
		lastTime = millis();
	}
}

void zeroCrossingInterrupt() {
	if(micros() - lastZeroCrossingActivationTime > SEMICYCLE_PERIOD) {
		++SemicyclesCounter;
		semicycleIntegralHeatingPowerAdjustment();
		lastZeroCrossingActivationTime = micros();
	}
}

void semicycleIntegralHeatingPowerAdjustment(){
	for(uint8_t i = 0; i < NUMBER_OF_HEATING_RESISTORS; ++i) {
		if(semicyclesPerHeatingResistor[i] >= SemicyclesCounter) {
			digitalWrite(HEATING_RESISTOR_PINS[i], HIGH);
		} else {
			digitalWrite(HEATING_RESISTOR_PINS[i], LOW);
		}
		
	}
}

