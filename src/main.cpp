#include <Arduino.h>

void zeroCrossingInterrupt();

const uint8_t ZERO_CROSSING_PIN = 2;
const uint16_t SEMICYCLE_PERIOD = 8000; 

uint32_t lastTime;
uint32_t lastZeroCrossingActivationTime;

volatile uint32_t SemicyclesCounter = 0;  

void setup() {
  pinMode(ZERO_CROSSING_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSSING_PIN), zeroCrossingInterrupt, RISING);
  Serial.begin(9600);
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
		lastZeroCrossingActivationTime = micros();
	}
}

