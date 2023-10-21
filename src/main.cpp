#include <Arduino.h>

void zeroCrossingInterrupt();

const int zeroCrossingPin = 2; 

uint32_t lastTime;

volatile uint32_t SemicyclesCounter = 0;  

void setup() {
  pinMode(zeroCrossingPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(zeroCrossingPin), zeroCrossingInterrupt, RISING);
  Serial.begin(9600);
	lastTime = millis();
}

void loop() {
	if(millis() - lastTime >= 1000){
		Serial.println(SemicyclesCounter);
		SemicyclesCounter = 0;
		lastTime = millis();
	}
  
}

void zeroCrossingInterrupt() {
  ++SemicyclesCounter;
}

