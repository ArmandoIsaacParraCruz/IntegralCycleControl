#include <Arduino.h>

const uint8_t ledPin = 13;

void setup() {
	pinMode(ledPin,OUTPUT);
}

void loop() {
	digitalWrite(ledPin,HIGH);
	delay(500);
	digitalWrite(ledPin,LOW);
	delay(500);
}

