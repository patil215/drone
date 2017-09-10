#include "Bluetooth.h";

void Bluetooth:initialize() {
	// pass
}

bool Bluetooth:packetReady() {
	return Serial.available();
}

char getPacket() {
	if (!packetReady) {
		return NULL;
	}

	char value = Serial.read();
	return value;
}
