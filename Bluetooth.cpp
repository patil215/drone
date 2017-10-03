#include "Bluetooth.h";


void Bluetooth::initialize(int pin) {
	// pass
}

bool Bluetooth::packetReady() {
	return Serial.available();
}

char getPacket() {
	//pass

	//char value = Serial.read();
	//return value;
}
