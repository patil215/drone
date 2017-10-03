#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

class Bluetooth {
	public:
		void initialize(int pin);
		bool packetReady();
		void getPacket();
};

#endif
