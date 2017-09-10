#include "Motor.h"

void Motor::initialize(int pin) {
	this.pin = pin;
	pinMode(pin, OUTPUT);
}

void Motor::setSpeed(double speed) {
	this.speed = speed;
	analogWrite(pin, speed);
}

double Motor::getSpeed() {
	return speed;
}