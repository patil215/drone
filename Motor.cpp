#include "Motor.h"
#import <Arduino.h>

void Motor::initialize(int p) {
	pin = p;
	pinMode(pin, OUTPUT);
}

void Motor::setSpeed(double s) {
	sp = s;
	analogWrite(pin, sp);
}

double Motor::getSpeed() {
	return sp;
}
