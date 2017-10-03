#ifndef _MOTOR_H_
#define _MOTOR_H_

class Motor {
	public:
		void initialize(int pin);
		void setSpeed(double s);
		double getSpeed();
	private:
		int pin;
		double sp;
};

#endif
