#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#define MAX_SENSOR_DISTANCE   500
#define NO_ECHO                0

class Ultrasonic
{
public:
		void begin( int triggerNum, int echoNum);
		float ReadDistanceCentimeter();
		float ReadDistanceInch();
		
private:
		int triggerPin;
		int echoPin;
		float getDistance();
};

#endif
