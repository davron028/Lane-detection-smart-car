#include "Ultrasonic.h"
#include <stdio.h>
#include <wiringPi.h>

void Ultrasonic::begin(int triggerNum, int echoNum){
	triggerPin = triggerNum;
	echoPin = echoNum;
}

float Ultrasonic::ReadDistanceCentimeter(){
	return getDistance();
}

float Ultrasonic::ReadDistanceInch(){
	return (getDistance() / 2.54);
}
float Ultrasonic::getDistance(){
	int start_time, end_time;
	float distance = 0.0;
	
	digitalWrite(triggerPin, LOW);
	delay(100);
	digitalWrite(triggerPin, HIGH);
	
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);
	while(digitalRead(echoPin) == LOW);
	start_time = micros();
	while(digitalRead(echoPin) ==HIGH);
	end_time = micros();
	distance = (end_time - start_time) /29.0/2;
	delay(10);
	return distance;
}


