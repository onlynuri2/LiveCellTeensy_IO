#include <Arduino.h>
#include "servofunc.h"

#include "Teensy_PWM.h"

//int main(void){}
void SerialEvent_From_PC();
void SerialEvent_From_Servo();
void SerialEvent_From_PC_Debug();

void setup() {

  // initialize serial:
	Serial.begin(115200);//Download, Debug
	Serial1.begin(115200);//Host(PC) Ez-Servo
	Serial2.begin(115200);//RS-485

	//while (!Serial) {} delay(10);

	pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
	analogWriteRes(16);

#ifdef TEENSY_PWM
	PWM_Instance = new Teensy_PWM(LED_PIN, 1000, 0);
#endif

	Servo.ServoPowerInit();

}

void loop() {

	SerialEvent_From_PC_Debug();
	SerialEvent_From_PC();
	//SerialEvent_From_Servo();

	Servo.CheckStatus();
}

//--------------------------------------------------------------------------------------------------------------------------
//                 SerialEvent From Host(PC)
//--------------------------------------------------------------------------------------------------------------------------
void SerialEvent_From_PC()
{
	char cmdbuf[66];
	uint8_t cnts = 0;
	uint8_t waitcnt = WAIT_CNT;

	while (Serial1.available()) {

		char inChar = (char)Serial1.read();

		if(inChar == '#') {//Command Start String

			memset(cmdbuf, 0, sizeof(cmdbuf));

			while( ++waitcnt ) {  if(Serial1.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }

			while(Serial1.available())
			{
				inChar = Serial1.read();

				if(inChar == '*')
				{
					Servo.RecvCmdFromPC(cmdbuf);
					break;
				}
				cmdbuf[cnts++] = inChar;

				while( ++waitcnt ) {  if(Serial1.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }//80u
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//                 SerialEvent From Servo
//--------------------------------------------------------------------------------------------------------------------------
void SerialEvent_From_Servo() //From Servo
{
	byte cmdbuf[66];
	byte inChar;
	uint8_t cnts = 0;
	uint8_t waitcnt = WAIT_CNT;

	while (Serial2.available()) {

		inChar = Serial2.read();

		if(cnts == 0)
		{
			if(inChar != 0xAA) break;
			memset(cmdbuf, 0, sizeof(cmdbuf));
		}

		cmdbuf[cnts++] = inChar;

		if((cmdbuf[cnts-2] == 0xAA) && (cmdbuf[cnts-1] == 0xEE)) {
			
			Servo.RecvDataHandlingFromServo(cmdbuf, cnts);
			break;
		}

		while( ++waitcnt ) {  if(Serial2.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }//80u

		if(!Serial2.available()) Servo.RecvDataHandlingFromServo(cmdbuf, cnts);
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//                 SerialEvent From Host(PC) - Debug
//--------------------------------------------------------------------------------------------------------------------------
void SerialEvent_From_PC_Debug()
{
	char cmdbuf[66];
	uint8_t cnts = 0;
	uint8_t waitcnt = WAIT_CNT;

	while (Serial.available()) {

		char inChar = (char)Serial.read();

		if(inChar == '#') {//Command Start String

			memset(cmdbuf, 0, sizeof(cmdbuf));

			while( ++waitcnt ) {  if(Serial.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }

			while(Serial.available())
			{
				inChar = Serial.read();

				if(inChar == '*')
				{
					Servo.RecvCmdFromPC(cmdbuf);
					break;
				}
				cmdbuf[cnts++] = inChar;

				while( ++waitcnt ) {  if(Serial.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }//80u
			}
		}
	}
}