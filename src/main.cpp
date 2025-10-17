#include <Arduino.h>
#include "servofunc.h"
#include "crc16.h"
#include "Teensy_PWM.h"

//int main(void){}
void SerialEvent_From_PC();
void SerialEvent_From_PC3();
void SerialEvent_From_Servo();
void SerialEvent_From_PC_Debug();

void setup() {

  // initialize serial:
	Serial.begin(115200);//Download, Debug
	Serial1.begin(115200);//Host(PC) Ez-Servo
	Serial2.begin(115200);//RS-485
	Serial3.begin(115200);

	//while (!Serial) {} delay(10);

	//pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
	//analogWriteRes(16);

#ifdef TEENSY_PWM
	PWM_Instance = new Teensy_PWM(LED_PIN, 1000, 0);
#endif

	Servo.ServoPowerInit();

	Servo.SetDebugMode(0);
}

void loop() {

	SerialEvent_From_PC_Debug();
	SerialEvent_From_PC();
	SerialEvent_From_PC3();
	//SerialEvent_From_Servo();

	//Servo.SetDebugMode(0);
	Servo.CheckStatus();
	//Servo.SetDebugMode(1);
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
//                 SerialEvent From Host(PC)
//--------------------------------------------------------------------------------------------------------------------------
void SerialEvent_From_PC3()
{
	char cmdbuf[66];
	uint8_t cnts = 0;
	uint8_t waitcnt = WAIT_CNT;

	while (Serial3.available()) {

		char inChar = (char)Serial3.read();

		if(inChar == '#') {//Command Start String

			memset(cmdbuf, 0, sizeof(cmdbuf));

			while( ++waitcnt ) {  if(Serial3.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }

			while(Serial3.available())
			{
				inChar = Serial3.read();

				if(inChar == '*')
				{
					Servo.RecvCmdFromPC(cmdbuf);
					break;
				}
				cmdbuf[cnts++] = inChar;

				while( ++waitcnt ) {  if(Serial3.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }//80u
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------
//                 SerialEvent From Servo
//--------------------------------------------------------------------------------------------------------------------------
void RemoveDoubleAA(uint8_t* src, uint8_t len, uint8_t* dest, uint8_t* newLen)
{
    if (len < 4) { // 최소 헤더 + 테일
        memcpy(dest, src, len);
        *newLen = len;
        return;
    }

    uint16_t di = 0; // dest index
    dest[di++] = src[0]; // 0xAA
    dest[di++] = src[1]; // 0xCC

    // 본문 (헤더 제외, 테일 제외)
    for (uint16_t i = 2; i < len - 2; i++)
    {
        if (src[i] == 0xAA && src[i + 1] == 0xAA)
        {
            dest[di++] = 0xAA; // 하나만 저장
            i++;               // 다음 바이트는 건너뜀
        }
        else
        {
            dest[di++] = src[i];
        }
    }

    // tail 그대로 복사
    dest[di++] = src[len - 2]; // 0xAA
    dest[di++] = src[len - 1]; // 0xEE

    *newLen = di;
}

void SerialEvent_From_Servo() //From Servo
{
	byte recvbuf[66];
	byte removebuf[66];
	byte inChar;
	uint8_t recvLen = 0, removeLen = 0;
	uint8_t waitcnt = WAIT_CNT;

	while (Serial2.available()) {

		inChar = Serial2.read();

		if((recvLen == 0) && (inChar != 0xAA)) break;
		else if((recvLen == 1) && (inChar != 0xCC)) break;

		recvbuf[recvLen++] = inChar;

		if((recvbuf[recvLen-2] == 0xAA) && (recvbuf[recvLen-1] == 0xEE)) {
			
			RemoveDoubleAA(recvbuf, recvLen, removebuf, &removeLen);

			uint16_t recvCRC = (uint16_t)removebuf[recvLen - 4] | ((uint16_t)removebuf[recvLen - 3] << 8);
			uint16_t calcCRC = CalcCRCbyAlgorithm(&removebuf[2], recvLen - 6);

			if (recvCRC == calcCRC) {
				Servo.RecvDataHandlingFromServo(removebuf, removeLen);
				break;
			}
		}

		while( ++waitcnt ) {  if(Serial2.available()) { waitcnt = WAIT_CNT; break; } delayMicroseconds(1); }//87us
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