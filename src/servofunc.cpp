#include <Arduino.h>
#include "servofunc.h"
#include "crc16.h"
#include "servodefine.h"
#include "Teensy_PWM.h"
#include <IntervalTimer.h>
#include "TeensyTimerTool.h"

ServoClass Servo;
extern void SerialEvent_From_Servo();

#ifndef TEENSY_PWM
IntervalTimer pwmTimer;
int dutyCycle = 0;
#endif

using namespace TeensyTimerTool;
IntervalTimer PluseTimer;
//--------------------------------------------------------------------------------------------------------------------------
//                 Servo Power On Init
//--------------------------------------------------------------------------------------------------------------------------
void ServoClass::ServoPowerInit()
{
    Serial.println("Servo Power Init Start -->");

    SetDebugMode(0);

    StepMotorInit();

    for(uint8_t id = ID_X; id <= ID_Y; id++) { GetSlaveInfo(id); }

    for(uint8_t id = ID_X; id <= ID_Y; id++)
    {
        if(GetMotorLive(id) == LIVE)
        {
            if(!Get_FFLAG_SERVOON(id)) { ServoOn(id); }
            else MoveOrg(id);
        }
    }

    Serial.println("Servo PowerInit End <--");
}

/*****************************************************************************
                 Step Motion Move ( MCU -> Step Motor )
******************************************************************************/
void UpPluseTimerCallBack()
{
    digitalWriteFast(UP_PLUSE_PIN, !digitalReadFast(UP_PLUSE_PIN));

    Servo.SetStepCurrentPos(Servo.GetStepCurrentPos() + 1);

    if((Servo.GetStepCurrentPos() == Servo.GetStepTaretPos()) || ((Servo.GetStepCurrentPos() >= Z_MAX_DIST)))
    {
        PluseTimer.end();
    }
}
void DnPluseTimerCallBack()
{
    digitalWriteFast(DN_PLUSE_PIN, !digitalReadFast(DN_PLUSE_PIN));

    Servo.SetStepCurrentPos(Servo.GetStepCurrentPos() - 1);

    if(HOME_S_DETECT)
    {
        Servo.SetMotorLive(ID_Z, 1);
        Servo.SetStepCurrentPos(0);
        Servo.SetStepTaretPos(0);
        PluseTimer.end();
    }
    else if((Servo.GetMotorLive(ID_Z) == LIVE) && ((Servo.GetStepCurrentPos() == Servo.GetStepTaretPos()) || ((Servo.GetStepCurrentPos() <= 0))))
    {
        PluseTimer.end();
    }
}

void ServoClass::MoveStepMotor(uint8_t dir, int pos, int speed)//0:down, 1:up
{
    SetStepTaretPos(pos);

    if((Servo.GetMotorLive(ID_Z) == LIVE) && (GetStepTaretPos() == GetStepCurrentPos())) { Serial.println("Step Z Same Position. Return.."); return; }

    PluseTimer.end();
    if(dir == UP_PLUSE_PIN) PluseTimer.begin(UpPluseTimerCallBack, speed);
    else PluseTimer.begin(DnPluseTimerCallBack, speed);
}

void ServoClass::StepMotorInit()
{
    pinMode(UP_PLUSE_PIN, OUTPUT); digitalWrite(UP_PLUSE_PIN, LOW);
    pinMode(DN_PLUSE_PIN, OUTPUT); digitalWrite(DN_PLUSE_PIN, LOW);

    pinMode(DN_LIMIT_PIN, INPUT);

    Servo.StepMotorHome();

    Serial.print("DEFAULT SPEED Z : "); Serial.println(DEFAULT_SPEED_Z);
    Serial.print("Max SPEED Z : "); Serial.println( GET_SPEED_Z(MAX_SPEED_Z));
}
void ServoClass::StepMotorHome()
{
    if(HOME_S_DETECT)
    {
        uint8_t cnt = 0;
        
        MoveStepMotor(UP_PLUSE_PIN, HOME_S_DISTANCE, GET_SPEED_Z(MAX_SPEED_Z));

        while( HOME_S_DETECT && ++cnt ){ delay(1); }

        if(HOME_S_DETECT) Serial.println("HOME Sensor Boot DETECT...");
    }

    if(GetMotorLive(ID_Z) == LIVE) SetStepCurrentPos(GetStepCurrentPos() + 1000);
    else SetStepCurrentPos(Z_MAX_DIST + 1000);

    MoveStepMotor(DN_PLUSE_PIN, 0);
}

//--------------------------------------------------------------------------------------------------------------------------
//                 Send Cmd ToServo Fucntions ( MCU -> Servo )
//--------------------------------------------------------------------------------------------------------------------------
/*****************************************************************************
                 Get SlaveInfo ( MCU -> Servo ) - 0x01 - #slaveinfo0*
******************************************************************************/
void ServoClass::GetSlaveInfo(uint8_t id)
{
    Serial.print("Current Get Motor Live : "); Serial.print(Servo.GetMotorLive(ID_X)); Serial.print(Servo.GetMotorLive(ID_Y)); Serial.println(Servo.GetMotorLive(ID_Z));
    if(id > ID_Z) { Serial.print("Wrong ID. return "); return; }

    byte data[] = { id, 0x01 };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Set Parameter ( MCU -> Servo ) - 0x12 - #setparm0,6,100000*
******************************************************************************/
void ServoClass::SetParameter(uint8_t id, uint8_t num, int value)
{
    byte data[] = { id, 0x12, num, 0x00, 0x00, 0x00, 0x00 };

    memcpy(&data[3], &value, sizeof(int));

    SendCmdToServo(id, data, sizeof(data));

    Serial.print("SetParm num : "); Serial.print(num); Serial.print(", Val : "); Serial.println(value);
}

/*****************************************************************************
                 Get Parameter ( MCU -> Servo ) - 0x13 - #getparm,0,6*
******************************************************************************/
void ServoClass::GetParameter(uint8_t id, uint8_t num)
{
    byte data[] = { id, 0x13, num };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Stop ( MCU -> Servo ) - 0x2A - #servoon,0,1*
******************************************************************************/
void ServoClass::ServoOn(uint8_t id, uint8_t OnOff)
{
    byte data[] = { id, 0x2A, OnOff };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Alarm Reset ( MCU -> Servo ) - 0x2B - #alarmreset,0*
******************************************************************************/
void ServoClass::AlarmReset(uint8_t id)
{
    byte data[] = { id, 0x2B };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Get Alarm Info ( MCU -> Servo ) - 0x2E - #getalarm,0*
******************************************************************************/
void ServoClass::GetAlarmInfo(uint8_t id)
{
    byte data[] = { id, 0x2E };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Stop ( MCU -> Servo ) - 0x31 - #movestop,x*
******************************************************************************/
void ServoClass::MoveStop(uint8_t id)
{
    if(id == ID_Z) { PluseTimer.end(); return; }

    byte data[] = { id, 0x31 };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Emc Stop ( MCU -> Servo ) - 0x32 - #moveemcstop,0*
******************************************************************************/
void ServoClass::MoveEmcStop(uint8_t id)
{
    if(id == ID_Z) { PluseTimer.end(); return; }

    byte data[] = { id, 0x32 };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Org ( MCU -> Servo ) - 0x33 - #moveorg,x*
******************************************************************************/
void ServoClass::MoveOrg(uint8_t id)
{
    if(id == ID_Z) { StepMotorHome(); return; }

    byte data[] = { id, 0x33 };
    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Singal Abs ( MCU -> Servo ) - 0x34 - #movesabs,x,30000,100000*
******************************************************************************/
void ServoClass::MoveSingleAbs(uint8_t id, int pos, uint32_t speed)
{
    if(id == ID_Z)
    {
        uint8_t dir;
        dir = (pos > GetStepCurrentPos()) ? UP_PLUSE_PIN : DN_PLUSE_PIN;

        MoveStepMotor(dir, pos, GET_SPEED_Z(speed));
        return; 
    }

    byte data[] = { id, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    if( pos > GetMaxDistance(id) )
    {
        Serial.print("Move Single Abs ID : "); Serial.print(id); Serial.print(" Position Over Distance.. - "); Serial.println(pos);
        return;
    }

    memcpy(&data[2], &pos, sizeof(int));
    memcpy(&data[6], &speed, sizeof(uint32_t));

    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Singal Inc ( MCU -> Servo ) - 0x35 - #movesinc,0,30000,50000*
******************************************************************************/
void ServoClass::MoveSingleInc(uint8_t id, int pos, uint32_t speed)
{
    byte data[] = { id, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    memcpy(&data[2], &pos, sizeof(int));
    memcpy(&data[6], &speed, sizeof(uint32_t));

    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move Jog ( MCU -> Servo ) - 0x37 - #movejog,0,1* 1:+, 0:-
******************************************************************************/
void ServoClass::MoveJog(uint8_t id, uint8_t direction, uint32_t speed)
{
    if(id == ID_Z)
    {
        uint8_t dir = (direction > GetStepCurrentPos()) ? UP_PLUSE_PIN : DN_PLUSE_PIN;;

        MoveStepMotor(dir, (dir == UP_PLUSE_PIN) ? Z_MAX_DIST : 0, GET_SPEED_Z(speed));
        return; 
    }

    byte data[] = { id, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00 };

    if((id == ID_Z) && (speed == 20000) && !direction) speed = 19000;//motor bug??

    memcpy(&data[2], &speed, sizeof(int));
    memcpy(&data[6], &direction, sizeof(uint8_t));

    SendCmdToServo(id, data, sizeof(data));

    Serial.print("Move Jog ID : "); Serial.print(id); Serial.print(", direction : "); Serial.print(direction); Serial.print(", speed : "); Serial.println(speed);
}

/*****************************************************************************
                 MoveSingleAbsOver ( MCU -> Servo ) - 0x38
******************************************************************************/
void ServoClass::MoveSingleAbsOver(uint8_t id, int pos)
{
    byte data[] = { id, 0x38, 0x00, 0x00, 0x00, 0x00 };

    if( pos > GetMaxDistance(id) )
    {
        Serial.print("Move Single Abs Over ID : "); Serial.print(id); Serial.print(" Position Over Distance.. - "); Serial.println(pos);
        return;
    }

    memcpy(&data[2], &pos, sizeof(int));

    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 MoveSingleIncOver ( MCU -> Servo ) - 0x39
******************************************************************************/
void ServoClass::MoveSingleIncOver(uint8_t id, int pos)
{
    byte data[] = { id, 0x39, 0x00, 0x00, 0x00, 0x00 };

    memcpy(&data[2], &pos, sizeof(int));

    SendCmdToServo(id, data, sizeof(data));
}

/*****************************************************************************
                 Move All Stop ( MCU -> Servo ) - 0x3B - #moveallstop*
******************************************************************************/
void ServoClass::MoveAllStop()
{
    PluseTimer.end();

    byte data[] = { ALLID, 0x3B };
    SendCmdToServo(ALLID, data, sizeof(data));
}

/*****************************************************************************
                 Move All Emc Stop ( MCU -> Servo ) - 0x3C - #moveallemcstop*
******************************************************************************/
void ServoClass::MoveAllEmcStop()
{
    PluseTimer.end();

    byte data[] = { ALLID, 0x3C };
    SendCmdToServo(ALLID, data, sizeof(data));
}

/*****************************************************************************
                 Move All Org ( MCU -> Servo ) - 0x3D - #moveallorg*
******************************************************************************/
void ServoClass::MoveAllOrg()
{
    StepMotorHome();

    byte data[] = { ALLID, 0x3D };
    SendCmdToServo(ALLID, data, sizeof(data));
}

#if 0
/*****************************************************************************
                 Move All Singal Abs ( MCU -> Servo ) - 0x3E
******************************************************************************/
void ServoClass::MoveAllSingleAbs(int pos, uint32_t speed)
{
    byte data[] = { ALLID, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    memcpy(&data[2], &pos, sizeof(int));
    memcpy(&data[6], &speed, sizeof(uint32_t));

    SendCmdToServo(ALLID, data, sizeof(data));
}

/*****************************************************************************
                 Move All Singal Inc ( MCU -> Servo ) - 0x3F
******************************************************************************/
void ServoClass::MoveAllSingleInc(int pos, uint32_t speed)
{
    Serial.println("Move All Singal Inc...........");
    byte data[] = { ALLID, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    memcpy(&data[2], &pos, sizeof(int));
    memcpy(&data[6], &speed, sizeof(uint32_t));

    SendCmdToServo(ALLID, data, sizeof(data));
}
#endif

/*****************************************************************************
                 Motion Status Req ( MCU -> Servo ) - 0x43 - #getallstatus,0*
******************************************************************************/
void ServoClass::MotionStatusReq(uint8_t id)
{
    static int Old_Pos[3] = {0,0,0};

	if(id == ID_Z) Set_ActPos(ID_Z, GetStepCurrentPos());
    else
    {
        uint32_t diff, OldAxisStatus = GetNowAxisStatus(id);

        byte data[] = { id, 0x43 };
        SendCmdToServo(id, data, sizeof(data));

        diff = OldAxisStatus ^ GetNowAxisStatus(id);
        uint32_t shift = 0x00000001;
        if(diff)
        {
            //Serial.print("MotionStatus Req ID ------------------> "); Serial.println(id);
            //Serial.print("Old AxisStatus : "); Serial.println(GetNowAxisStatus(id), HEX);
            //Serial.print("New AxisStatus : "); Serial.println(GetNowAxisStatus(id), HEX);

            for (int i = 0; i < 32; i++) {

                if( (shift << i) & diff )
                {
                    //Serial.print("Change bit : "); Serial.print(shift << i, HEX); Serial.print(", Value : "); Serial.println(((shift << i) & GetNowAxisStatus(id)) ? '1' : '0');

                    if(((shift << i) == FFLAG_SERVOON) && ((shift << i) & GetNowAxisStatus(id))) { MoveOrg(id); }
                    else if(((shift << i) == FFLAG_INPOSITION) && ((shift << i) & GetNowAxisStatus(id)))
                    {
                        //SendPositionToHost(id);
                        //Serial.print("Inposition Complete. @@ id : "); Serial.println(id);
                    }
                    else if((shift << i) == FFLAG_ORIGINRETOK)
                    {
                        //Serial.println("Clear Position-------------------------------------");
                        //ClearPosition(id);
                        //SendPositionToHost(id);
                    }
                }
            }

            //MotionStatusInfoDisp(id);
        }
    }

    /* Send to HOST Current Position */
    if(Get_ActPos(id) != Old_Pos[id]) { Old_Pos[id] = Get_ActPos(id); SendPositionToHost(id); }
}

/*****************************************************************************
                 Motion Status Check
******************************************************************************/
void ServoClass::MotorIdleCheck(uint8_t id)
{
    static int8_t Old_Status[3] = {-1,-1,-1};

    if( MotorIsIdle(id) != Old_Status[id] )
    {
        Old_Status[id] = MotorIsIdle(id);
        
        char status[30];
        memset(status, 0, sizeof(status));
        sprintf((char*)status, "%s%s", (id == ID_X) ? MOTOR_STATUS_X : (id == ID_Y) ? MOTOR_STATUS_Y : MOTOR_STATUS_Z , MotorIsIdle(id) ? ":idle" : ":busy");
        SendCmdToHost(status);

        //if(MotorIsIdle(id)) { Serial.print(id); Serial.println(" Motor Status *********************** I D L E **************************"); }
        //else { Serial.print(id); Serial.println(" Motor Status *********************** B U S Y **************************"); }
    }
}

void ServoClass::MotionStatusInfoDisp(uint8_t id, bool add)
{
    if(add)
    {
        Serial.print("------------------------------ ID : "); Serial.print(id); Serial.println(" ------------------------------");

        Serial.print("Get FFLAG ERRORALL        : "); Serial.println(Get_FFLAG_ERRORALL(id));
        Serial.print("Get FFLAG ORIGINRETURNING : "); Serial.println(Get_FFLAG_ORIGINRETURNING(id));
        Serial.print("Get FFLAG INPOSITION      : "); Serial.println(Get_FFLAG_INPOSITION(id));
        Serial.print("Get FFLAG SERVOON         : "); Serial.println(Get_FFLAG_SERVOON(id));
        Serial.print("Get FFLAG ALARMRESET      : "); Serial.println(Get_FFLAG_ERRORALL(id));
        Serial.print("Get FFLAG ORIGINRETOK     : "); Serial.println(Get_FFLAG_ORIGINRETOK(id));
        Serial.print("Get FFLAG MOTIONING       : "); Serial.println(Get_FFLAG_MOTIONING(id));
        Serial.print("Get FFLAG ERRORALL        : "); Serial.println(Get_FFLAG_ERRORALL(id));
        Serial.print("Get FFLAG ERRORALL        : "); Serial.println(Get_FFLAG_ERRORALL(id));

        Serial.print("Get FFLAG MOTIONDIR       : "); Serial.println((Get_FFLAG_MOTIONDIR(id) ? "Back" : "Forward"));

        Serial.print("Motor IDLE/BUSY           : "); Serial.println((MotorIsIdle(id) ? "IDLE" : "BUSY"));

        Serial.print("Motor Live/Dead           : "); Serial.println((GetMotorLive(id) == LIVE) ? "Live" : "Dead");

        Serial.println("--------------------------------------------------------------------");
    }

    if(Get_FFLAG_ERRORALL(id)) { Serial.println("Get FFLAG ERRORALL Occurred ....."); GetAlarmInfo(id); AlarmReset(id); }
    if(Get_FFLAG_ORIGINRETURNING(id)) { Serial.println("Get FFLAG ORIGINRETURNING Now ....."); }
    if(!Get_FFLAG_INPOSITION(id)) { Serial.println("Get_FFLAG_INPOSITION Required ....."); }
    if(!Get_FFLAG_SERVOON(id)) { Serial.println("Get FFLAG SERVOON Not Ready ....."); }
    if(Get_FFLAG_ALARMRESET(id)) { Serial.println("Get Get FFLAG ALARMRESET Now ....."); }
    if(!Get_FFLAG_ORIGINRETOK(id)) { Serial.println("Get FFLAG ORIGINRETOK Required ....."); }
    if(Get_FFLAG_MOTIONING(id)) { Serial.println("Get FFLAG MOTIONING Now ....."); }

    Serial.print("Cmd Pos : "); Serial.print(Get_CmdPos(id)); Serial.print(", Act Pos : "); Serial.print(Get_ActPos(id)); Serial.print(", Speed : "); Serial.println(Get_ActVel(id));

    if(Get_FFLAG_INPOSITION(id) && Get_FFLAG_SERVOON(id) && (Get_PosErrs(id) > POS_ERR_MAX)) { Serial.println("Position Error Occurred !!!!!!!!!!!!!!!!!....."); }
}

/*****************************************************************************
                 Cleer Postion ( MCU -> Servo ) - 0x56
******************************************************************************/
void ServoClass::ClearPosition(uint8_t id)
{
    byte data[] = { id, 0x56 };
    SendCmdToServo(id, data, sizeof(data));
}

//--------------------------------------------------------------------------------------------------------------------------
//                 Check Servo Status
//--------------------------------------------------------------------------------------------------------------------------
void ServoClass::CheckStatus(uint8_t force)
{
    static unsigned long lastchecktimes;

    if((force == FORCE_CHECK) || ((millis() - lastchecktimes) >= 300))
    {
        //SetDebugMode(0);

        for(uint8_t id = ID_X; id <= ID_Z; id++) if((GetMotorLive(id) == LIVE) || force) { MotionStatusReq(id); MotorIdleCheck(id); }

        //SetDebugMode(1);

        /* Over Distance Error Exception */
        for(uint8_t id = ID_X; id <= ID_Y; id++) 
        {
            if(GetMotorLive(id) != LIVE) continue;
            if((Get_PosErrs(id) < POS_ERR_MAX) && (Get_ActPos(id) > GetMaxDistance(id)) && Get_FFLAG_MOTIONING(id) && !Get_FFLAG_MOTIONDIR(id)) {
                
            MoveEmcStop(id); 
            Serial.print("Max Distance Over in Check Status ------------ ID : "); Serial.print(id); Serial.print(", Act Pos : "); Serial.print(Get_ActPos(id)); Serial.print(", Max : "); Serial.print(GetMaxDistance(id)); Serial.print(", PosErrs : "); Serial.println(Get_PosErrs(id));            }
        }

        lastchecktimes = millis();

#if 1
        static uint8_t oldval = 0, newval;
        {
            
            newval = digitalRead(DN_LIMIT_PIN);
            if( newval != oldval )
            {
                oldval = newval;

                if(HOME_S_DETECT) Serial.println("HOME Sensor OOOOO");
                else Serial.println("HOME Sensor XXXXX");            }
        }
#endif
    }
}

//--------------------------------------------------------------------------------------------------------------------------
//                 Send Cmd To HOST(PC) ( MCU -> HOST(PC) )
//--------------------------------------------------------------------------------------------------------------------------
void ServoClass::SendCmdToHost(char* data)
{
    static unsigned long last_time = 0;

    if((millis() - last_time) < HOST_DLY) { delay(HOST_DLY - (millis() - last_time)); }

    Serial1.print('#');
    Serial1.write(data);
    Serial1.print('*');

    last_time = millis();
}

char sendmsg[50];
void ServoClass::SendPositionToHost(uint8_t id)
{
    memset(sendmsg, 0, sizeof(sendmsg));

    int pos = Get_ActPos(id)/((id == ID_Z) ? 1 : SCALEFACTOR);
    sprintf((char*)sendmsg, "%s:%d", (id == ID_X) ? MOTOR_POS_X : (id == ID_Y) ? MOTOR_POS_Y : MOTOR_POS_Z , pos);//motorpos0:100000000
    //Serial.print("SendPositionToHost : "); Serial.print(sendmsg); Serial.print(", real : "); Serial.println(Get_ActPos(id));

    SendCmdToHost(sendmsg);
}

//--------------------------------------------------------------------------------------------------------------------------
//                 Send Cmd To Servo ( MCU -> Servo )
//--------------------------------------------------------------------------------------------------------------------------
void ServoClass::SendCmdToServo(uint8_t id, byte* _cmds, uint8_t len)
{
    if(id == ID_Z) return;

    byte cmds[len + 6];
    uint16_t crc;
    uint8_t waitcnt = WAIT_CNT, FrameType = *(_cmds + 1);

    static unsigned long last_time = 0;

    if((millis() - last_time) < SERVO_DLY) { delay(SERVO_DLY - (millis() - last_time)); }

    //if((id != ALLID) && (Servo.GetMotorLive(id) == DEAD)) { if(GetDebugMode()) { Serial.print("Send Cmd To Servo Motor Not Live Error --- ID : "); Serial.println(id); } return; }

    if((id != ALLID) && (Get_FFLAG_ERRORALL(id) || !Get_FFLAG_SERVOON(id)))// || Get_FFLAG_ORIGINRETURNING(id) || !Get_FFLAG_ORIGINRETOK(id))
    {
        if( (FAS_MoveStop <= FrameType) && (FrameType <= FAS_AllSingleAxisIncPos))
        {
            Serial.print("Unstable SendCmdToServo FrameType : "); Serial.println(FrameType, HEX);
            MotionStatusInfoDisp(id, 1);
            return;
        }
    }

    crc = CalcCRCbyAlgorithm(_cmds, (int)len);

    cmds[0] = 0xAA; cmds[1] = 0xCC; cmds[len + 4] = 0xAA; cmds[len + 5] = 0xEE;
    memcpy(&cmds[2], _cmds, len);
    memcpy(&cmds[len + 2], &crc, sizeof(uint16_t));

    Serial2.write(cmds, sizeof(cmds));

    /* Send Cmd Debug */
    if(GetDebugMode())
    {
        Serial.print("--> Send Cmd To Servo len is "); Serial.print(len); Serial.print(" :  ");
        for(uint8_t i=0; i<sizeof(cmds); i++ ) { if (cmds[i] < 0x10) { Serial.print("0"); } Serial.print(cmds[i], HEX); Serial.print(' ');}
        Serial.println();
    }

    if(id == ALLID) { delayMicroseconds(300); return; }

    /* Instant response received */
    while( ++waitcnt ) {  if(Serial2.available()) { break; } delayMicroseconds(20); }//1600u
    SerialEvent_From_Servo();

    if(!waitcnt) {
        Serial.print("waitcnt Error ---- "); Serial.println(waitcnt);
        if((FrameType == FAS_GetSlaveInfo)) Servo.SetMotorLive(id, 0);//Frame Type - Get Slave Info
    }

    last_time = millis();
}

//--------------------------------------------------------------------------------------------------------------------------
//                 Recv Data From Servo ( Servo -> MCU )
//--------------------------------------------------------------------------------------------------------------------------
void ServoClass::RecvDataHandlingFromServo(byte* _data, uint8_t len)
{
    byte* data = _data;
    uint8_t id;
    uint8_t FrameType;
    uint8_t CommError;

    if(GetDebugMode())// Motion Status Check exception
    {
        Serial.print("<-- Recvived Data From Servo  :  ");//Serial.println(__func__);
        for(uint8_t i=0; i<len; i++) { if (*(data+i) < 0x10) { Serial.print("0"); } Serial.print(*(data+i), HEX); Serial.print(' '); }
        Serial.println();
    }

    id = *(data + 2);
    FrameType = *(data + 3);
    CommError = *(data + 4);

    if(CommError) { Serial.print("Communication Error...... Maybe test return : "); Serial.println(CommError, HEX); return; }

    if((*data == 0xAA) && (*(data+1) == 0xCC) && (*(data+len-2) == 0xAA) && (*(data+len-1) == 0xEE))
    {
        if(FrameType == FAS_GetSlaveInfo) //Get Slave Info
        {
            SetMotorLive(id, *(data+5) == 0x32);
            Serial.print("Set Motor Live id : "); Serial.print(id); Serial.print(", Live : "); Serial.println(*(data+5) == 0x32);
        }
        else if(FrameType == FAS_GetParameter) { Serial.print("Get Parm : "); Serial.println(*((uint32_t*)(data + 5))); }
        else if(FrameType == FAS_ServoEnable) { Serial.println("Servo Success"); } // Servo On
        else if(FrameType == FAS_ServoAlarmReset) { Serial.println("Alarm Reset OK. Servo On"); ServoOn(id, 1);} // Alarm Reset
        else if(FrameType == FAS_GetAlarmType) { Serial.print("Alarm Type : "); Serial.println(*(data + 5)); } // GetAlarmType
        //else if(FrameType == FAS_MoveSingleAxisAbsPos) { Serial.println("Moving S Abs Rsp Check"); CheckStatus(id); } // Move Single Abs Response
        else if(FrameType == FAS_GetAllStatus) SaveGetAllStatus(id, data); //GetAllStatus
    }
    else
        Serial.println("Wrong Response Received...........");
}

#ifndef TEENSY_PWM
void updatePWM() {
#define FREQ    (1000000 / LED_FREQ)

    static bool state = false;
    state = !state;
    if (state) {
        digitalWrite(LED_PIN, HIGH);
        pwmTimer.update(FREQ - dutyCycle);
    } else {
        digitalWrite(LED_PIN, LOW);
        pwmTimer.update(FREQ - (LED_FREQ - dutyCycle));
    }
}
#endif

//--------------------------------------------------------------------------------------------------------------------------
//                 Recv Cmd From Host(PC) ( Host(PC) -> MCU )
//--------------------------------------------------------------------------------------------------------------------------
void ServoClass::RecvCmdFromPC(char* cmds)
{
    char* cmdptr = NULL;
    uint8_t id;
    int pos = 0;
    uint32_t speed = DEFAULT_SPEED;

    Serial.print("Recv Cmd From PC : "); Serial.println(cmds);

    if(!strncmp(cmds, SLAVEINFO, strlen(SLAVEINFO)))           /* 0x01 - #slaveinfo,x* */
    {
        cmdptr = cmds + strlen(SLAVEINFO);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! SLAVEINFO Wrong ID : ");Serial.println(id); return; }

            GetSlaveInfo(id);
        }
    }
    else if(!strncmp(cmds, SETPARM, strlen(SETPARM)))           /* 0x12 - #setparm,x,6,100000* */
    {
        cmdptr = cmds + strlen(SETPARM);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! SETPARM Wrong ID : ");Serial.println(id); return; }

            cmdptr = strchr(cmdptr, ',');
            if(cmdptr)
            {
                uint8_t num; int value;

                num = atoi(++cmdptr);

                cmdptr = strchr(cmdptr, ',');
                if(cmdptr)
                {
                    value = atoi(++cmdptr);
                    SetParameter(id, num, value);
                }
            }
        }
    }
    else if(!strncmp(cmds, GETPARM, strlen(GETPARM)))           /* 0x13 - #getparm,x,6* */
    {
        cmdptr = cmds + strlen(GETPARM);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! GETPARM Wrong ID : ");Serial.println(id); return; }

            cmdptr = strchr(cmdptr, ',');
            if(cmdptr)
            {
                uint8_t num;

                num = atoi(++cmdptr);
                GetParameter(id, num);
            }
        }
    }
    else if(!strncmp(cmds, SERVOON, strlen(SERVOON)))           /* 0x2A - #servoon,x* */
    {
        cmdptr = cmds + strlen(SERVOON);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! SERVOON Wrong ID : ");Serial.println(id); return; }

            ServoOn(id, 1);
        }
    }
    else if(!strncmp(cmds, SERVOOFF, strlen(SERVOOFF)))           /* 0x2A - #servooff,x* */
    {
        cmdptr = cmds + strlen(SERVOOFF);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! SERVOOFF Wrong ID : ");Serial.println(id); return; }

            ServoOn(id, 0);
        }
    }
    else if(!strncmp(cmds, ALARMRESET, strlen(ALARMRESET)))           /* 0x2B - #alarmreset,x* */
    {
        cmdptr = cmds + strlen(ALARMRESET);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! ALARMRESET Wrong ID : ");Serial.println(id); return; }

            AlarmReset(id);
        }
    }
    else if(!strncmp(cmds, GETALARM, strlen(GETALARM)))           /* 0x2E - #getalarm,x* */
    {
        cmdptr = cmds + strlen(GETALARM);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! GETALARM Wrong ID : ");Serial.println(id); return; }

            GetAlarmInfo(id);
        }
    }
    else if(!strncmp(cmds, MOVESTOP, strlen(MOVESTOP)))          /* 0x31 - #movestop,x* */
    {
        cmdptr = cmds + strlen(MOVESTOP);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! MOVESTOP Wrong ID : ");Serial.println(id); return; }

            MoveStop(id);
        }
    }
    else if(!strncmp(cmds, MOVEMCSTOP, strlen(MOVEMCSTOP)))        /* 0x32 - #moveemcstop,x* */
    {
        cmdptr = cmds + strlen(MOVEMCSTOP);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! MOVEMCSTOP Wrong ID : ");Serial.println(id); return; }

            MoveEmcStop(id);
        }
    }
    else if(!strncmp(cmds, MOVEORG, strlen(MOVEORG)))           /* 0x33 - #moveorg,x* */
    {
        cmdptr = cmds + strlen(MOVEORG);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! MOVEORG Wrong ID : ");Serial.println(id); return; }

            if(Get_FFLAG_MOTIONING(id)) { MoveEmcStop(id); delay(EMCSTOP_DLY); }

            MoveOrg(id);
        }
    }
    // #movesabs,x,30000,100000*
    else if (!strncmp(cmds, MOVE_S_ABS, strlen(MOVE_S_ABS))) {
        char* cmdptr = cmds + strlen(MOVE_S_ABS) + strlen(",");
        
        char axis = *cmdptr;
        int id;

        if (axis == 'x') id = ID_X;
        else if (axis == 'y') id = ID_Y;
        else if (axis == 'z') id = ID_Z;
        else {
            Serial.print("! MOVESABS Wrong ID : "); Serial.println(axis);
            return;
        }

        // 위치값 파싱
        cmdptr = strchr(cmdptr, ',');
        if (!cmdptr) {
            Serial.println("! MOVESABS Missing position");
            return;
        }
        int pos = atoi(++cmdptr);
        if(id != ID_Z) pos *= SCALEFACTOR;

        // 속도값 있는지 확인
        int speed = GetMoveSpeed(id); // 기본 속도
        char* speedptr = strchr(cmdptr, ',');

        if (speedptr) {
            speed = atoi(++speedptr);
        }

        MoveSingleAbs(id, pos, speed);
    }
    // #movetabs,x,10000,100000,y,10000,100000,z,10000,20000*
    // #movetabs,x,10000,y,10000*
    else if(!strncmp(cmds, MOVE_T_ABS, strlen(MOVE_T_ABS)))
    {
        char* token = strtok(cmds + strlen(MOVE_T_ABS) + strlen(","), ",");

        while (token) {
            char axis = token[0];
            int id;

            if (axis == 'x') id = ID_X;
            else if (axis == 'y') id = ID_Y;
            else if (axis == 'z') id = ID_Z;
            else {
                Serial.print("! MOVE T ABS Wrong ID : "); Serial.println(axis);
                token = strtok(NULL, ",");
                continue;
            }

            // 위치값 추출
            token = strtok(NULL, ",");
            if (!token) {
                Serial.println("! Missing position value.");
                break;
            }
            int pos = atoi(token);
            if(id != ID_Z) pos *= SCALEFACTOR;

            // 다음 토큰이 속도인지, 아니면 다음 축인지 판단
            char* next = strtok(NULL, ",");
            int speed = GetMoveSpeed(id); // 기본 속도

            if (next && (next[0] != 'x' && next[0] != 'y' && next[0] != 'z')) {
                speed = atoi(next);
                next = strtok(NULL, ",");
            }

            MoveSingleAbs(id, pos, speed);
            token = next; // 다음 축으로 이동
        }
    }
    else if(!strncmp(cmds, MOVEJOG, strlen(MOVEJOG)))          /* 0x37 - #movejog,x,1* 1:+, 0:- */
    {
        uint8_t direction = 1;//0:-Jog 1:+Jog

        cmdptr = cmds + strlen(MOVEJOG);
        cmdptr = strchr(cmdptr, ',');

        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! MOVEORG Wrong ID : ");Serial.println(id); return; }

            speed = GetMoveSpeed(id);

            cmdptr = strchr(cmdptr, ',');
            if(cmdptr)
            {
                direction = atoi(++cmdptr);

                if(((Get_PosErrs(id) < POS_ERR_MAX)) && (Get_ActPos(id) > GetMaxDistance(id)) && direction) { // direction - 0:-   1:+
                    
                    Serial.print("Max Distance Over in MOVEJOG ------------ ID : "); Serial.print(id); 
                    Serial.print(", Act Pos : "); Serial.print(Get_ActPos(id));
                    Serial.print(", Max : "); Serial.print(GetMaxDistance(id));
                    Serial.print(", PosErrs : "); Serial.println(Get_PosErrs(id));
                    return;            
                }

                if(Get_FFLAG_MOTIONING(id)) { MoveEmcStop(id); delay(EMCSTOP_DLY); }

                cmdptr = strchr(cmdptr, ',');
                if(cmdptr) speed = atoi(++cmdptr);

                MoveJog(id, direction, speed);
            }
        }
    }
    else if(!strncmp(cmds, MOVEALLSTOP, strlen(MOVEALLSTOP)))          /* 0x3B - #moveallstop* */
    {
        MoveAllStop();
    }
    else if(!strncmp(cmds, MOVEALLEMCSTOP, strlen(MOVEALLEMCSTOP)))          /* 0x3C - #moveallemcstop* */
    {
        MoveAllEmcStop();
    }
    else if(!strncmp(cmds, MOVEALLORG, strlen(MOVEALLORG)))          /* 0x3D - #moveallorg* */
    {
        if(Get_FFLAG_MOTIONING(ID_X) || Get_FFLAG_MOTIONING(ID_Y) || Get_FFLAG_MOTIONING(ID_Z)) { MoveAllEmcStop(); delay(EMCSTOP_DLY); }

        MoveAllOrg();
    }
    // #moveallsabs,10000*  #moveallsinc,10000,10000*
    else if(!strncmp(cmds, MOVEALL_S_ABS, strlen(MOVEALL_S_ABS)) || !strncmp(cmds, MOVEALL_S_INC, strlen(MOVEALL_S_INC)))          /* 0x3E 0x3F */
    {
        cmdptr = strchr(cmds, ',');
        if(cmdptr)
        {
            pos = atoi(++cmdptr);

            cmdptr = strchr(cmdptr, ',');
            if(cmdptr) speed = atoi(++cmdptr);

            if(Get_FFLAG_MOTIONING(ID_X) || Get_FFLAG_MOTIONING(ID_Y) || Get_FFLAG_MOTIONING(ID_Z)) { MoveAllEmcStop(); delay(EMCSTOP_DLY);}

            if(!strncmp(cmds, MOVEALL_S_ABS, strlen(MOVEALL_S_ABS)))
            {
                for(uint8_t id = ID_X; id <= ID_Z; id++) { if(GetMotorLive(id) == LIVE) { speed = GetMoveSpeed(id); MoveSingleAbs(id, pos, speed); }}
                //MoveAllSingleAbs(pos, speed);
            }
            else
            {
                for(uint8_t id = ID_X; id <= ID_Z; id++) { if(GetMotorLive(id) == LIVE) { speed = GetMoveSpeed(id); MoveSingleInc(id, pos, speed); }}
                //MoveAllSingleInc(pos, speed);
            }
        }
    }
    else if(!strncmp(cmds, GETALLSTATUS, strlen(GETALLSTATUS)))      /* 0x43 - #getallstatus,x* */
    {
        cmdptr = cmds + strlen(GETALLSTATUS);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! MOVEORG Wrong ID : ");Serial.println(id); return; }

            MotionStatusReq(id);

            MotionStatusInfoDisp(id, true);
        }
    }
    else if(!strncmp(cmds, CLEAR_POS, strlen(CLEAR_POS)))      /* 0x56 - #clearpos,x* */
    {
        cmdptr = cmds + strlen(CLEAR_POS);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            id = *(++cmdptr) - 'x';
            if(MOTOR_ALL_ID_CHECK) { Serial.print("! MOVEORG Wrong ID : ");Serial.println(id); return; }

            ClearPosition(id);
        }
    }
    else if(!strncmp(cmds, TRY_CONNECT, strlen(TRY_CONNECT)))
    {
        Serial.println("TRY CONNECT Received...");

        memset(sendmsg, 0, sizeof(sendmsg));

        sprintf((char*)sendmsg, "%s:%d%d%d", TRY_CONNECT , GetMotorLive(ID_X) == LIVE, GetMotorLive(ID_Y) == LIVE, GetMotorLive(ID_Z) == LIVE);//TryConnection:110
        Serial.print("sendmsg : "); Serial.println(sendmsg);
        SendCmdToHost(sendmsg);

        delay(1);
        SendPositionToHost(ID_X); delay(1);
        SendPositionToHost(ID_Y); delay(1);
        SendPositionToHost(ID_Z);
    }
    else if(!strncmp(cmds, MOTOR_LIVE_TEST, strlen(MOTOR_LIVE_TEST)))
    {
        Serial.println("MOTOR LIVE TEST Received...");

        memset(sendmsg, 0, sizeof(sendmsg));

        sprintf((char*)sendmsg, "%s:%d,%d,%d", MOTOR_LIVE_TEST , GetMotorLive(ID_X) == LIVE, GetMotorLive(ID_Y) == LIVE, GetMotorLive(ID_Z) == LIVE);
        SendCmdToHost(sendmsg);
    }
    else if(!strncmp(cmds, MOTOR_STATUS_REQ, strlen(MOTOR_STATUS_REQ))) /* #servostatusreq*     #servostatusreq,0* */
    {
        Serial.println("Servo Status Req Received...");

        memset(sendmsg, 0, sizeof(sendmsg));

        sprintf((char*)sendmsg, "%s:%d,%d,%d", MOTOR_STATUS_REQ , MotorIsIdle(ID_X) == 1, MotorIsIdle(ID_Y) == 1, MotorIsIdle(ID_Z) == 1);

        SendCmdToHost(sendmsg);

        Serial.println(sendmsg);
    }
    else if(!strncmp(cmds, MOTOR_POS_REQ, strlen(MOTOR_POS_REQ))) /* #motorposreq*  */
    {
        Serial.println("Servo Status Req Received...");

        memset(sendmsg, 0, sizeof(sendmsg));

        sprintf((char*)sendmsg, "%s:%d,%d,%d", MOTOR_POS_REQ , Get_ActPos(ID_X) / SCALEFACTOR, Get_ActPos(ID_Y) / SCALEFACTOR, Get_ActPos(ID_Z));

        SendCmdToHost(sendmsg);

        Serial.println(sendmsg);
    }
    else if(!strncmp(cmds, LEDBR, strlen(LEDBR)))
    {
        Serial.println("LED Brightness...");

        cmdptr = cmds + strlen(GETALLSTATUS);
        cmdptr = strchr(cmdptr, ',');
        if(cmdptr)
        {
            float brightness = atof(++cmdptr);//  /100.0f;
            Serial.print("LED Brightness Pre : "); Serial.println(brightness);

#ifdef TEENSY_PWM
            extern Teensy_PWM* PWM_Instance;

            brightness /= 100.0f;
            if(0)//if(brightness == 100.0f)
            {
                brightness = 99.999;
                PWM_Instance->setPWM(LED_PIN, LED_FREQ, brightness);
                Serial.print("LED Bright After : "); Serial.println(brightness);
            }
            else
                PWM_Instance->setPWM(LED_PIN, LED_FREQ, brightness);
#else
            dutyCycle = round(map(brightness, 0, PWM_MAX_VAL, 0, LED_FREQ));

            if(dutyCycle == LED_FREQ)
            {
                pwmTimer.end();
                digitalWrite(LED_PIN, HIGH);
            }
            else if(dutyCycle == 0)
            {
                pwmTimer.end();
                digitalWrite(LED_PIN, LOW);
            }
            else
            {
                pwmTimer.begin(updatePWM, dutyCycle);
            }

            Serial.print("LED duty After : "); Serial.println(dutyCycle);
#endif
        }
    }
}