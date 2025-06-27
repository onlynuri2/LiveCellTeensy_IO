#ifndef _SERVOFUNC_H_
#define _SERVOFUNC_H_

#include <Arduino.h>
#include "servodefine.h"

#define LED_PIN     33
#define LED_FREQ    1000
#define PWM_MAX_VAL 10000 // GUI Max Input
//#define TEENSY_PWM

#define UP_PLUSE_PIN		31
#define DN_PLUSE_PIN        32
#define DN_LIMIT_PIN        24

#define HOME_S_DETECT       (digitalRead(DN_LIMIT_PIN) == HIGH)
#define HOME_S_DISTANCE     1700

#define MOTOR_ALL_ID_CHECK  (id != ID_X) && (id != ID_Y) && (id != ID_Z)
#define ID_X    0
#define ID_Y    1
#define ID_Z    2
#define ALLID   99

#define LIVE    1
#define DEAD    0
#define NONE    -1

#define X_MAX_DIST  30000 * SCALEFACTOR
#define Y_MAX_DIST  35000 * SCALEFACTOR
#define Z_MAX_DIST  14000

#define MAX_SPEED_X       200000
#define DEFAULT_SPEED_X   100000

#define MAX_SPEED_Y       150000
#define DEFAULT_SPEED_Y   66000

#define MAX_SPEED_Z         10000
#define Min_SPEED_Z         1000
#define MAX_PERIOD_Z        1000
#define MIN_PERIOD_Z        100
#define INIT_SPEED_Z        6000
#define DEFAULT_SPEED_Z     MAX_PERIOD_Z - ((INIT_SPEED_Z - Min_SPEED_Z) * (MAX_PERIOD_Z - MIN_PERIOD_Z)) / (MAX_SPEED_Z - Min_SPEED_Z)
#define GET_SPEED_Z(speed)    MAX_PERIOD_Z - ((speed - Min_SPEED_Z) * (MAX_PERIOD_Z - MIN_PERIOD_Z)) / (MAX_SPEED_Z - Min_SPEED_Z)

#define DEFAULT_SPEED       DEFAULT_SPEED_Z

#define SCALEFACTOR       10

#define POS_ERR_MAX     2000

#define FORCE_CHECK 0xFF

#define WAIT_CNT    100

#define SERVO_DLY   2 //mills
#define HOST_DLY    2 //mills
#define EMCSTOP_DLY    10 //mills
/*************************** MCU -> Servo Command ***************************/
#define SLAVEINFO       "slaveinfo"         /* 0x01 */

#define SETPARM         "setparm"           /* 0x12 */
#define GETPARM         "getparm"           /* 0x13 */

#define SERVOON         "servoon"           /* 0x2A */
#define SERVOOFF        "servooff"          /* 0x2A */

#define ALARMRESET      "alarmreset"        /* 0x2B */
#define GETALARM        "getalarm"          /* 0x2E */

#define MOVESTOP        "movestop"          /* 0x31 */
#define MOVEMCSTOP      "moveemcstop"       /* 0x32 */
#define MOVEORG         "moveorg"           /* 0x33 */
#define MOVE_S_ABS      "movesabs"          /* 0x34 */
#define MOVE_S_INC      "movesinc"          /* 0x35 */

#define MOVE_T_ABS      "movetabs"          /* Mark Define */

#define MOVEJOG         "movejog"           /* 0x37 */

#define MOVEALLSTOP     "moveallstop"       /* 0x3B */
#define MOVEALLEMCSTOP  "moveallemcstop"    /* 0x3C */
#define MOVEALLORG      "moveallorg"        /* 0x3D */
#define MOVEALL_S_ABS   "moveallsabs"       /* 0x3E */
#define MOVEALL_S_INC   "moveallsinc"       /* 0x3E */

#define GETALLSTATUS    "getallstatus"      /* 0x43 */

#define CLEAR_POS       "clearpos"     /* 0x56 */

#define MOVEALL_S_INC   "moveallsinc"       /* 0x3E */

/*******************************************************************************/
#define TRY_CONNECT     (char*)"TryConnection"
#define MCU_LIVE_TEST   (char*)"mculive"
#define MOTOR_LIVE_TEST   (char*)"motorlive"
#define LEDBR           (char*)"ledbrightness"
/*************************** MCU -> HOST(PC) Command ***************************/
#define MOTOR_POS_X       "motorposx"
#define MOTOR_POS_Y       "motorposy"
#define MOTOR_POS_Z       "motorposz"

#define MOTOR_STATUS_REQ  (char*)"motorstatusreq"
#define MOTOR_POS_REQ  (char*)"motorposreq"

#define MOTOR_STATUS        (char*)"motorstatus"
#define MOTOR_STATUS_X        (char*)"motorstatusx"
#define MOTOR_STATUS_Y        (char*)"motorstatusy"
#define MOTOR_STATUS_Z        (char*)"motorstatusz"
#define MOTOR_STATUS_IDLE  (char*)"motorstatusidle"
#define MOTOR_STATUS_BUSY  (char*)"motorstatusbusy"

typedef struct {

    uint32_t Input;
    uint32_t Output;
    uint32_t AxisStatus;
    int     CmdPos;
    int     ActPos;
    uint32_t PosErr;
    uint32_t ActVel;
    uint32_t PTItem;

} MOTIONStatus;

class ServoClass {

    private:

        MOTIONStatus MotionStatus[3];
        uint8_t DebugMode;
        int8_t MotorLive[3];

        int MaxDistance[3];
        uint32_t MaxSpeed[3];

        int StepTargetPluse;
        int StepCurrentPluse;

    public:

        ServoClass() : MaxDistance{ X_MAX_DIST, Y_MAX_DIST, Z_MAX_DIST }, MaxSpeed{ DEFAULT_SPEED_X, DEFAULT_SPEED_Y, DEFAULT_SPEED_Z }
        { 
            memset(MotionStatus, 0, sizeof(MotionStatus));
            memset(MotorLive, -1, sizeof(MotorLive));
            DebugMode = 1;
            StepTargetPluse = StepCurrentPluse = -1;
        }

        inline int GetMaxDistance(uint8_t id) { return MaxDistance[id]; }
        inline int GetMoveSpeed(uint8_t id) { return MaxSpeed[id]; }

        inline void SetDebugMode(uint8_t _debug) { DebugMode = _debug; }
        inline uint8_t GetDebugMode() { return DebugMode; }

        inline void SetMotorLive(uint8_t id, int8_t _set) { MotorLive[id] = _set; }
        inline int8_t GetMotorLive(uint8_t id) { return MotorLive[id]; }

        inline uint32_t GetNowAxisStatus(uint8_t id) { return MotionStatus[id].AxisStatus; }

        
        void SendCmdToHost(char* data);
        void SendPositionToHost(uint8_t id);
        void SendCmdToServo(uint8_t id, byte* _cmds, uint8_t len);
        void RecvDataHandlingFromServo(byte* _data, uint8_t len);
        void RecvCmdFromPC(char* cmds);

        void GetSlaveInfo(uint8_t id);                                                 /* 0x01 */

        void SetParameter(uint8_t id, uint8_t num, int value);                      /* 0x12 */
        void GetParameter(uint8_t id, uint8_t num);                                 /* 0x13*/

        void ServoOn(uint8_t id, uint8_t OnOff = 1);                                /* 0x2A */
        void AlarmReset(uint8_t id);                                                /* 0x2B */
        void GetAlarmInfo(uint8_t id);                                              /* 0x2E */
        void MoveStop(uint8_t id);                                                  /* 0x31 */
        void MoveEmcStop(uint8_t id);                                               /* 0x32 */
        void MoveOrg(uint8_t id);                                                   /* 0x33 */
        void MoveSingleAbs(uint8_t id, int pos, uint32_t speed = DEFAULT_SPEED);    /* 0x34 */
        void MoveSingleInc(uint8_t id, int pos, uint32_t speed = DEFAULT_SPEED);    /* 0x35 */
        void MoveJog(uint8_t id, uint8_t direction, uint32_t speed);                /* 0x37 */
        void MoveSingleAbsOver(uint8_t id, int pos);                                /* 0x38 */
        void MoveSingleIncOver(uint8_t id, int pos);                                /* 0x39 */

        void MoveAllStop();                                                         /* 0x3B */
        void MoveAllEmcStop();                                                      /* 0x3C */
        void MoveAllOrg();                                                          /* 0x3D */
        //void MoveAllSingleAbs(int pos, uint32_t speed = DEFAULT_SPEED);             /* 0x3E */
        //void MoveAllSingleInc(int pos, uint32_t speed = DEFAULT_SPEED);                 

        void MotionStatusReq(uint8_t id);
        void MotionStatusInfoDisp(uint8_t id, bool add = false);
        void MotorIdleCheck(uint8_t id);

        void ClearPosition(uint8_t id);                                             /* 0x56 */

        void CheckStatus(uint8_t force = 0);

        void ServoPowerInit();

        inline void SaveGetAllStatus(uint8_t id, byte* data) { //Cmd 0x43
            memcpy(&MotionStatus[id], (data + 5), 32);
        }

        inline MOTIONStatus* GetMotionStatusInfo(uint8_t id) { return &MotionStatus[id]; }

        inline uint8_t MotorIsIdle(uint8_t id) {
            if(id == ID_Z) return (StepTargetPluse == StepCurrentPluse);
            else
            return ( GetMotorLive(id) && !Get_FFLAG_ERRORALL(id) && !Get_FFLAG_ORIGINRETURNING(id) && Get_FFLAG_INPOSITION(id) && Get_FFLAG_SERVOON(id) && !Get_FFLAG_ALARMRESET(id) && Get_FFLAG_ORIGINRETOK(id) && !Get_FFLAG_MOTIONING(id) ); 
        }
        inline uint8_t MotorIsBusy(uint8_t id) { return !MotorIsIdle(id); }

        /* Status Flag Info - 0x43 FAS_GetAllStatus */
        inline uint32_t Get_Input(uint8_t id) { return MotionStatus[id].Input; }
        inline uint32_t Get_Output(uint8_t id) { return MotionStatus[id].Output; }
        inline uint32_t Get_AxisStatus(uint8_t id) { return MotionStatus[id].AxisStatus; }
        inline int Get_CmdPos(uint8_t id) { return MotionStatus[id].CmdPos; }
        inline int Get_ActPos(uint8_t id) { return MotionStatus[id].ActPos; }
        inline void Set_ActPos(uint8_t id, int pos) { MotionStatus[id].ActPos = pos; }
        inline uint32_t Get_PosErrs(uint8_t id) { return MotionStatus[id].PosErr; }
        inline uint32_t Get_ActVel(uint8_t id) { return MotionStatus[id].ActVel; }
        inline uint32_t Get_PTItem(uint8_t id) { return MotionStatus[id].PTItem; }

        inline bool Get_FFLAG_ERRORALL(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERRORALL ); }         // 여러 에러중 하나 이상의 에러가 발생함.
        inline bool Get_FFLAG_HWPOSILMT(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_HWPOSILMT ); }         // +방향 리미트 센서가 ON 이 된경우
        inline bool Get_FFLAG_HWNEGALMT(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_HWNEGALMT ); }         // -방향 리미트 센서가 ON 이 된경우
        inline bool Get_FFLAG_SWPOGILMT(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_SWPOGILMT ); }         // +방향 프로그램 리미트를 초과한 경우
        inline bool Get_FFLAG_SWNEGALMT(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_SWNEGALMT ); }         // -방향 프로그램 리미트를 초과한 경우
        inline bool Get_FFLAG_ERRPOSOVERFLOW(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERRPOSOVERFLOW ); }         // 위치명령 완료후 위치오차가 ‘Pos Error OverflowLimit’값보다 크게 발생한 경우
        inline bool Get_FFLAG_ERROVERCURRENT(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERROVERCURRENT ); }         // 모터 구동소자에 과전류 이상 Alarm 발생.
        inline bool Get_FFLAG_ERROVERSPEED(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERROVERSPEED ); }         // 모터의 속도가 3000[rpm]을 초과한 Alarm 발생.
        inline bool Get_FFLAG_ERRPOSTRACKING(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERRPOSTRACKING ); }         // 위치명령중 위치오차가 ‘Pos Tracking Limit’값보다 크게 발생한 경우
        inline bool Get_FFLAG_ERROVERLOAD(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERROVERLOAD ); }         // 모터의 최대토크를 초과하는 부하가 5초 이상 또는 10바퀴 이상의 거리 동안 가해지는 Alarm 발생.
        inline bool Get_FFLAG_ERROVERHEAT(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERROVERHEAT ); }         // 드라이브의 내부온도가 55°C 를 초과하는 Alarm 발생.
        inline bool Get_FFLAG_ERRBACKEMF(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERRBACKEMF ); }         // 모터의 역기전력 전압이 70V 를 초과하는 Alarm 발생
        inline bool Get_FFLAG_ERRMOTORPOWER(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERRMOTORPOWER ); }         // 모터 전압 이상 Alarm 발생
        inline bool Get_FFLAG_ERRINPOSITION(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ERRINPOSITION ); }         // Inposition 이상 Alarm 발생.
        inline bool Get_FFLAG_EMGSTOP(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_EMGSTOP ); }         // 모터가 비상 정지 상태임.
        inline bool Get_FFLAG_SLOWSTOP(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_SLOWSTOP ); }         // 모터가 일반 정지 상태임.
        inline bool Get_FFLAG_ORIGINRETURNING(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ORIGINRETURNING ); }         // 원점복귀 운전중임.
        inline bool Get_FFLAG_INPOSITION(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_INPOSITION ); }         // Inposition 이 완료된 상태임.
        inline bool Get_FFLAG_SERVOON(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_SERVOON ); }         // 모터가 Servo ON 상태임
        inline bool Get_FFLAG_ALARMRESET(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ALARMRESET ); }         // AlarmReset 명령이 실시된 상태임.
        inline bool Get_FFLAG_PTSTOPED(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_PTSTOPPED ); }         // 포지션테이블 운전이 종료된 상태임
        inline bool Get_FFLAG_ORIGINSENSOR(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ORIGINSENSOR ); }         // 원점센서가 ON 되어 있는 상태임.
        inline bool Get_FFLAG_ZPULSE(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ZPULSE ); }         // 모터 운전중 엔코더의 Z 상 신호 위치에 도달한 경우
        inline bool Get_FFLAG_ORIGINRETOK(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_ORIGINRETOK ); }         // 원점복귀 운전이 완료된 상황임.
        inline bool Get_FFLAG_MOTIONDIR(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_MOTIONDIR ); }         // 모터의 운전 방향 (+방향:OFF, -방향:ON)
        inline bool Get_FFLAG_MOTIONING(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_MOTIONING ); }         // 모터가 현재 운전중임.
        inline bool Get_FFLAG_MOTIONPAUSE(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_MOTIONPAUSE ); }         // 운전중 Pause 명령으로 정지 상태임.
        inline bool Get_FFLAG_MOTIONACCEL(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_MOTIONACCEL ); }         // 가속구간의 운전중임.
        inline bool Get_FFLAG_MOTIONDECEL(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_MOTIONDECEL ); }         // 감속구간의 운전중임
        inline bool Get_FFLAG_MOTIONCONST(uint8_t id) { return ( MotionStatus[id].AxisStatus & FFLAG_MOTIONCONST ); }         // 가/감속 구간이 아닌 정속도 운전중인 상태임

        void StepMotorInit();
        void StepMotorHome();
        void MoveStepMotor(uint8_t dir, int pos, int speed = DEFAULT_SPEED_Z);
        inline void SetStepTaretPos(int pluse) { StepTargetPluse = pluse; }
        inline int GetStepTaretPos() { return StepTargetPluse; }
        inline void SetStepCurrentPos(int pluse) { StepCurrentPluse = pluse; }
        inline int GetStepCurrentPos() { return StepCurrentPluse; }
};

extern ServoClass Servo;

#endif