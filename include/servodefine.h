#ifndef _SERVODEFINE_H_
#define _SERVODEFINE_H_

//------------------------------------------------------------------
//                 Frame Type
//------------------------------------------------------------------

#define FAS_GetSlaveInfo 			0x01	//1
#define FAS_GetMotorInfo			0x05	//5
#define FAS_SaveAllParameters		0x10	//16
#define FAS_GetRomParameter			0x11	//17
#define FAS_SetParameter			0x12	//18
#define FAS_GetParameter			0x13	//(19)

#define FAS_SetIOOutput				0x20	//(32)
#define FAS_SetIOInput				0x21	//(33)
#define FAS_GetIOInput				0x22	//(34)
#define FAS_GetIOOutput				0x23	//(35)
#define FAS_SetIOAssignMap			0x24	//(36)
#define FAS_GetIOAssignMap			0x25	//(37)
#define FAS_IOAssignMapReadROM		0x26	//(38)
#define FAS_TriggerOutput_RunA		0x27	//(39)
#define FAS_TriggerOutput_Status	0x28	//(40)
#define FAS_ServoEnable				0x2A	//(42)
#define FAS_ServoAlarmReset			0x2B	//(43)
#define FAS_GetAlarmType			0x2E	//(46)

#define FAS_MoveStop				0x31	//(49)
#define FAS_EmergencyStop			0x32	//(50)
#define FAS_MoveOriginSingleAxis	0x33	//(51)
#define FAS_MoveSingleAxisAbsPos	0x34	//(52)
#define FAS_MoveSingleAxisIncPos	0x35	//(53)
#define FAS_MoveToLimit				0x36	//(54)
#define FAS_MoveVelocity			0x37	//(55)
#define FAS_PositionAbsOverride		0x38	//(56)
#define FAS_PositionIncOverride		0x39	//(57)
#define FAS_VelocityOverride		0x3A	//(58)
#define FAS_AllMoveStop				0x3B	//(59)
#define FAS_AllEmergencyStop		0x3C	//(60)
#define FAS_AllMoveOriginSingleAxis	0x3D	//(61)
#define FAS_AllSingleAxisAbsPos		0x3E	//(62)
#define FAS_AllSingleAxisIncPos		0x3F	//(63)
#define FAS_MoveSingleAxisAbsPosEx	0x80	//(128)
#define FAS_MoveSingleAxisIncPosEx	0x81	//(129)
#define FAS_MoveVelocityEx			0x82	//(130)

#define FAS_GetAxisStatus			0x40	//(64)
#define FAS_GetIOAxisStatus			0x41	//(65)
#define FAS_GetMotionStatus			0x42	//(66)
#define FAS_GetAllStatus			0x43	//(67)

#define FAS_SetCommandPos			0x50	//(80)
#define FAS_GetCommandPos			0x51	//(81)
#define FAS_SetActualPos			0x52	//(82)
#define FAS_GetActualPos			0x53	//(83)
#define FAS_GetPosError				0x54	//(84)
#define FAS_GetActualVel			0x55	//(85)
#define FAS_ClearPosition			0x56	//(86)
#define FAS_MovePause				0x58	//(88)
#define FAS_PosTableReadItem		0x60	//(96)

#define FAS_PosTableWriteItem		0x61	//(97)
#define FAS_PosTableReadROM			0x62	//(98)
#define FAS_PosTableWriteROM		0x63	//(99)
#define FAS_PosTableRunItem			0x64	//(100)
#define FAS_PosTableReadOneItem		0x6A	//(106)
#define FAS_PosTableWriteOneItem	0x6B	//(107)
#define FAS_MovePush				0x78	//(120)
#define FAS_GetPushStatus			0x79	//(121)

//------------------------------------------------------------------
//                 Axis Status - Status Flag -> Command 0x40
//------------------------------------------------------------------
#define	FFLAG_ERRORALL			 0x00000001
#define	FFLAG_HWPOSILMT			 0x00000002
#define	FFLAG_HWNEGALMT			 0x00000004
#define	FFLAG_SWPOGILMT			 0x00000008
#define	FFLAG_SWNEGALMT			 0x00000010
#define	FFLAG_RESERVED0			 0x00000020
#define	FFLAG_RESERVED1			 0x00000040
#define	FFLAG_ERRPOSOVERFLOW	 0x00000080
#define	FFLAG_ERROVERCURRENT	 0x00000100
#define	FFLAG_ERROVERSPEED		 0x00000200
#define	FFLAG_ERRPOSTRACKING	 0x00000400
#define	FFLAG_ERROVERLOAD		 0x00000800
#define	FFLAG_ERROVERHEAT		 0x00001000
#define	FFLAG_ERRBACKEMF		 0x00002000
#define	FFLAG_ERRMOTORPOWER		 0x00004000
#define	FFLAG_ERRINPOSITION		 0x00008000
#define	FFLAG_EMGSTOP			 0x00010000
#define	FFLAG_SLOWSTOP			 0x00020000
#define	FFLAG_ORIGINRETURNING	 0x00040000
#define	FFLAG_INPOSITION		 0x00080000
#define	FFLAG_SERVOON			 0x00100000
#define	FFLAG_ALARMRESET		 0x00200000
#define	FFLAG_PTSTOPPED			 0x00400000
#define	FFLAG_ORIGINSENSOR		 0x00800000
#define	FFLAG_ZPULSE			 0x01000000
#define	FFLAG_ORIGINRETOK		 0x02000000
#define	FFLAG_MOTIONDIR			 0x04000000
#define	FFLAG_MOTIONING			 0x08000000
#define	FFLAG_MOTIONPAUSE		 0x10000000
#define	FFLAG_MOTIONACCEL		 0x20000000
#define	FFLAG_MOTIONDECEL		 0x40000000
#define	FFLAG_MOTIONCONST		 0x80000000

//------------------------------------------------------------------
//                 Parameters Defines.
//------------------------------------------------------------------
typedef enum
{
	SERVO_MINI_PULSEPERREVOLUTION = 0,
	SERVO_MINI_AXISMAXSPEED,
	SERVO_MINI_AXISSTARTSPEED,
	SERVO_MINI_AXISACCTIME,
	SERVO_MINI_AXISDECTIME,

	SERVO_MINI_SPEEDOVERRIDE,           // 5
	SERVO_MINI_JOGHIGHSPEED,
	SERVO_MINI_JOGLOWSPEED,
	SERVO_MINI_JOGACCDECTIME,

	SERVO_MINI_SERVOALARMLOGIC,
	SERVO_MINI_SERVOONLOGIC,            // 10
	SERVO_MINI_SERVORESETLOGIC,

	SERVO_MINI_SWLMTPLUSVALUE,
	SERVO_MINI_SWLMTMINUSVALUE,
	SERVO_MINI_SOFTLMTSTOPMETHOD,
	SERVO_MINI_HARDLMTSTOPMETHOD,       // 15 - 0xFF
	SERVO_MINI_LIMITSENSORLOGIC,		// 16 - 0x10

	SERVO_MINI_ORGSPEED,
	SERVO_MINI_ORGSEARCHSPEED,
	SERVO_MINI_ORGACCDECTIME,
	SERVO_MINI_ORGMETHOD,               // 20 - 0x14
	SERVO_MINI_ORGDIR,
	SERVO_MINI_ORGOFFSET,
	SERVO_MINI_ORGPOSITIONSET,
	SERVO_MINI_ORGSENSORLOGIC,

	SERVO_MINI_POSITIONLOOPGAIN,        // 25 - 0x19
	SERVO_MINI_INPOSITIONVALUE,
	SERVO_MINI_POSTRACKINGLIMIT,
	SERVO_MINI_MOTIONDIR,

	SERVO_MINI_LIMITSENSORDIR,
	SERVO_MINI_ORGTORQUERATIO,          // 30 - 0x1E

	SERVO_MINI_POSERROVERFLOWLIMIT,
	SERVO_MINI_POSVALUECOUNTINGMETHOD,

    SERVO_MINI_SERVO_ON_METHOD,
    SERVO_MINI_BREAK_DELAY_TIME,
    SERVO_MINI_RESERVED,                // 35 - 0x23
    SERVO_MINI_RUN_CURRENT,
    SERVO_MINI_STOP_CURRENT,
    SERVO_MINI_BOOST_CURRENT,

	MAX_SERVO_MINI_PARAM

} FM_EZISERVO_MINI_PARAM;

#endif