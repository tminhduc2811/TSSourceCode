#include "stm32f4xx.h"
#include <math.h>

/* Define */
#define				pi				(double)3.14159265358979
/* Define Type */
typedef enum
{
	Check_NOK = 0,
	Check_OK,
}Check_Status;

typedef enum{
	Veh_NoneError 				= 0,
	Veh_ReadMessage_Err,        //Over flow Rx buffer
	Veh_ReadGxGLLMessage_Err,		//
	Veh_ReadGxGGAMessage_Err,
	Veh_GxGLLCheckSum_Err,
	Veh_GxGGACheckSum_Err,
	Veh_InvalidGxGLLMessage_Err,
	Veh_CommandMessageCheckSum_Err,
	Veh_IMUWrongMessage_Err,
}Vehicle_Error;

typedef struct Status{
	Check_Status 				IMU_FirstSetAngle;							// (*) IMU first received angle after turning on
	Check_Status				Veh_Sample_Time;								// (*) Vehicle sample time finished
	Check_Status				Veh_Send_Data;									// (*) Vehicle send data time finished
	Check_Status				GPS_Coordinate_Reveived;				// (*) 
	Check_Status				GPS_ValidGPS;
	Check_Status				Veh_SendData_Flag;
	Check_Status				Veh_Calib_Flag;
	Check_Status				Srf05_TimeOut_Flag;
	Check_Status				Veh_Timer_Finish;
	Check_Status				Veh_Timer_Start;
	Check_Status				IMU_Calib_Finish;
	Check_Status				Veh_Send_Parameters;
	Check_Status				Veh_Auto_Flag;
	Check_Status				GPS_Start_Receive_PathCor;
	Check_Status				GPS_SelfUpdatePosition_Flag;
	Check_Status				GPS_FirstGetPosition;
	Check_Status				Veh_Object_Avoid_Flag;
}Status;

typedef	enum{
	None = 0,
	Vehicle_Config,
	Sample_Time,
	Send_Time,
	IMU_Config,
	Soft_Reset,
	Manual_Config,
	Auto_Config,
	Path_Plan,
	Flash_Save,
	KeyBoard_Control,
}Command_State;

typedef enum{
	NSave = 0,
	PID_Parameter, 
	GPS_Coordinates,
}Internal_Flash;

typedef enum{
	None_Mode = 0,
	Auto_Mode,
	Manual_Mode,
	Calib_Mode,
	KeyBoard_Mode,
	Soft_Reset_Mode,
}Mode;

typedef enum{
	Invalid=0,
	RTK_Fixed	=	4,
	RTK_Float	=	5,
	Dead	=	6,
}GPS_Quality;

typedef enum{
	NoSensor = 0,
	Sensor_Mid,
	Sensor_Left,
	Sensor_Right,
	Sensor_Back,
}Srf05_Sensor_Number;

typedef struct Error{
	uint8_t			Error_Buffer[20];
	uint8_t			Error_Index;
}Error;

typedef struct Time{
	uint32_t		Sample_Time;
	uint32_t    Time_Sample_Count;
	uint32_t    Send_Time;
	uint32_t		Time_Send_Count;
	uint32_t		Srf05_Sample_Time;
	uint32_t		Srf05_Count;
	double			T;
}Time;

typedef	struct  DCMotor{
	/* PID parameters */
	double		Kp;
	double 		Ki;
	double		Kd;
	/* Encoder parameters */
	uint16_t  Enc;
	uint16_t 	PreEnc;
	uint8_t		OverFlow;
	/* Input and Output of PID controller */
	double		Set_Vel;
	double 		Current_Vel;	
	uint8_t		Change_State;
	double		*SampleTime;
	double		Pre_PID;
	double    Pre2_Error;
	double    Pre_Error;
	double		PID_Out;
}DCMotor;


typedef struct trimf{
	double a1;
	double a2;
	double a3;
}trimf;

typedef struct trapf{
	double h1;
	double h2;
	double h3;
	double h4;
}trapf;

typedef struct IMU{
	/* Current Angle and Set Angle */
	double		Angle;
	double 		Set_Angle;
	double		Pre_Angle;
	/* Fuzzy input and output */
	double		Fuzzy_Out;
	double		Fuzzy_Error;
	double 		Fuzzy_Error_dot;
	/* Variables Ke, Kedot and Ku */
	double		Ke;
	double  	Kedot;
	double 		Ku;
}IMU;


typedef struct GPS{
	/* Robot state */
	double 		CorX;
	double 		CorY;
	double 		Pre_CorX;
	double 		Pre_CorY;
	double		dx;
	double		dy;
	int				NewDataAvailable;
	int				Times;
	IMU				*Angle;
	int				P_Yaw_Index;
	double 		efa;
	/* Stanley control variables */
	double		Thetae;
	double  	Thetad;
	double		Delta_Angle;
	double		K;
	double		Step;
	double 		Robot_Velocity; // (Vr + Vl) / 2
	double		dmin;
	/* Goal radius reached */
	Check_Status			Goal_Flag;
	/* GPS NEO M8P input coordinates */
	double		Latitude;
	double    Longitude;
	int 			NbOfWayPoints;
	int				GPS_Quality;
	int				NbOfP;
	int 			Cor_Index;
	/* Buffer read and write data */
	char			TempBuffer[50][30];
	double    Path_X[20];
	double		Path_Y[20];
	double		P_X[3000];
	double    P_Y[3000];
	double		P_Yaw[3000];
	/* Error GPS code */
	Vehicle_Error GPS_Error;
}GPS;


typedef	struct Vehicle
{
	Mode								Mode;
	int 								LengthOfCommands;
	int									SendData_Ind;
	char								ManualCtrlKey;
	double    					Max_Velocity;
	double							Manual_Velocity;
	double							Manual_Angle;
	/* Calibration variables */
	uint16_t						Distance;
	uint8_t							TotalDistance;
	/* Srf05 sensor variables */
	uint8_t						  Srf05_Selected_Sensor;
	Vehicle_Error				Veh_Error;
	double							Sensor_Angle;
}Vehicle;

typedef	struct Message
{
	char    Message[50][30];
}Message;

typedef struct FlashMemory{
	uint32_t 		 WriteIn32bBuffer[100];
	uint8_t			 ReadOutBuffer[500];
	uint8_t			 WriteInBuffer[500];
	char  			 Message[20][30];
	int				 	 Length;
}FlashMemory;

#define  					K1 															1/(2*pi)
#define						K2															4/pi
#define						K3															1
#define						Wheel_Radius 										0.085
#define						IMU_AngleIndex									17
#define						FLASH_ProgramType_Byte					VoltageRange_1
#define						FLASH_ProgramType_HalfWord			VoltageRange_2
#define						FLASH_ProgramType_Word					VoltageRange_3
#define						FLASH_ProgramType_DoubleWord		VoltageRange_4
#define						FLASH_PIDPara_BaseAddr 					0x08060000	  //(4 KBytes) (0x08060000 - 0x08060FFF)
#define						FLASH_FuzPara_BaseAddr					0x08061000		//(4 Kbytes) (0x08061000 - 0x08061FFF)
#define						FLASH_GPSPara_BaseAddr					0x08040000		// (128 KBytes) 
/* Export variables */
extern char												TempBuffer[2][30];
extern Status											VehStt;
extern DCMotor 										M1,M2;
extern Vehicle										Veh;
extern trimf 											In1_NS,In1_ZE,In1_PS,In2_ZE;
extern trapf 											In1_NB,In1_PB,In2_NE,In2_PO;
extern double 										NB,NM,NS,ZE,PS,PM,PB;
extern GPS   											GPS_NEO;
extern IMU												Mag;
extern Message  									U2,U6;
extern Time												Timer;
extern FlashMemory 								Flash;
extern Error											Veh_Error;
/*--------Export Function------------------------- */
void 	 					LedTest(void);     																				// Use 4 leds on board to test code
double 					ToDegree(double rad); 																		// Convert rad to degree
double 					ToRadian(double degree); 																	// Convert degree to rad
uint8_t 				ToChar(double value, uint8_t *pBuffer,int NbAfDot); 			// Convert double value to char array
uint8_t					ToHex(uint8_t input);
double					ToRPM(double vel);
double					fix(double value);
double 					Pi_To_Pi(double angle);
int							LengthOfLine(uint8_t *inputmessage);
int							LengthOfIMULine(uint8_t *inputmessage);
double 					Degree_To_Degree(double angle);
void						Convert_Double_Array(double *pInputArray, int n);
/*------------ Error update --=========-----------*/
void						Error_ResetIndex(Error *perror);	
void						Error_AppendError(Error *perror, Vehicle_Error Error_Code);
/*------------ Vehicle Status update -------------*/
void						Status_ParametersInit(Status *pstt);
void						Status_UpdateStatus(Check_Status *pstt, Check_Status stt);
Check_Status		Status_CheckStatus(Check_Status *pstt);
/*------------ Timer Function --------------------*/
void						Time_ParametersInit(Time *ptime, uint32_t Sample, uint32_t Send);
void						Time_SampleTimeUpdate(Time *ptime, uint32_t Sample);
void						Time_GetSampleTime(Time *ptime);
void						Time_SendTimeUpdate(Time *ptime, uint32_t TSend);
/*------------ Vehicle status functions ----------*/
void						Veh_ParametersInit(Vehicle *pveh);
void						Veh_GetManualCtrlKey(Vehicle *pveh, char key);
void						Veh_UpdateVehicleFromKey(Vehicle *pveh);
void						Veh_CheckStateChange(DCMotor *ipid, uint8_t State);
void						Veh_UpdateMaxVelocity(Vehicle *pveh, double MaxVelocity);
void						Veh_GetSensorAngle(Vehicle *pveh, double Angle);
Vehicle_Error		Veh_GetCommandMessage(uint8_t *inputmessage, char result[50][30]);
/*------------ PID Function ----------------------*/
void						PID_UpdateEnc(DCMotor *ipid, uint16_t PulseCount);
void						PID_SavePIDParaToFlash(FlashMemory *pflash, DCMotor *M1, DCMotor *M2);
void 						PID_Compute(DCMotor *ipid);
void 						PID_ParametersInitial(DCMotor *ipid);
void					  PID_UpdateSetVel(DCMotor *ipid, double SetVal);
void 						PID_ParametersUpdate(DCMotor *ipid, double Kp, double Ki, double Kd);
void						PID_ResetEncoder(DCMotor *ipid);
void						PID_ResetPID(DCMotor *ipid);
/* -------Send and Receive data function------------ */
void 						GetMessageInfo(char *inputmessage, char result[50][30], char character);
double 					GetValueFromString(char *value);
int 						Readline(uint8_t *inputmessage, uint8_t *outputmessage);
uint8_t			 		LRCCalculate(uint8_t *pBuffer, int length);
Check_Status  	IsValidData(char input);
Check_Status 		IsCorrectMessage(uint8_t *inputmessage, int length, uint8_t byte1, uint8_t byte2);
Check_Status		StringHeaderCompare(char *s1, char header[]);
Command_State		GetNbOfReceiveHeader(char *input);
int							FeedBack(uint8_t *outputmessage, char inputstring[20]);
Check_Status		IsDataTransferCompleted(void);
/*--------Stanley functions and GPS --------------*/
void						GPS_ParametersInit(GPS *pgps);
void 						GPS_StanleyControl(GPS *pgps, double SampleTime, double M1Velocity, double M2Velocity);
double					GPS_LLToDegree(double LL);
void 						GPS_LatLonToUTM(GPS *pgps);  // Get 2 values of lat lon and update UTM coordiante to Corx and Cory
void	 					GPS_GetLatFromString(GPS *pgps, char *inputmessage);
void						GPS_GetLonFromString(GPS *pgps, char *inputmessage);
void						GPS_ClearPathBuffer(GPS *pgps);
void 						GPS_ClearPathCorBuffer(GPS *pgps);
void						GPS_UpdatePathYaw(GPS *pgps);
void						GPS_UpdatePathCoordinate(GPS *pgps, uint8_t *inputmessage);
void						GPS_UpdatePathCoordinateV2(GPS *pgps, uint8_t *inputmessage);
void						GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash);
void						GPS_UpdateParameters(GPS *pgps, double K, double Step);
void						GPS_UpdateCoordinateXY(GPS *pgps, double Cor_X, double Cor_Y);
void						GPS_PathPlanning(GPS *pgps, float Step);
Check_Status		GPS_HeaderCompare(uint8_t *s1, char Header[5]);
Vehicle_Error		GPS_GetLLQMessage(GPS *pgps, uint8_t *inputmessage,char result[50][30]);
/*--------Fuzzy control-------------------*/
void						Fuzzy_ParametersInit(void);
double 					Trapf(trapf *ptrapf, double x);
double 					Trimf(trimf *ptrimf, double x);
void						Trimf_Update(trimf *ptrimf, double a1, double a2, double a3);
void						Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4);
void						Defuzzification_Max_Min(IMU *pimu);

/*--------IMU functions ---------*/
void						IMU_ParametesInit(IMU *pimu);
void						IMU_UpdateSetAngle(IMU *pimu, double ComAngle);
void						IMU_UpdatePreAngle(IMU *pimu);
void						IMU_UpdateFuzzyInput(IMU *pimu, double *pSampleTime);
void						IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku);
Vehicle_Error	 	IMU_GetValueFromMessage(IMU *pimu, uint8_t *inputmessage);

/*-------- Flash Memory Embedded functions --------*/
void 						WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr);
void 						ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr);
void						EraseMemory(uint32_t Flash_Sector);































