#include "functions.h"

/* Global variables */
GPIO_InitTypeDef				 F_GPIO_Struct;
/*----- Stanley Variables -----*/

/*----- Robot Parameter -------*/
Status						VehStt;
DCMotor 					M1, M2, Ang;
GPS   						GPS_NEO;
IMU								Mag;
Message  					U2, U6;
Time							Timer;
FlashMemory 			Flash;
Vehicle						Veh;
double 						NB,NM,NS,ZE,PS,PM,PB;
trimf 						In1_NS,In1_ZE,In1_PS,In2_ZE;
trapf 						In1_NB,In1_PB,In2_NE,In2_PO;
Error							Veh_Error;
char							TempBuffer[2][30];
/*----- Error functions -----------------------------------*/
void	Error_ResetIndex(Error *perror)
{
	perror->Error_Index = 0;
}

void	Error_AppendError(Error *perror, Vehicle_Error Code)
{
	perror->Error_Buffer[perror->Error_Index++] = Code;
}
/*----- Init, and function to config vehicle status -------*/
void	Status_ParametersInit(Status *pstt)
{
	pstt->GPS_Coordinate_Reveived = Check_NOK;
	pstt->GPS_ValidGPS						= Check_NOK;
	pstt->IMU_FirstSetAngle				= Check_NOK;
	pstt->Veh_Sample_Time					= Check_NOK;
	pstt->Veh_Send_Data						= Check_NOK;
	pstt->Veh_SendData_Flag				= Check_OK;
	pstt->Srf05_TimeOut_Flag			= Check_NOK;
	pstt->Veh_Timer_Finish				= Check_NOK;
	pstt->IMU_Calib_Finish				= Check_NOK;
	pstt->Veh_Send_Parameters			= Check_NOK;
	pstt->Veh_Auto_Flag						= Check_NOK;
	pstt->GPS_Start_Receive_PathCor	 = Check_NOK;
	pstt->GPS_SelfUpdatePosition_Flag = Check_NOK;
	pstt->GPS_FirstGetPosition    = Check_NOK;
	pstt->Veh_Object_Avoid_Flag = Check_NOK;
}

void	Status_UpdateStatus(Check_Status *pstt, Check_Status stt)
{
	*(pstt) = stt;
}

Check_Status	Status_CheckStatus(Check_Status *pstt)
{
	return *pstt;
}

/*----- Functions -----*/
/** @brief  : Led test
**	@agr    : void
**	@retval : None
**/
void LedTest(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	F_GPIO_Struct.GPIO_Mode 				= GPIO_Mode_OUT;
	F_GPIO_Struct.GPIO_Pin					= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	F_GPIO_Struct.GPIO_OType				= GPIO_OType_PP;
	F_GPIO_Struct.GPIO_PuPd					= GPIO_PuPd_NOPULL;
	F_GPIO_Struct.GPIO_Speed				= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&F_GPIO_Struct);
}

/** @brief  : Function compute PID value 
**	@agr    : void
**	@retval : None
**/
void PID_Compute(DCMotor *ipid)
{
	ipid->SampleTime = &Timer.T;
	ipid->PID_Out = ipid->Pre_PID + ipid->Kp * (ipid->Set_Vel - ipid->Current_Vel) + 0.5 * ipid->Ki * *(ipid->SampleTime) * ((ipid->Set_Vel - ipid->Current_Vel) + ipid->Pre_Error) + (ipid->Kd / *(ipid->SampleTime)) * ((ipid->Set_Vel - ipid->Current_Vel) - 2 * ipid->Pre_Error + ipid->Pre2_Error);
	ipid->Pre2_Error = ipid->Pre_Error;
	ipid->Pre_Error = ipid->Set_Vel - ipid->Current_Vel;
	if(ipid->PID_Out < 0)
		ipid->PID_Out = (double)0;
	if(ipid->PID_Out > 100)
		ipid->PID_Out = (double)100;
	ipid->Pre_PID = ipid->PID_Out;
}

/** @brief  : First initial PID parameters
**	@agr    : void
**	@retval : None
**/
void PID_ParametersInitial(DCMotor *ipid)
{
	ipid->Pre2_Error = 0;
	ipid->Pre_Error = 0;
	ipid->Pre_PID = 0;
	ipid->PID_Out = 0;
	ipid->Set_Vel = 0;
	ipid->Current_Vel = 0;
	ipid->Enc = 0;
	ipid->PreEnc = 0;
	ipid->OverFlow = 0;
	ipid->Change_State = 1;
}

/** @brief  : Set velocity update for motor
**	@agr    : void
**	@retval : None
**/
void PID_UpdateSetVel(DCMotor *ipid, double SetVal)
{
	ipid->Set_Vel = SetVal;
}

/** @brief  : PID update parameters function
**	@agr    : void
**	@retval : None
**/
void PID_ParametersUpdate(DCMotor *ipid, double Kp, double Ki, double Kd)
{
	ipid->Kp = Kp;
	ipid->Ki = Ki;
	ipid->Kd = Kd;
}

/** @brief  : PID update encoder counter
**	@agr    : void
**	@retval : None
**/
void	PID_UpdateEnc(DCMotor *ipid, uint16_t PulseCount)
{
	ipid->Enc = PulseCount;
}

/** @brief  : PID reset encoder
**	@agr    : void
**	@retval : None
**/
void	PID_ResetEncoder(DCMotor *ipid)
{
	ipid->PreEnc = ipid->Enc;
	ipid->OverFlow = 1;
}

/** @brief  : PID reset PID
**	@agr    : void
**	@retval : None
**/
void	PID_ResetPID(DCMotor *ipid)
{
	ipid->Pre_PID = 0;
	ipid->Pre_Error = 0;
	ipid->Pre2_Error = 0;
}

/** @brief  : PID Save parameters to interflash memory
**	@agr    : void
**	@retval : None
**/
void	PID_SavePIDParaToFlash(FlashMemory *pflash, DCMotor *M1, DCMotor *M2)
{
	pflash->Length = 0;
	pflash->Length += ToChar(M1->Kp,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M1->Ki,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M1->Kd,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M2->Kp,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M2->Ki,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	pflash->Length += ToChar(M2->Kd,&pflash->WriteInBuffer[Flash.Length],6);
	pflash->WriteInBuffer[pflash->Length++] = 0x0D;
	pflash->WriteInBuffer[pflash->Length++] = 0x0A;
	EraseMemory(FLASH_Sector_7);
	WriteToFlash(pflash,FLASH_Sector_7,FLASH_PIDPara_BaseAddr);
}

/** @brief  : Convert rad to degree
**  @agr    : input rad value
**  @retval : degree value
**/
double ToDegree(double rad)
{
	double result;
	result = (rad * 180) / pi;
	return result;
}

/** @brief  : Convert degree to rad
**  @agr    : input degree value
**  @retval : rad value
**/
double ToRadian(double degree)
{
	double result;
	result = (degree * pi) / 180;
	return result;
}

/** @brief  : Calculate length of a line (end with 0x0D, 0x0A)
**  @agr    : Input buffer
**  @retval : Length
**/
int	LengthOfLine(uint8_t *inputmessage)
{
	int length = 0;
	while((inputmessage[length] != 0x0D) && (inputmessage[length + 1] != 0x0A))
	{
		length++;
	}
	return length;
}

int	LengthOfIMULine(uint8_t *inputmessage)
{
	int length = 0;
	while(inputmessage[length] != 0x0D)
	{
		length++;
	}
	return length;
}
/* ----------------------- Timer functions -----------------------------------*/
void	Time_ParametersInit(Time *ptime, uint32_t Sample, uint32_t Send)
{
	ptime->Time_Sample_Count = 0;
	ptime->Time_Send_Count = 0;
	ptime->Srf05_Count	= 0;
	ptime->Sample_Time = Sample;
	ptime->Send_Time = Send;
	ptime->Srf05_Sample_Time = 20000;
}

void	Time_SampleTimeUpdate(Time *ptime, uint32_t Sample)
{
	ptime->Sample_Time = Sample;
}

void	Time_GetSampleTime(Time *ptime)
{
	ptime->T					 = ptime->Sample_Time * pow(10,-6);
}

void	Time_SendTimeUpdate(Time *ptime, uint32_t TSend)
{
	ptime->Send_Time = TSend * 1000;
}
/*------------------------ Vehicle Status Function ----------------------------*/
void	Veh_ParametersInit(Vehicle *pveh)
{
	pveh->Max_Velocity = ToRPM(1);
	pveh->ManualCtrlKey = 0;
	pveh->Manual_Angle = 0;
	pveh->Manual_Velocity = 0;
	pveh->Mode = KeyBoard_Mode;
	pveh->LengthOfCommands = 0;
	pveh->SendData_Ind = 0;
	pveh->Distance = 0;
	pveh->TotalDistance = 0;
	pveh->Srf05_Selected_Sensor = 1;
	pveh->Veh_Error = Veh_NoneError;
	pveh->Sensor_Angle = 0;
}

void	Veh_UpdateVehicleFromKey(Vehicle *pveh)
{
	if(pveh->ManualCtrlKey == 'W')
	{
		IMU_UpdateSetAngle(&Mag,0);
		pveh->Manual_Velocity += 0.2 * pveh->Max_Velocity;
	}
	else if(pveh->ManualCtrlKey == 'S')
	{
		IMU_UpdateSetAngle(&Mag,0);
		pveh->Manual_Velocity -= 0.2 * pveh->Max_Velocity;
	}
	else if(pveh->ManualCtrlKey == 'D')
	{
		pveh->Manual_Angle += 30;
		if(pveh->Manual_Angle > 180) pveh->Manual_Angle -= 360;
		IMU_UpdateSetAngle(&Mag,pveh->Manual_Angle);
	}
	else if(pveh->ManualCtrlKey == 'A')
	{
		pveh->Manual_Angle -= 30;
		if(pveh->Manual_Angle < -180) pveh->Manual_Angle += 360;
		IMU_UpdateSetAngle(&Mag,pveh->Manual_Angle);
	}
	if(pveh->Manual_Velocity > pveh->Max_Velocity) pveh->Manual_Velocity = pveh->Max_Velocity;
	else if(pveh->Manual_Velocity < 0) pveh->Manual_Velocity = 0;
	pveh->ManualCtrlKey = 0;
}

void	Veh_UpdateMaxVelocity(Vehicle *pveh, double MaxVelocity)
{
	pveh->Max_Velocity = MaxVelocity;
}

void	Veh_GetSensorAngle(Vehicle *pveh, double Angle)
{
	pveh->Sensor_Angle = Angle;
}

Vehicle_Error	Veh_GetCommandMessage(uint8_t *inputmessage, char result[50][30])
{
	int length = LengthOfLine(inputmessage);
	GetMessageInfo((char*)inputmessage,result,',');
	if(IsCorrectMessage((uint8_t*)&inputmessage[1],length - 3,inputmessage[length - 2],inputmessage[length - 1]))
		return Veh_NoneError;
	else
		return Veh_CommandMessageCheckSum_Err;
}

void	Veh_CheckStateChange(DCMotor *ipid, uint8_t State)
{
	if(ipid->Change_State != State)
	{
		PID_ResetPID(ipid);
		ipid->Change_State = State;
	}
}


/* ----------------------- GPS function ---------------------------------------*/
/** @brief  : Round value
**  @agr    : input
**  @retval : Return fix value
**/
double fix(double value)
{
	double result;
	int temp;
	temp = (int)value;
	result = (double)temp;
	return result;
}


/** @brief  : Seperating each info of message by ','
**  @agr    : input message, result buffer
**  @retval : None
**/
void GetMessageInfo(char *inputmessage, char result[50][30], char character)
{
	int col = 0, row = 0, index = 0;
	for(int i = 0; i < 50; i++)
	{
		for(int j = 0; j < 30; j++)
		{
			result[i][j] = 0;
		}
	}
	
	while((inputmessage[index] != 0x0D) && (inputmessage[index + 1] != 0x0A) && (inputmessage[index] != 0) && (index < 1000))
	{
		if(inputmessage[index] != character)
		{
			result[row][col] = inputmessage[index];
			col++;
		}
		else 
		{
			row++;
			col = 0;
		}
		index++;
	}
}



/** @brief  : Get double value from string or array
**  @agr    : Input string or array
**  @retval : Double output value
**/
double GetValueFromString(char *value)
{
	double p1 = 0,p2 = 0, h = 1, result;
	int row = 0, col = 0, index = 0, len1, len2, sign = 1;
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 30; j++)
		{
			TempBuffer[i][j] = 0;
		}
	}
	if(value[0] == '-') 
	{
		index++;
		sign = -1;
	}
	else if(value[0] == ' ')
	{
		index++;
		sign = 1;
	}
	while(value[index] != 0)
	{
		if(value[index] != '.')
		{
			TempBuffer[row][col] = value[index];
			col++;
		}
		else
		{
			row++;
			len1 = col;
			col = 0;
		}
		index++;
	}
	if(row == 0)
	{
		len1 = col;
		for(int i = len1 - 1; i >= 0; i--)
		{
			p1 += ((uint8_t)TempBuffer[0][i] - 48) * h;
			h *= 10;
		}
		p2 = 0;
	}
	else
	{
		for(int i = len1 - 1; i >= 0; i--)
		{
			p1 += ((uint8_t)TempBuffer[0][i] - 48) * h;
			h *= 10;
		}
		len2 = col;
		h = 0.1;
		for(int i = 0; i < len2; i++)
		{
			p2 += ((uint8_t)TempBuffer[1][i] - 48) * h;
			h *= 0.1;
		}
	}
	result = sign*(p1 + p2);
	return result;
}

/** @brief  : Convert value to char array
**  @agr    : Input value, result buffer
**  @retval : None 
**/
Check_Status IsDouble(double value)
{
	if ((value - (uint32_t)value) != (double)0)
	{
		return Check_OK;
	}
	else return Check_NOK;
}
/** @brief  : Convert value to char array
**  @agr    : Input value, result buffer
**  @retval : None 
**/
uint8_t ToChar(double value, uint8_t *pBuffer, int NbAfDot)
{
	uint32_t BefD;
	double AftD;
	uint8_t buffer[20], index = 0, temp1, strleng = 0, len = 0;
	if(value < 0.0)
	{
		value = -value;
		pBuffer[0] = (uint8_t)'-';
		index++;
		strleng++;
	}
	BefD = (uint32_t)value;
	AftD = value - (double)BefD;
	if (BefD == 0) 
	{
		pBuffer[index] = (uint8_t)'0';
		index++;
		strleng++;
	}
	else
	{
		while(BefD != 0)
		{
			temp1 = BefD % 10;
			BefD /= 10;
			buffer[len] = temp1 + 48;
			len++;
		}
		strleng = index + len;
		// take the befD value
		for (int i = 0; i < strleng; i++)
		{
			len--;
			pBuffer[index + i] = buffer[len];
		}
	}
	if(IsDouble(AftD))
	{
		pBuffer[strleng] = (uint8_t)'.';
		strleng++;
		for (int i = 0; i < NbAfDot; i++)
		{
			AftD *= 10;
			pBuffer[strleng + i] = (uint8_t)AftD + 48;
			AftD = AftD - (uint8_t)AftD;
		}
		strleng += NbAfDot;
	}
	return strleng;
}

/** @brief  : Is valid data
**  @agr    : Input message
**  @retval : None 
**/
Check_Status IsValidData(char input)
{
	if(input == 'A') return Check_OK;
	else if(input == 'V') return Check_NOK;
	return Check_NOK;
}

/** @brief  : Readline function
**  @agr    : input and output buffer
**  @retval : Length of receive message
**/
int Readline(uint8_t *inputmessage, uint8_t *outputmessage)
{
	int i = 0, j = 0;
	while(inputmessage[i] != 0)
	{
		if(((inputmessage[i] == 0x0D) && ((inputmessage[i + 1] == 0x0A))))
		{
			break;
		}
		else
		{
			outputmessage[j] = inputmessage[i];
			j++;
		}
		i++;
	}
	return j;
}

/** @brief  : LRC calculate
**  @agr    : Input string array and its lenght
**  @retval : Result
**/
uint8_t LRCCalculate(uint8_t *pBuffer, int length)
{
	unsigned int result = 0;
	for(int i = 0; i < length; i++)
	{
		result ^= pBuffer[i];
	}
	return result;
}

/** @brief  : Convert decimal value to hexadecimal
**  @agr    : Input value
**  @retval : Result
**/
uint8_t ToHex(uint8_t input)
{
	if(input < 10)
		input += 48;
	else
		input += 55;
	return input;
}

/** @brief  : Check sum and return OK or Not OK
**  @agr    : Input message, 4 higher bits and 4 lower bits of byte
**  @retval : Check status
**/
Check_Status IsCorrectMessage(uint8_t *inputmessage, int length, uint8_t byte1, uint8_t byte2)
{
	uint8_t CheckSum, c1, c2;
	CheckSum = LRCCalculate(inputmessage,length);
	c1 = (CheckSum & 0xF0) >> 4;
	c2 = CheckSum & 0x0F;
	c1 = ToHex(c1);
	c2 = ToHex(c2);
	if((c1 == byte1) && (c2 == byte2))
		return Check_OK;
	else
		return Check_NOK;
}

/** @brief  : Compare 2 input string
**  @agr    : Input message
**  @retval : 1 - equal, 0 - not equal
**/
Check_Status StringHeaderCompare(char *s1, char header[])
{
	int i = 0;
	while(s1[i] != 0)
	{
		if(s1[i] != header[i]) return Check_NOK;
		i++;
	}
	return Check_OK;
}

/** @brief  : Get number of receive message
**  @agr    : Input message
**  @retval : None 
**/
Command_State		GetNbOfReceiveHeader(char *input)
{
	if(StringHeaderCompare(input,"$VEHCF"))
		return Vehicle_Config;
	else if(StringHeaderCompare(input,"$TSAMP"))
		return Sample_Time;
	else if(StringHeaderCompare(input,"$TSEND"))
		return Send_Time;
	else if(StringHeaderCompare(input,"$IMUCF"))
		return IMU_Config;
	else if(StringHeaderCompare(input,"$SFRST"))
		return Soft_Reset;
	else if(StringHeaderCompare(input,"$MACON"))
		return Manual_Config;
	else if(StringHeaderCompare(input,"$AUCON"))
		return Auto_Config;
	else if(StringHeaderCompare(input,"$VPLAN"))
		return Path_Plan; 	
	else if(StringHeaderCompare(input,"$FSAVE"))
		return Flash_Save;
	else if(StringHeaderCompare(input,"$KCTRL"))
		return KeyBoard_Control;
	else return None;
}

/** @brief  : Check if Data transfer finish or not
**  @agr    : None
**  @retval : Status
**/
Check_Status	IsDataTransferCompleted(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6))
	{
		return Check_OK;
	}
	else
		return Check_NOK;
}

/** @brief  : Feed back message
**  @agr    : Result and status
**  @retval : lenght 
**/
int FeedBack(uint8_t *outputmessage, char inputstring[20])
{
	int i = 0;
	while(inputstring[i] != 0)
	{
		outputmessage[i] = inputstring[i];
		i++;
	}
	outputmessage[i++]  = 0x0D;
	outputmessage[i++]  = 0x0A;
	return i;
}

void	Convert_Double_Array(double *pInputArray, int n)
{
	double temp;
	for(int i = 0; i < (n/2); i++)
	{
		temp = pInputArray[i];
		pInputArray[i] = pInputArray[n - 1 - i];
		pInputArray[n - 1 - i] = temp;
	}
}

/*-------------------- Stanley Function ------------------*/
/** @brief  : ToRPM
**  @agr    : Input velocity in m/s from panel
**  @retval : RPM value
**/
double ToRPM(double vel)
{
	double result;
	result = (vel / (2 * pi * Wheel_Radius)) * 60;
	return result;
}


/** @brief  : Pi to Pi
**  @agr    : input angle
**  @retval : double value
**/
double Pi_To_Pi(double angle)
{
	double result;
	if(angle > pi)
	{
		result = angle - 2 * pi;
	}
	else if (angle < -pi)
	{
		result = angle + 2 * pi;
	}
	else
	{
		result = angle;
	}
	return result;
}

double Degree_To_Degree(double angle)
{
	if(angle > 180)
		angle = angle - 360;
	else if(angle < -180)
		angle = angle + 360;
	else angle = angle;
	return angle;
}
/** @brief  : Convert lat lon coordinate into UTM
**  @agr    : input lat and lon values from GNGLL message GLONASS Lat Lon
**  @retval : Result buffer x ,y
**/
void GPS_LatLonToUTM(GPS *pgps)
{
	double la, lo, lat, lon, sa, sb, e2, e2cuadrada, c, Huso, S, deltaS, a, epsilon, nu, v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm, xx, yy;
	la = pgps->Latitude;
	lo = pgps->Longitude;
	sa = 6378137.00000;
	sb = 6356752.314245;
	e2 = pow(pow(sa,2) - pow(sb,2),0.5) / sb;
	e2cuadrada = pow(e2,2);
	c = pow(sa,2) / sb;
	lat = la * (pi / 180);
	lon = lo * (pi / 180);
	Huso = fix((lo / 6) + 31);
	S = ((Huso * 6) - 183);
	deltaS = lon - (S * (pi / 180));
	a = cos(lat) * sin(deltaS);
	epsilon = 0.5 * log((1 + a) / (1 - a));
	nu = atan(tan(lat) / cos(deltaS)) - lat;
	v = (c / pow((1 + (e2cuadrada * pow(cos(lat),2))),0.5)) * 0.9996;
	ta = (e2cuadrada / 2) * pow(epsilon,2) * pow(cos(lat),2);
	a1 = sin(2 * lat);
	a2 = a1 * pow(cos(lat),2);
	j2 = lat + (a1 / 2);
	j4 = ((3 * j2) + a2) / 4;
	j6 = ((5 * j4) + (a2 * pow(cos(lat),2))) / 3;
	alfa = ((double)3 / (double)4) * e2cuadrada;
	beta = ((double)5 / (double)3) * pow(alfa,2);
	gama = ((double)35 / (double)27) * pow(alfa,3);
	Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	xx = epsilon * v * (1 + (ta / 3)) + 500000;
	yy = nu * v * (1 + ta) + Bm;
	if (yy < 0)
	{
		yy = 9999999 + yy;
	}
	pgps->CorX = xx;
	pgps->CorY = yy;
}

/** @brief  : Convert lattitude and longtitude value into Degree
**  @agr    : Lattitude value
**  @retval : Degree value
**/
double GPS_LLToDegree(double LL)
{
	double degree, minute, temp;
	degree = (int)(LL / 100);
	temp = (double)(degree * 100);
	minute = LL - temp;
	minute = minute / 60;
	return (degree +  minute);
}

/** @brief  : Initial value for GPS functions
**  @agr    : input
**  @retval : Return fix value
**/
void GPS_ParametersInit(GPS *pgps)
{
	pgps->CorX = 0;
	pgps->CorY = 0;
	pgps->Latitude = 0;
	pgps->Longitude = 0;
	pgps->Robot_Velocity = 0;
	pgps->NbOfWayPoints = 0;
	pgps->Pre_CorX = 0;
	pgps->Pre_CorY = 0;
	pgps->Goal_Flag = Check_NOK;
	pgps->GPS_Error = Veh_NoneError;
	pgps->K = 0.5;
	pgps->NbOfP = 0;
	pgps->Step = 0.5;
	pgps->dmin = 0;
	pgps->Cor_Index = 0;
	pgps->efa = 0;
}

/** @brief  : GPS updates path yaw 
**  @agr    : GPS
**  @retval : none
**/
void	GPS_UpdatePathYaw(GPS *pgps)
{
	for(int i = 0; i < pgps->NbOfP; i++)
	{
		pgps->P_Yaw[i] = atan2(pgps->P_Y[i + 1] - pgps->P_Y[i],pgps->P_X[i + 1] - pgps->P_X[i]);
	}
	pgps->P_Yaw[pgps->NbOfP - 1] = pgps->P_Yaw[pgps->NbOfP - 2];
}

/** @brief  : GPS updates path coordinate from PC format message includes: pathx, pathy and pathyaw
**  @agr    : GPS and inputmessage
**  @retval : none
**/
void	GPS_UpdatePathCoordinate(GPS *pgps, uint8_t *inputmessage)
{
	GetMessageInfo((char*)inputmessage,pgps->TempBuffer,',');
	pgps->Latitude 			= GetValueFromString(&pgps->TempBuffer[1][0]);
	pgps->Longitude			= GetValueFromString(&pgps->TempBuffer[2][0]);
	GPS_LatLonToUTM(pgps);
	pgps->Path_X[pgps->NbOfWayPoints]			= pgps->CorX;	
	pgps->Path_Y[pgps->NbOfWayPoints]			= pgps->CorY;
	pgps->NbOfWayPoints++;
	pgps->CorX = 0;
	pgps->CorY = 0;
	pgps->Latitude = 0;
	pgps->Longitude = 0;
}

void	GPS_UpdatePathCoordinateV2(GPS *pgps, uint8_t *inputmessage)
{
	GetMessageInfo((char*)inputmessage,pgps->TempBuffer,',');
	pgps->NbOfP = (int)GetValueFromString(&pgps->TempBuffer[1][0]);
	pgps->P_X[pgps->NbOfP] = GetValueFromString(&pgps->TempBuffer[2][0]);
	pgps->P_Y[pgps->NbOfP] = GetValueFromString(&pgps->TempBuffer[3][0]);
}
/** @brief  : GPS update K and Step
**  @agr    : GPS, K and Step
**  @retval : none
**/
void	GPS_UpdateParameters(GPS *pgps, double K, double Step)
{
	pgps->K = K;
	pgps->Step = Step;
}

/** @brief  : GPS over write GPS coordinate
**  @agr    : GPS and K
**  @retval : none
**/
void GPS_UpdateCoordinateXY(GPS *pgps, double Cor_X, double Cor_Y)
{
	pgps->CorX = Cor_X;
	pgps->CorY = Cor_Y;
}

/** @brief  : Save GPS path coordinate to internal flash memory
**  @agr    : GPS and FlashMemory
**  @retval : none
**/
void	GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash)
{
	pflash->Length = 0;
	pflash->Length += ToChar(pgps->NbOfWayPoints,&pflash->WriteInBuffer[pflash->Length],1);
	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	for(int i = 0; i < pgps->NbOfWayPoints; i++)
	{
		pflash->Length += ToChar(pgps->Path_X[i],&pflash->WriteInBuffer[pflash->Length],6);
		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
		pflash->Length += ToChar(pgps->Path_Y[i],&pflash->WriteInBuffer[pflash->Length],6);
		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
	}
	EraseMemory(FLASH_Sector_6);
	WriteToFlash(pflash,FLASH_Sector_6,FLASH_GPSPara_BaseAddr);
}

/** @brief  : Function get lat lon value from GNGLL message
**  @agr    : String value received from message
**  @retval : Value
**/
void GPS_GetLatFromString(GPS *pgps, char *inputmessage)
{
	double s1 = 0, s2 = 0;
	int temp = 1000;
	for(int i = 0; i < 4; i++)
	{
		s1 += (inputmessage[i] - 48) * temp;
		temp /= 10;
	}
	temp = 10000;
	for(int i = 5; i < 10; i++)
	{
		s2 += (inputmessage[i] - 48) * temp;
		temp /= 10;
	}
	s2 /= 100000;
	pgps->Latitude = GPS_LLToDegree(s1 + s2);
}

void GPS_ClearPathCorBuffer(GPS *pgps)
{
	for(int i = 0; i < 20; i++)
	{
		pgps->Path_X[i] = 0;
		pgps->Path_Y[i] = 0;
	}
}

void GPS_ClearPathBuffer(GPS *pgps)
{
	for(int i = 0; i < 3000; i++)
	{
		pgps->P_X[i] = 0;
		pgps->P_Y[i] = 0;
		pgps->P_Yaw[i] = 0;
	}
}


/** @brief  : Function get lat lon value from GNGLL message
**  @agr    : String value received from message
**  @retval : Value
**/
void GPS_GetLonFromString(GPS *pgps, char *inputmessage)
{
	double s1 = 0, s2 = 0;
	int temp = 10000;
	for(int i = 0; i < 5; i++)
	{
		s1 += (inputmessage[i] - 48) * temp;
		s2 += (inputmessage[i + 6] - 48) * temp;
		temp /= 10;
	}
	s2 /= 100000;
	pgps->Longitude = GPS_LLToDegree(s1 + s2);
}

void	GPS_PathPlanning(GPS *pgps, float Step)
{
	double temp, a, b;
	pgps->NbOfP = 0;
	for(int i = 0; i < pgps->NbOfWayPoints - 1; i++)
	{
		a = (pgps->Path_Y[i + 1] - pgps->Path_Y[i]) / (pgps->Path_X[i + 1] - pgps->Path_X[i]);
		b = pgps->Path_Y[i] - a * pgps->Path_X[i];
		temp = pgps->Path_X[i];
		if(pgps->Path_X[i] < pgps->Path_X[i + 1])
		{
			while(temp < pgps->Path_X[i + 1])
			{
				pgps->P_X[pgps->NbOfP] = temp;
				pgps->P_Y[pgps->NbOfP] = a * pgps->P_X[pgps->NbOfP] + b;
				pgps->NbOfP++;
				temp += Step;
			}
		}
		else
		{
			while(temp > pgps->Path_X[i + 1])
			{
				pgps->P_X[pgps->NbOfP] = temp;
				pgps->P_Y[pgps->NbOfP] = a * pgps->P_X[pgps->NbOfP] + b;
				pgps->NbOfP++;
				temp -= Step;
			}
		}
	}
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void GPS_StanleyControl(GPS *pgps, double SampleTime, double M1Velocity, double M2Velocity)
{                   /*   Current pose of the robot   / /  Path coordinate  / /  ThetaP  */
	double dmin = 0,dx,dy,d;
	int 	 index = 0;
	double efa, goal_radius, VM1, VM2, AngleRadian;
	pgps->Angle = &Mag;
	AngleRadian = pgps->Angle->Angle * (double)pi/180;
	/*if(45*pi/180 <AngleRadian > 135*pi/180)
		AngleRadian -= 20*pi/180;
	else if(-135*pi/180 <AngleRadian < -45*pi/180)
		AngleRadian += 20*pi/180;*/
	AngleRadian = pi/2 - AngleRadian;
	/*if(AngleRadian > 0)
		AngleRadian += 33*pi/180;
	else
		AngleRadian -= 33*pi/180;*/
	/* V = sqrt(vx^2 + vy^2) */
	
	VM1 = Wheel_Radius * 2 * pi * M1Velocity/60;	
	VM2 = Wheel_Radius * 2 * pi * M2Velocity/60;
	pgps->Robot_Velocity = (VM1 + VM2)/2;
	//Searching the nearest point
	//---------------------------
	for(int i = 0; i < pgps->NbOfP; i++)
	{
		dx = pgps->CorX - pgps->P_X[i];
		dy = pgps->CorY - pgps->P_Y[i];
		d  = sqrt(pow(dx,2) + pow(dy,2));
		if(i == 0)
		{
			dmin 	= d;
			index = i;
		}
		else
		{
			if(dmin > d) // d is the new value that near the point
			{
				dmin 	= d;  // min 
				index = i;	// position of the minimum value
			}
		}
	}
	pgps->dmin = dmin;
	pgps->P_Yaw_Index = index;
	efa = - ((pgps->CorX - pgps->P_X[index]) * (cos(AngleRadian + pi/2)) + (pgps->CorY - pgps->P_Y[index]) * sin(AngleRadian + pi/2));
	pgps->efa = efa;
	goal_radius = sqrt(pow(pgps->CorX - pgps->P_X[pgps->NbOfWayPoints - 1],2) + pow(pgps->CorY - pgps->P_Y[pgps->NbOfWayPoints - 1],2));
	if(goal_radius <= 1)
		Status_UpdateStatus(&GPS_NEO.Goal_Flag,Check_OK);
	pgps->Thetae = Pi_To_Pi(AngleRadian - pgps->P_Yaw[index]);
	pgps->Thetad = -atan2(pgps->K * efa, pgps->Robot_Velocity);
	pgps->Delta_Angle  = (pgps->Thetae + pgps->Thetad)*(double)180/pi;
}

/** @brief  : Header compare GPS message
**  @agr    : Input header
**  @retval : Check status
**/
Check_Status	GPS_HeaderCompare(uint8_t *s1, char Header[5])
{
	for(int i = 0; i < 5; i++)
	{
		if(s1[i] != (uint8_t)Header[i])
			return Check_NOK;
	}
	return Check_OK;
}

/** @brief  : Get message from NMEA protocol
**  @agr    : GPS and Inputmessage
**  @retval : None
**/
Vehicle_Error	GPS_GetLLQMessage(GPS *pgps, uint8_t *inputmessage,	char result[50][30])
{
	int Message_Index = 0, GxGLL_Index = 0, GxGGA_Index = 0, Length = 0;
	while(inputmessage[Message_Index] != 0)
	{
		if(inputmessage[Message_Index] == (uint8_t)'$')
		{
			if((GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GNGGA")) || (GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GPGGA")))
			{
				GxGGA_Index = Message_Index;
				Message_Index++;
			}
			else if((GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GNGLL")) || (GPS_HeaderCompare(&inputmessage[Message_Index + 1],"GPGLL")))
			{
				GxGLL_Index = Message_Index;
				break;
			}
			else Message_Index++;
		}
		else Message_Index++;
		if(Message_Index > 1000)
		{
			return Veh_ReadMessage_Err;
		}
	}
	if(GxGLL_Index != 0)
	{
		Length = LengthOfLine(&inputmessage[GxGLL_Index]);
		GetMessageInfo((char*)&inputmessage[GxGLL_Index], result, ',');
		if(IsCorrectMessage(&inputmessage[GxGLL_Index + 1], Length - 4, inputmessage[GxGLL_Index + Length - 2], inputmessage[GxGLL_Index + Length - 1]))
		{
			if(IsValidData(result[6][0]))
			{
				GPS_GetLatFromString(&GPS_NEO,&result[1][0]);
				GPS_GetLonFromString(&GPS_NEO,&result[3][0]);
				GPS_LatLonToUTM(&GPS_NEO);
			}
			else return Veh_InvalidGxGLLMessage_Err;
		}
		else return Veh_GxGLLCheckSum_Err;
	}
	else return Veh_ReadGxGLLMessage_Err;
	if(GxGGA_Index != 0)
	{
		Length = LengthOfLine(&inputmessage[GxGGA_Index]);
		GetMessageInfo((char*)&inputmessage[GxGGA_Index], result, ',');
		if(IsCorrectMessage(&inputmessage[GxGGA_Index + 1], Length - 4, inputmessage[GxGGA_Index + Length - 2], inputmessage[GxGGA_Index + Length - 1]))
		{
			pgps->GPS_Quality = (GPS_Quality)GetValueFromString(&result[6][0]);
		}
		else return Veh_GxGGACheckSum_Err;
	}
	else return Veh_ReadGxGGAMessage_Err;
	return Veh_NoneError;
}

/** @brief  : Get message from GNGLL/GPGLL 
**  @agr    : GPS and Inputmessage
**  @retval : None
**/
Check_Status	GPS_GetCoordinateMessage(uint8_t *inputmessage, char result[50][30])
{
	int index = 0;
	Check_Status flag = Check_NOK;
	while(inputmessage[index] != 0)
	{
		if(inputmessage[index] == '$')
		{
			if(((inputmessage[index + 1] == 'G') && (inputmessage[index + 2] == 'N') && (inputmessage[index + 3] == 'G') && (inputmessage[index + 4] == 'L') && (inputmessage[index + 5] == 'L')) || ((inputmessage[index + 1] == 'G') && (inputmessage[index + 2] == 'P') && (inputmessage[index + 3] == 'G') && (inputmessage[index + 4] == 'L') && (inputmessage[index + 5] == 'L')))
			{
				flag = Check_OK;
				break;
			}
			else index++;
		}
		else index++;
	}
	if(flag)
		GetMessageInfo((char*)&inputmessage[index],result,',');
	else return Check_NOK;
	return IsCorrectMessage((uint8_t*)&inputmessage[index + 1], 46,(uint8_t)result[7][2], (uint8_t)result[7][3]);
}

/** @brief  : Get quality value from GNGGA/GPGGA message
**  @agr    : GPS and Inputmessage
**  @retval : None
**/
Check_Status GPS_GetQualityFromString(GPS *pgps, uint8_t *inputmessage, char result[50][30]) 
{
	int index = 0;
	Check_Status flag = Check_NOK;
	while(inputmessage[index] != 0)
	{
		if(inputmessage[index] == '$')
		{
			if(((inputmessage[index + 1] == 'G') && (inputmessage[index + 2] == 'N') && (inputmessage[index + 3] == 'G') && (inputmessage[index + 4] == 'G') && (inputmessage[index + 5] == 'A')) || ((inputmessage[index + 1] == 'G') && (inputmessage[index + 2] == 'P') && (inputmessage[index + 3] == 'G') && (inputmessage[index + 4] == 'G') && (inputmessage[index + 5] == 'A')))
			{
				flag = Check_OK;
				break;
			}
			else index++;
		}
		else index++;
	}
	if(flag)
		GetMessageInfo((char*)&inputmessage[index],result,',');
	else return Check_NOK;
	return IsCorrectMessage(&inputmessage[index + 1],LengthOfLine(&inputmessage[index + 1]) - 3, (uint8_t)result[14][5], (uint8_t)result[14][6]);
}

/*-------------------- Fuzzy control -----------------------*/
/* Variables of fuzzy control block */

/** @brief  : Find maximum number
**  @agr    : 2 input values
**  @retval : Output maximum value
**/
double Fuzzy_Min(double in1, double in2)
{
	double min;
	min = in1;
	if(min > in2) min = in2;
	return min;
}

double Fuzzy_Max(double *input,int len)
{
	double max;
  max = input[0];
	for(int i = 1; i < len; i++)
	{
		if(max < input[i])
		{
			max = input[i];
		}
	}
	return max;
}

/** @brief  : Trapf function
**  @agr    : 4 parameters (left to right)
**  @retval : Value relates to x
**/
double Trapf(trapf *ptrapf, double x)
{
	double result;
	if(x < ptrapf->h1) result = 0;
	else if(x < ptrapf->h2) result = (x - ptrapf->h1) / (ptrapf->h2 - ptrapf->h1);
	else if(x < ptrapf->h3) result = 1;
	else if(x < ptrapf->h4) result = (ptrapf->h4 - x) / (ptrapf->h4 - ptrapf->h3);
	else result = 0;
	return result;
}

/** @brief  : Trimf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
double Trimf(trimf *ptrimf, double x)
{
	double result;
	if(x < ptrimf->a1) result = 0;
	else if(x < ptrimf->a2) result = (x - ptrimf->a2) / (ptrimf->a2 - ptrimf->a1);
	else if(x < ptrimf->a3) result = (ptrimf->a3 - x) / (ptrimf->a3 - ptrimf->a2);
	else result = 0;
	return result;
}

/** @brief  : Update Paramter for Trimf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Trimf_Update(trimf *ptrimf, double a1, double a2, double a3)
{
	ptrimf->a1 = a1;
	ptrimf->a2 = a2;
	ptrimf->a3 = a3;
}

/** @brief  : Update Paramerters for trapf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4)
{
	ptrapf->h1 = a1;
	ptrapf->h2 = a2;
	ptrapf->h3 = a3;
	ptrapf->h4 = a4;
}

/** @brief  : Fuzzy init parameter procedure
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Fuzzy_ParametersInit(void)
{
	/*   Input 1 (e = Set_theta - theta)  */
		// NB : -2 - -0.17
		Trapf_Update(&In1_NB,-2,-1,-0.22,-0.17);
		// NS : 0.15 - 0.45
		Trimf_Update(&In1_NS,-0.22,-0.11,-0.044);
		// ZE : 0 - 0.2
		Trimf_Update(&In1_ZE,-0.056,0,0.056);
		// PS : 0.15 - 0.45
		Trimf_Update(&In1_PS,0.044,0.11,0.22);
		// PB : 0.4 - 1
		Trapf_Update(&In1_PB,0.17,0.22,1,2);
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE : 0.3 - 1
		Trapf_Update(&In2_NE,-2,-1,-0.4,-0.3);
		// ZE : 0 - 0.4
		Trimf_Update(&In2_ZE,-0.4,0,0.4);
		// PO : 0.3 - 1
		Trapf_Update(&In2_PO,0.3,0.4,1,2);
		/* Output value */
		NB = -0.9;
		NM = -0.75;
		NS = -0.5;
		ZE = 0;
		PS = 0.5;
		PM = 0.75;
		PB = 0.9;
}

void	SelectFuzzyOutput(double vel)
{
	/*----------Fuzzy parameter init ------------------*/
	if (vel < ToRPM(0.3))
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB : 0.4 - 1
		In1_NB.h1 = -2;
		In1_NB.h2 = -1;
		In1_NB.h3 = -0.22;
		In1_NB.h4 = -0.17;
		// NS : 0.15 - 0.45
		In1_NS.a1 = -0.22;
		In1_NS.a2 = -0.11;
		In1_NS.a3 = -0.044;
		// ZE : 0 - 0.2
		In1_ZE.a1 = -0.056;
		In1_ZE.a2 = 0;
		In1_ZE.a3 = 0.056;
		// PS : 0.15 - 0.45
		In1_PS.a1 = 0.044;
		In1_PS.a2 = 0.11;
		In1_PS.a3 = 0.22;
		// PB : 0.4 - 1
		In1_PB.h1 = 0.17;
		In1_PB.h2 = 0.22;
		In1_PB.h3 = 1;
		In1_PB.h4 = 2;
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE : 0.3 - 1
		In2_NE.h1 = -2;
		In2_NE.h2 = -1;
		In2_NE.h3 = -0.4;
		In2_NE.h4 = -0.3;
		// ZE : 0 - 0.4
		In2_ZE.a1 = -0.4;
		In2_ZE.a2 = 0;
		In2_ZE.a3 = 0.4;
		// PO : 0.3 - 1
		In2_PO.h1 = 0.3;
		In2_PO.h2 = 0.4;
		In2_PO.h3 = 1;
		In2_PO.h4 = 2;
		/* Output value */
		NB = -0.9;
		NM = -0.75;
		NS = -0.5;
		ZE = 0;
		PS = 0.5;
		PM = 0.75;
		PB = 0.9;
	}
	else
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB : 0.4 - 1
		In1_NB.h1 = -2;
		In1_NB.h2 = -1;
		In1_NB.h3 = -0.45;
		In1_NB.h4 = -0.4;
		// NS : 0.15 - 0.45
		In1_NS.a1 = -0.45;
		In1_NS.a2 = -0.2;
		In1_NS.a3 = -0.15;
		// ZE : 0 - 0.2
		In1_ZE.a1 = -0.2;
		In2_ZE.a2 = 0;
		In2_ZE.a3 = 0.2;
		// PS : 0.15 - 0.45
		In1_PS.a1 = 0.15;
		In1_PS.a2 = 0.2;
		In1_PS.a3 = 0.45;
		// PB : 0.4 - 1
		In1_PB.h1 = 0.4;
		In1_PB.h2 = 0.45;
		In1_PB.h3 = 1;
		In1_PB.h4 = 2;
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE : 0.3 - 1
		In2_NE.h1 = -2;
		In2_NE.h2 = -1;
		In2_NE.h3 = -0.4;
		In2_NE.h4 = -0.3;
		// ZE : 0 - 0.4
		In2_ZE.a1 = -0.4;
		In2_ZE.a2 = 0;
		In2_ZE.a3 = 0.4;
		// PO : 0.3 - 1
		In2_PO.h1 = 0.3;
		In2_PO.h2 = 0.4;
		In2_PO.h3 = 1;
		In2_PO.h4 = 2;
		/* Output value */
		NB = -0.75;
		NM = -0.4;
		NS = -0.175;
		ZE = 0;
		PS = 0.175;
		PM = 0.4;
		PB = 0.75;
	}
}

/** @brief  : Defuzzification Max Min sugeno
**  @agr    : 2 input value
**  @retval : Output value
**/

void	Defuzzification_Max_Min(IMU *pimu)
{
	double pBeta[5], num = 0, den = 0, temp;
	//NB and NE is NB
	pBeta[0] = Fuzzy_Min(Trapf(&In1_NB,pimu->Fuzzy_Error), Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	num = NB * pBeta[0];
	den = pBeta[0];
	//NS and NE is NM
	//NB and ZE is NM
	pBeta[0] = Fuzzy_Min(Trimf(&In1_NS,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trapf(&In1_NB,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,2);
	num += NM * temp;
	den += temp;
	//ZE and NE is NS
	//NS and ZE is NS
	//NB and PO is NS
	pBeta[0] = Fuzzy_Min(Trimf(&In1_ZE,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_NS,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[2] = Fuzzy_Min(Trapf(&In1_NB,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,3);
	num += NS * temp;
	den += temp;
	//PS and NE is ZE
	//ZE and ZE is ZE
	//NS and PO is ZE
	pBeta[0] = Fuzzy_Min(Trimf(&In1_PS,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_ZE,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[2] = Fuzzy_Min(Trimf(&In1_NS,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,3);
	num += ZE * temp;
	den += temp;
	//PB and NE is PS
	//PS and ZE is PS
	//ZE and PO is PS
	pBeta[0] = Fuzzy_Min(Trapf(&In1_PB,pimu->Fuzzy_Error),Trapf(&In2_NE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_PS,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[2] = Fuzzy_Min(Trimf(&In1_ZE,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,3);
	num += PS * temp;
	den += temp;
	//PB and ZE is PM
	//PS and PO is PM
	pBeta[0] = Fuzzy_Min(Trapf(&In1_PB,pimu->Fuzzy_Error),Trimf(&In2_ZE,pimu->Fuzzy_Error_dot));
	pBeta[1] = Fuzzy_Min(Trimf(&In1_PS,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	temp = Fuzzy_Max(pBeta,2);
	num += PM * temp;
	den += temp;
	//PB and PO is PB
	pBeta[0] = Fuzzy_Min(Trapf(&In1_PB,pimu->Fuzzy_Error),Trapf(&In2_PO,pimu->Fuzzy_Error_dot));
	num += PB * pBeta[0];
	den += pBeta[0];
	if(den == 0) pimu->Fuzzy_Out = 0;
	else
	{
		pimu->Fuzzy_Out = num / den;
	}
}


/*------------------------ Flash read/write function ------------------*/

/*------------------------ IMU functions ------------------*/
/** @brief  : Get data from IMU message
**  @agr    : inputmessage, val
**  @retval : Output value
**/
void	IMU_ParametesInit(IMU *pimu)
{
	pimu->Pre_Angle 					= 0;
	pimu->Set_Angle 					= 0;
	pimu->Angle 							= 0;
	pimu->Fuzzy_Out 					= 0;
	pimu->Fuzzy_Error 				= 0;
	pimu->Fuzzy_Error_dot			= 0;
}

/** @brief  : Update Set angle
**  @agr    : IMU and Angle
**  @retval : none
**/
void	IMU_UpdateSetAngle(IMU *pimu, double ComAngle)
{
	double temp;
	temp = pimu->Angle + ComAngle;
	pimu->Set_Angle = Degree_To_Degree(temp);
}

/** @brief  : Update previous angle
**  @agr    : IMU
**  @retval : none
**/
void	IMU_UpdatePreAngle(IMU *pimu)
{
	pimu->Pre_Angle = pimu->Angle;
}


/** @brief  : Update input for fuzzy controller
**  @agr    : imu and sampletime
**  @retval : none
**/
void	IMU_UpdateFuzzyInput(IMU *pimu, double *pSampleTime)
{
	pimu->Fuzzy_Error 		= pimu->Set_Angle - pimu->Angle;
	pimu->Fuzzy_Error_dot = -(pimu->Angle - pimu->Pre_Angle)/(*pSampleTime);
	if(pimu->Fuzzy_Error > 180) pimu->Fuzzy_Error -= 360;
	else if(pimu->Fuzzy_Error < -180) pimu->Fuzzy_Error += 360;
	pimu->Fuzzy_Error 		*= pimu->Ke;
	pimu->Fuzzy_Error_dot *= pimu->Kedot;
}

/** @brief  : Update fuzzy coefficients
**  @agr    : imu and Ke,kedot,ku
**  @retval : none
**/
void	IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku)
{
	pimu->Ke		  = Ke;
	pimu->Kedot 	= Kedot;
	pimu->Ku 			= Ku;
}

/** @brief  : Get data from IMU message
**  @agr    : inputmessage, val
**  @retval : Output value
**/
Vehicle_Error IMU_GetValueFromMessage(IMU *pimu, uint8_t *inputmessage)
{
	int temp = 100000;
	double Angle = 0;
	if(inputmessage[0] == 0x0A)
	{
		for(int i = 0; i < 6; i++)
		{
			Angle += (inputmessage[IMU_AngleIndex + 1 + i] - 48) * temp;
			temp /= 10;
		}
		Angle /= 1000;
		if(inputmessage[IMU_AngleIndex] == (uint8_t)' ') pimu->Angle = Angle;
		else pimu->Angle = -Angle;
	}
	else
	{
		pimu->Angle = 0;
		return Veh_IMUWrongMessage_Err;
	}
	return Veh_NoneError;
}

/** @brief  : Copy IMU message to TX buffer 
**  @agr    : input and output message
**  @retval : Length
**/
int IMU_GetCommandMessage(char *inputmessage, uint8_t *outputmessage)
{
	int i = 0;
	while(inputmessage[i] != 0)
	{
		outputmessage[i] = (uint8_t)inputmessage[i];
		i++;
	}
	outputmessage[i++] = 0x0D;
	outputmessage[i++] = 0x0A;
	return i;
}


/*---------------- Read / Write Flash Memory Functions -----------------*/
/** @brief  : Convert 4 bytes (in char) to a Word data in order to save in flash memory
**  @agr    : Input array of bytes data
**  @retval : Number of 32 bits address
**/
int Convert4BytesToWordData(uint8_t *pInput, uint32_t *pOutBuffer, int InputLength)
{
	int length = 0, h = 0;
	if((InputLength % 4) != 0)
		length = InputLength / 4 + 1;
	else
		length /= 4;
	for(int i = 0; i < length; i++)
	{
		for(int j = 3; j >= 0; j--)
		{
			pOutBuffer[i] += pInput[h] << (8 * j);
			h++;
		}
	}
	return length;
}

/** @brief  : Erase sector flash memory
**  @agr    : None
**  @retval : None
**/
void EraseMemory(uint32_t Flash_Sector)
{
	FLASH_Unlock();
	FLASH_EraseSector(Flash_Sector,FLASH_ProgramType_Word);
	FLASH_Lock();
}

/** @brief  : Write word data to Flash memory
**  @agr    : Flash_Sector, Sector_BaseAddr and input word data pointer
**  @retval : None
**/
void WriteToFlash(FlashMemory *pflash, uint32_t FLASH_Sector, uint32_t FLASH_BaseAddr)
{
	for(int i = 0; i < 100; i++)
	{
		pflash->WriteIn32bBuffer[i] = 0;
	}
	pflash->WriteIn32bBuffer[0] = Convert4BytesToWordData(pflash->WriteInBuffer,&pflash->WriteIn32bBuffer[1],pflash->Length);
	FLASH_Unlock();
	FLASH_ProgramWord(FLASH_BaseAddr,pflash->WriteIn32bBuffer[0]);
	FLASH_BaseAddr += 4;
	for(int i = 0; i < pflash->WriteIn32bBuffer[0]; i++)
	{
		FLASH_ProgramWord(FLASH_BaseAddr,pflash->WriteIn32bBuffer[i + 1]);
		FLASH_BaseAddr += 4;
	}
	FLASH_Lock();
}

/** @brief  : Read from flash memory
**  @agr    : input and output message
**  @retval : None
**/
void ReadFromFlash(FlashMemory *pflash, uint32_t FLASH_BaseAddr)
{
	int i = 0;
	uint32_t mask = 0xFF000000;
	int length = (int)(*(uint32_t*)FLASH_BaseAddr);
	FLASH_BaseAddr += 4;
	for(int count = 0; count < length; count++)
	{
		for(int j = 3; j >= 0; j--)
		{
			pflash->ReadOutBuffer[i] = (uint8_t)(((*(uint32_t*)FLASH_BaseAddr) & mask) >> (8 * j));
			mask >>= 8;
			i++;
		}
		mask = 0xFF000000;
		FLASH_BaseAddr += 4;
	}
}


void GPS_UpdateNewCoordinates(GPS *pgps, double SampleTime)
{
	double dx, dy;
	dx = pgps->CorX - pgps->Pre_CorX;
	dy = pgps->CorY - pgps->Pre_CorY;
	pgps->Pre_CorX = pgps->CorX;
	pgps->Pre_CorY = pgps->CorY;
	pgps->CorX += dx * SampleTime;
	pgps->CorY += dy * SampleTime;
}









