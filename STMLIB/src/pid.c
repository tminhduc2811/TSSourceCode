#include "pid.h"

void	PID_ParametersInit(pid *ppid)
{
	ppid->Pre_PID = 0;
	ppid->Pre2_Error = 0;
	ppid->Pre_Error = 0;
	ppid->PID_Out = 0;
}

void	PID_Compute(pid *ppid, double *Current, double *Set, double *SampleTime)
{
	ppid->CurrentPoint = Current;
	ppid->SetPoint		 = Set;
	ppid->SampleTime	 = SampleTime;
	ppid->PID_Out 		 = ppid->Kp;
}

void	PID_UpdateParameters(pid *ppid, double Kp, double Ki, double Kd)
{
	ppid->Kp = Kp;
	ppid->Ki = Ki;
	ppid->Kd = Kd;
}

void	PID_UpdateOutputSaturation(pid *ppid, double Omax, double Omin)
{
	ppid->omax = Omax;
	ppid->omin = Omin;
}



