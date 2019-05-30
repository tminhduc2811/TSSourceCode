/* ---------------- Driver for PID control --------------- ***
** Written by Huynh Nhat Tu ------------------------------ ***
** Using Discrete pid algorithm -------------------------- ***
** For controlling purpose ------------------------------- ***
*/

typedef struct pid{
	/* input and output of PID controller */
	double			*SetPoint;
	double			*CurrentPoint;
	double			*SampleTime;
	double			PID_Out;
	/* PID Parameters */
	double			Kp;
	double 			Ki;
	double			Kd;
	/* PID max and min output */
	double			omax;
	double 			omin;
	/* Other parameters */
	double			Pre_PID;
	double 			Pre2_Error;
	double			Pre_Error;
}pid;

/* Export functions */
void	PID_ParameteresInit(pid *ppid);
void	PID_Compute(pid *ppid, double *Current, double *Set, double *SampleTime);
void	PID_UpdateParameters(pid *ppid, double Kp, double Ki, double Kd);
void	PID_UpdateOutputSaturation(pid *ppid, double Omax, double Omin);
