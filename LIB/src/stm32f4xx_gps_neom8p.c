#include "stm32f4xx_gps_neom8p.h"
// GPS Handler message from Module GPS Neo M8P
// Received message via USART 's module, Stm32f4 check sum and
// get the right header message

/* (#) Function return a double value from int value */

/**
   *@brief     Function return a double value
   *@agr       int value
   *@RetVal    Return double from int
**/
double ToDouble(int input)
	{
		return (double)input;
	}

/* (#) Function return a int value */
/**
	 *@brief     Function return a int value
	 *@agr       double value or float
	 *@RetVal    int value
**/
int ToInt(double input)
	{
		return (int)input;
	}

/* (#) Function count length of 1 line array */

/**
	*@brief      Function return a length of a array in line
	*@agr        Input array
	*@RetVal     Length of the line in array
**/
uint16_t GPS_LengthOfLineInArray(uint8_t *inputarray)
	{
		uint16_t result;
		int i = 0;
		while((inputarray[i]!= 0x0D) & (inputarray[i + 1] != 0x0A))
			{
				i++;
			}
		result = i + 2;
		return result;
 	}
	
/* (#) Function fix return a double with zero in the right point */

/* (#) Function Calculate the Latitude to degree */
/**
	 *@brief      Function calculate lat to degree
	 *@agr        Latitude
	*@RetVal     A latitude value in degree , ex: 1049.59177
**/
double GPS_LatToDegree(uint8_t *LatInArray)
	{
		double result,temp;
		result = ToDouble(LatInArray[0]) * 10 + ToDouble(LatInArray[1]);
		temp = ToDouble(LatInArray[5] * 10000 + LatInArray[6] * 1000 + LatInArray[7] * 100 + LatInArray[8] * 10 + LatInArray[9]);
		result = result + (ToDouble(LatInArray[2] * 10 + LatInArray[3]) + temp / 100000)/60;
		return result;
	}
	
/* (#) Function Calculate the Lontitude to degree */
/**
	 *@brief      Function calculate lon to degree
	 *@agr        Lotitude
	*@RetVal     A lonitude value in degree, ex:10638.48072
**/
double GPS_LonToDegree(uint8_t *LonInArray)
	{
		double result,temp;
		result = ToDouble(LonInArray[0] * 100 + LonInArray[1] * 10 + LonInArray[2]);
		temp = ToDouble(LonInArray[6] * 10000 + LonInArray[7] * 1000 + LonInArray[8] * 100 + LonInArray[9] * 10 + LonInArray[10]);
		result = result + (ToDouble(LonInArray[3] * 10 + LonInArray[4]) + temp / 100000)/60;
		return result;
	}
/* (#) Function RealLine received buffer in DMA storage */
/**
	 *@brief      Function return a line in receive buffer
	 *@agr        Result buffer and received DMA buffer
	 *@RetVal     Return a result buffer content of the a line array
**/
void GPS_ReadLine(uint8_t *result, uint8_t *inputbuff, uint16_t NbOfData)
	{
		int i = 0;
		if(NbOfData != 0)
			{
				 while(!((inputbuff[i] == 0x0D) & (inputbuff[i+1] == 0x0A)))
					{
						result[i] = inputbuff[i];
						i++;
					}
			}
	}

/* (#) Function math fix roung the double value */
	
/**
	 *@brief      Function roung the double 
	 *@agr        Double
	 *@RetVal     Return a rounded double
**/
double fix(double input)
	{
		double result;
		int temp;
		temp = (int)input;
		result = (double)temp;
		return result;
	}
	
/* (#) Build a function to calculate Lat Lon into UTM value */

/** 
   *@brief      Calculate Lat Lon to UTM
	 *@agr        Input Lat and Lon value from the message and the value buffer
   *@RetVal     Return UTM coordinate
**/

void GPS_LatLonToUTM(double Lat, double Lon, double *result)
	{
		double pi, la, lo, lat, lon, sa, sb, e2, e2cuadrada, c, Huso, S, deltaS, a, epsilon, nu, v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm, xx, yy;
		pi = 3.14159265358979;
		la = Lat;
		lo = Lon;
		sa = 6378137.00000;
		sb = 6356752.314245;
		e2 = pow((pow(sa,2) - pow(sb,2)),0.5) / sb;
		e2cuadrada = pow(e2,2);
		c = pow(sa,2)/sb;
		lat = la * (pi / 180);
		lon = lo * (pi / 180);
		Huso = fix((lo/6) - 183);
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
		j4 = ((3 * j2) + a2) /4;
		j6 = ((5 * j4) + (a2 * pow(cos(lat),2)))/3;
		alfa = (ToDouble(3) / 4) * e2cuadrada;
		beta = (ToDouble(5) / 3) * pow(alfa,2);
		gama = (ToDouble(35) / 27) * pow(alfa,3);
		Bm = 0.9996 * c * (lat - alfa *j2 + beta * j4 - gama * j6);
		xx = epsilon * v * (1 + (ta/3)) + 500000;
		yy = nu * v * (1 + ta) + Bm;
		if (yy < 0)
			{
				yy = 9999999 + yy;
			}
		result[0] = xx;
		result[1] = yy;
	}

/* (#) Function Check the 1 Line in buffer and calculate the lat lon to utm if the condition is true*/

/**
   *@brief     Function check the GNGLL message and calculate the UTM
	 *@agr       The array input 
	 *@RetVal    Return the value UTM, ex: $GNGLL,1049.59177,N,10638.48072,E,154502.00,A,D*71
**/

void GPS_GetUTMFromGNGLLMessage(uint8_t *inputbuff,double *result)
	{
		double lat,lon;
		double UTM[2];
		if((inputbuff[0] == (uint8_t)'$') & (inputbuff[1] == (uint8_t)'G') & (inputbuff[2] == (uint8_t)'N') & (inputbuff[3] == (uint8_t)'G') & (inputbuff[4] == (uint8_t)'L') & (inputbuff[5] == (uint8_t)'L'))
			{
				lat = GPS_LatToDegree(&inputbuff[7]);
				lon = GPS_LonToDegree(&inputbuff[20]);
				GPS_LatLonToUTM(lat,lon,UTM);
				result[0] = UTM[0];
				result[1] = UTM[1];
			}
	}

/* (#) Function send data content of coordinate x and y via USART DMA */

/**
	*@brief        Function send data coordinate
	*@agr          Buffer contain coordinate
	*@RetVal       void
**/
void GPS_USARTDMA_SendCoordinate(double *cor, uint8_t *y)
	{
		
	}

/* (#) Function Send 1 Line message */

/**
	*@brief        Send the received message via USART
	*@agr          Input buff and DMA Tx buff
	*RetVal        Void
**/
void GPS_USARTDMA_SendMessage(uint8_t *inputbuff, uint8_t *Tx_Buffer,uint16_t LenOfBuff)
	{
		for(int i = 0; i < LenOfBuff; i++)
		{
			Tx_Buffer[i] = inputbuff[i];
		}
	}
	
	
	
	
	
	
	
	
	
	
	