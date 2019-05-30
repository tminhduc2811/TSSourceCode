#include "stm32f4xx.h"
#include <string.h>
#include <math.h>
/* Define value */

/* Build-in function */
extern double ToDouble(int input);
extern int ToInt(double input);
extern void GPS_LatLonToUTM(double Lat,double Lon,double *result);
extern void GPS_ReadLine(uint8_t *result, uint8_t *inputbuff, uint16_t NbOfData);
extern void GPS_GetUTMFromGNGLLMessage(uint8_t *inputbuff,double *result);
extern void GPS_USARTDMA_SendCoordinate(double *cor,uint8_t *y);
extern uint16_t GPS_LengthOfLineInArray(uint8_t *inputarray);
extern void GPS_USARTDMA_SendMessage(uint8_t *inputbuff, uint8_t *Tx_Buffer,uint16_t LenOfBuff);