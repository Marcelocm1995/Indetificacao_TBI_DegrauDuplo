/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_1ORDER_H__
#define __FILTER_1ORDER_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

double FilterDoubleVal(double LfFiltred_Old, double LfRawVal, double LfFilterFactor);
float FilterFloatVal(float LfFiltred_Old, float LfRawVal, float LfFilterFactor);

#endif /* __FILTER_1ORDER_H__ */
