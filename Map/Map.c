#include "Map.h"

float MapFloat(float inVal, float inMin, float inMax, float outMin, float outMax)
{
	if (inVal < inMin)
		inVal = inMin;

	if (inVal > inMax)
		inVal = inMax;

	return ((inVal - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

double MapDouble(double inVal, double inMin, double inMax, double outMin, double outMax)
{
	if (inVal < inMin)
		inVal = inMin;

	if (inVal > inMax)
		inVal = inMax;

	return ((inVal - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}
