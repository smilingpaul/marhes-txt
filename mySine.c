/*
 * mysin.c
 *
 *  Created on: Nov 22, 2010
 *      Author: marhes
 *      Taken from: http://www.hevi.info/tag/fast-sine-function/
 */

#include "mysin.h"

// sin (a+b) = sin(a)*cos(b) + sin(b)*cos(a)
// a = 10*m where m is a natural number and 0<= m <= 90
// i.e. lets a+b = 18.22
// then a = 10, b = 8.22
double myFastSin ( double angle )
{
	int a = angle * 0.1f;
    double b = angle - 10 * a;
    return sinTable[a] * cosTable[(int)b] + b * hollyConstant * sinTable[9-a];
}

double myFastCos ( double angle )
{
	int a = angle * 0.1f;
    double b = angle - 10 * a;
    return sinTable[a] * cosTable[(int)b] + b * hollyConstant * sinTable[9-a];
}
