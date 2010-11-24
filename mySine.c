/*
 * mysin.c
 *
 *  Created on: Nov 22, 2010
 *      Author: marhes
 *      Taken from: http://www.hevi.info/tag/fast-sine-function/
 */

#include "mySine.h"

double hollyConstant = 0.017453292519943295769236907684886;

//First of all sine and cosine tables
double sinTable[] = {
	0.0,                                    //sin(0)
	0.17364817766693034885171662676931 ,    //sin(10)
	0.34202014332566873304409961468226 ,    //sin(20)
	0.5 ,                                   //sin(30)
	0.64278760968653932632264340990726 ,    //sin(40)
	0.76604444311897803520239265055542 ,    //sin(50)
	0.86602540378443864676372317075294 ,    //sin(60)
	0.93969262078590838405410927732473 ,    //sin(70)
	0.98480775301220805936674302458952 ,    //sin(80)
	1.0                                     //sin(90)
};

double cosTable[] = {
	1.0 ,                                   //cos(0)
	0.99984769515639123915701155881391 ,    //cos(1)
	0.99939082701909573000624344004393 ,    //cos(2)
	0.99862953475457387378449205843944 ,    //cos(3)
	0.99756405025982424761316268064426 ,    //cos(4)
	0.99619469809174553229501040247389 ,    //cos(5)
	0.99452189536827333692269194498057 ,    //cos(6)
	0.99254615164132203498006158933058 ,    //cos(7)
	0.99026806874157031508377486734485 ,    //cos(8)
	0.98768834059513772619004024769344      //cos(9)
};

// sin (a+b) = sin(a)*cos(b) + sin(b)*cos(a)
// a = 10*m where m is a natural number and 0<= m <= 90
// i.e. lets a+b = 18.22
// then a = 10, b = 8.22
double myFastSin ( double angle )
{
	int8_t sign;
	int32_t deg;

	deg = angle * 180 / M_PI;

	// Adjust angle to -180..180
	while (deg > 180)
		deg -= 360;
	while (deg < -180)
		deg += 360;

	// Get sign of angle, since sine is symmetric
	if (deg < 0)
	{
		sign = -1;
		deg = -deg;
	}
	else
		sign = 1;

	// Mirror over 90
	if (deg > 90)
		deg = 180 - angle;

	int a = deg * 0.1f;
    double b = deg - 10 * a;
    return sinTable[a] * cosTable[(int)b] + b * hollyConstant * sinTable[9-a];
}

double myFastCos ( double angle )
{
	return myFastSin(M_PI_2 - angle);
}
