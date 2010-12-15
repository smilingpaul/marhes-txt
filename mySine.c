/*
 * mysin.c
 *
 *  Created on: Nov 22, 2010
 *      Author: marhes
 *      Taken from: http://www.hevi.info/tag/fast-sine-function/
 */

#include "mySine.h"

//double hollyConstant = 0.01745;//3292519943295769236907684886;
//
////First of all sine and cosine tables
//double sinTable[] = {
//	0.0,                                    //sin(0)
//	0.1736,//4817766693034885171662676931 ,    //sin(10)
//	0.342,//02014332566873304409961468226 ,    //sin(20)
//	0.5 ,                                   //sin(30)
//	0.6427,//8760968653932632264340990726 ,    //sin(40)
//	0.766,//04444311897803520239265055542 ,    //sin(50)
//	0.866,//02540378443864676372317075294 ,    //sin(60)
//	0.939,//69262078590838405410927732473 ,    //sin(70)
//	0.9848,//0775301220805936674302458952 ,    //sin(80)
//	1.0                                     //sin(90)
//};
//
//double cosTable[] = {
//	1.0 ,                                   //cos(0)
//	0.9998,//4769515639123915701155881391 ,    //cos(1)
//	0.99939,//082701909573000624344004393 ,    //cos(2)
//	0.9986,//2953475457387378449205843944 ,    //cos(3)
//	0.99756,//405025982424761316268064426 ,    //cos(4)
//	0.99619,//469809174553229501040247389 ,    //cos(5)
//	0.99452,//189536827333692269194498057 ,    //cos(6)
//	0.9925,//4615164132203498006158933058 ,    //cos(7)
//	0.990,//26806874157031508377486734485 ,    //cos(8)
//	0.9876//8834059513772619004024769344      //cos(9)
//};

double sinTable[] = {
		0,
		0.0174524064372835,
		0.034899496702501,
		0.0523359562429438,
		0.0697564737441253,
		0.0871557427476582,
		0.104528463267653,
		0.121869343405147,
		0.139173100960065,
		0.156434465040231,
		0.17364817766693,
		0.190808995376545,
		0.207911690817759,
		0.224951054343865,
		0.241921895599668,
		0.258819045102521,
		0.275637355816999,
		0.292371704722737,
		0.309016994374947,
		0.325568154457157,
		0.342020143325669,
		0.3583679495453,
		0.374606593415912,
		0.390731128489274,
		0.4067366430758,
		0.422618261740699,
		0.438371146789077,
		0.453990499739547,
		0.469471562785891,
		0.484809620246337,
		0.5,
		0.515038074910054,
		0.529919264233205,
		0.544639035015027,
		0.559192903470747,
		0.573576436351046,
		0.587785252292473,
		0.601815023152048,
		0.615661475325658,
		0.629320391049837,
		0.642787609686539,
		0.656059028990507,
		0.669130606358858,
		0.681998360062498,
		0.694658370458997,
		0.707106781186547,
		0.719339800338651,
		0.73135370161917,
		0.743144825477394,
		0.754709580222772,
		0.766044443118978,
		0.777145961456971,
		0.788010753606722,
		0.798635510047293,
		0.809016994374947,
		0.819152044288992,
		0.829037572555042,
		0.838670567945424,
		0.848048096156426,
		0.857167300702112,
		0.866025403784439,
		0.874619707139396,
		0.882947592858927,
		0.891006524188368,
		0.898794046299167,
		0.90630778703665,
		0.913545457642601,
		0.92050485345244,
		0.927183854566787,
		0.933580426497202,
		0.939692620785908,
		0.945518575599317,
		0.951056516295153,
		0.956304755963035,
		0.961261695938319,
		0.965925826289068,
		0.970295726275996,
		0.974370064785235,
		0.978147600733806,
		0.981627183447664,
		0.984807753012208,
		0.987688340595138,
		0.99026806874157,
		0.992546151641322,
		0.994521895368273,
		0.996194698091746,
		0.997564050259824,
		0.998629534754574,
		0.999390827019096,
		0.999847695156391,
		1
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

	return sinTable[deg];

//	int a = deg * 0.1f;
//    double b = deg - 10 * a;
//    return sinTable[a] * cosTable[(int)b] + b * hollyConstant * sinTable[9-a];
}

double myFastCos ( double angle )
{
	return myFastSin(M_PI_2 - angle);
}
