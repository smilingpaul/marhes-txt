/*
 * helperFuncs.c
 *
 *  Created on: Aug 30, 2010
 *      Author: Titus
 */

// Includes
#include "app_types.h"
#include "LPC23xx.h"
#include "armVIC.h"
//#include <string.h>

#include "helperFuncs.h"

/*************************************************************************
 * Function Name: uintToString
 * Parameters: uint32_t num: the number to convert to a string
 * 			   char* buffer: the location to return the string
 * Return: char*: the location of the return string
 *
 * Description: Converts a uint to a string
 *
 *************************************************************************/
char* uintToString(uint32_t num, char* buffer)
{
	const char SHIFT = 48;
	char* temp = buffer;
	uint32_t quotient = num;
	uint32_t remainder = 0;

	while(quotient > 0)
	{
		remainder = quotient % 10;

		*buffer = (char)remainder + SHIFT;
		buffer++;

		quotient /= 10;
	}

	*buffer = '\0';
	rev(temp);

	return temp;
}

/*************************************************************************
 * Function Name: rev
 * Parameters: char* str: the string to reverse
 * Return: char*: the location of the returned string
 *
 * Description: Reverses the order of a string
 *
 *************************************************************************/
char* rev(char* str)
{
  int end = strsize(str)-1;
  int start = 0;

  while( start<end )
  {
    str[start] ^= str[end];
    str[end] ^=   str[start];
    str[start]^= str[end];

    ++start;
    --end;
  }

  return str;
}

int strsize(const char* str)
{
	const char *s;

	s = str;
	while (*s)
		s++;
	return s - str;
}
