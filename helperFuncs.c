/*
 * helperFuncs.c
 *
 *  Created on: Aug 30, 2010
 *      Author: Titus
 */

// Includes
#include "app_types.h"
#include "LPC23xx.h"
//#include "armVIC.h"
//#include <string.h>

#include "helperFuncs.h"

/*
** itoa() - will convert integers to char array
**
** expects: an integer
**
** returns: pointer to it's buffer
*/

/* 10 digits + 1 sign + 1 trailing nul */
static char itoa_buf[INT_STR_LENGTH];

char *itoa(int i)
{
    char *pos = itoa_buf + sizeof(itoa_buf) - 1;
    unsigned int u;
    int negative = 0;

    if (i < 0)
    {
        negative = 1;
        u = ((unsigned int)(-(1+i))) + 1;
    }
    else
    {
        u = i;
    }

    *pos = 0;

    do
    {
        *--pos = '0' + (u % 10);
        u /= 10;
    } while (u);

    if (negative)
    {
        *--pos = '-';
    }

    return pos;
}

char* ftostr( char* buffer, float value, int places )
{
    int whole;
    int fraction;
    float fractionFloat;
    char* cptr;
    char* bufIterator;

    bufIterator = buffer;
    
    whole = (int)value;
    if(value < 0)
        fractionFloat = -value - whole;
    else
        fractionFloat = value - whole;
    fraction = (int)(fractionFloat * powf(10.0f,places) + 0.5f);
    
//    strcpy(buffer, itoa(whole));
//    strcat(buffer, ".");
//    strcat(buffer, itoa(fraction));
    
    // Copy whole part of number to string
    cptr = itoa(whole);
    while(*cptr != 0)
    {
    	*bufIterator = *cptr;
    	cptr++;
    	bufIterator++;
    }
    *bufIterator = '.';
    cptr = itoa(fraction);
    while(*cptr != 0)
    {
    	*bufIterator = *cptr;
    	cptr++;
    	bufIterator++;
    }
    return buffer;
}
