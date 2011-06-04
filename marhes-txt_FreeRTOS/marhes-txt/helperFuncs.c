/*
 * helperFuncs.c
 *
 *  Created on: Aug 30, 2010
 *      Author: Titus
 */

// Includes
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

char *itoa(int32_t i)
{
    char *pos = itoa_buf + sizeof(itoa_buf) - 1;
    unsigned int u;
    int32_t negative = 0;

    if (i < 0)
    {
        negative = 1;
        u = ((int32_t)(-(1+i))) + 1;
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
