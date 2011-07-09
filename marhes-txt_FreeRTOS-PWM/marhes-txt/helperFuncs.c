/**
 * @file helperFuncs.c
 * 
 * @brief Includes some helper functions - itoa.
 *
 * @author Titus Appel
 *
 * @version 1.0
 *
 * @date 2010/08/30
 *
 * Contact: titus.appel@gmail.com
 */

#include "helperFuncs.h"

/// The character buffer for conversion
static char itoa_buf[INT_STR_LENGTH];

/**
 * @brief Converts an integer to a char array
 *
 * @param[in] i Input integer (positive or negative)
 * @return Pointer to the char array's buffer
 */
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
