/*
 * helperFuncs.h
 *
 *  Created on: Aug 30, 2010
 *      Author: Titus
 */

#ifndef HELPERFUNCS_H_
#define HELPERFUNCS_H_

// Includes
#include "app_types.h"
#include "LPC23xx.h"
#include "armVIC.h"
#include <string.h>

/*************************************************************************
 *             Function declarations
 *************************************************************************/
char* uintToString(uint32_t num, char* buffer);
char* rev(char* str);
int strsize(const char* str);

#endif /* HELPERFUNCS_H_ */
