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

#define INT_STR_LENGTH          12

/*************************************************************************
 *             Function declarations
 *************************************************************************/
char *itoa(int i);
char *ftostr( char* buffer, float value, int places );

#endif /* HELPERFUNCS_H_ */
