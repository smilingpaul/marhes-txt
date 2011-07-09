/**
 * @file helperFuncs.h
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

#ifndef HELPERFUNCS_H_
#define HELPERFUNCS_H_

#include "app_types.h"

/// The maximum string length to return - 10 digits + 1 sign + 1 trailing null
#define INT_STR_LENGTH          12

char *itoa(int32_t i);

#endif
