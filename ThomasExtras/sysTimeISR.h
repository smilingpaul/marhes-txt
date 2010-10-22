/******************************************************************************
 *
 * $RCSfile: $
 * $Revision: $
 *
 * This module provides the interface definitions for sysTimeISR.c
 * Copyright 2007, Martin Thomas
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_SYSTIME_ISR_H
#define INC_SYSTIME_ISR_H

#include "sysTime.h"

#if defined(SYSTIME_INT_MODE)
void sysTimeISR(void) __attribute__((naked));
#endif

#endif
