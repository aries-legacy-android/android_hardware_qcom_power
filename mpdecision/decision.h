/******************************************************************************
 * 	D E C I S I O N . H

 Copyright (c) 2011 Qualcomm Technologies, Inc.  All Rights Reserved.
 Qualcomm Technologies Proprietary and Confidential.
 ******************************************************************************/

#ifndef DECISION_H
#define DECISION_H

enum CORE_STATUS {
	CORE_DOWN = 0,
	CORE_UP = 1,
};

/* Disable mp-decision hook for performance benchmarks
   Return value: 0 if successful, -1 if failed. */
int disable_decision_engine();

/* Renable mp-decision, called after 'disable_decision_engine'
   for performance benchmarks
   Return value: 0 if successful, -1 if failed. */
int enable_decision_engine();

/* External hook for requesting performance boost
   * tasks - unused parameter
   Return value: 0 if successful, -1 if failed. */
int core_boost(int tasks);

#endif
