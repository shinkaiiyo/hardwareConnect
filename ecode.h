#ifndef _ecode_h_
#define _ecode_h_

#include "stdint.h"

#define ERROR_OFFSET											 0x80

#define ERROR_DISCONNECTED					                     ERROR_OFFSET + 1
#define ERROR_OUTOF_RANGE_LINEAR_BALLON					         ERROR_OFFSET + 2
#define ERROR_OUTOF_RANGE_LINEAR_TUBE					         ERROR_OFFSET + 3
#define ERROR_OUTOF_RANGE_LINEAR_GUIDEWIRE					     ERROR_OFFSET + 4
#define ERROR_OUTOF_RANGE_ROTATION_TUBE					         ERROR_OFFSET + 5
#define ERROR_OUTOF_RANGE_ROTATION_GUIDEWIRE					 ERROR_OFFSET + 6
#define ERROR_FLAME_TIMEOUT					                     ERROR_OFFSET + 7
#define ERROR_NO_BOARDCASE					                     ERROR_OFFSET + 8
#define ERROR_MOTION_MODE 										 ERROR_OFFSET + 9
#define ERROR_CRC_CHECK 										 ERROR_OFFSET + 10
#define ERROR_ENDLESS_LOOP                                       ERROR_OFFSET + 11
#define ERROR_MESSAGE_TYPE                                       ERROR_OFFSET + 12

#endif
