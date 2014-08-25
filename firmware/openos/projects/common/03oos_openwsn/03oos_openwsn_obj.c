/**
DO NOT EDIT DIRECTLY!!

This file was 'objectified' by SCons as a pre-processing
step for the building a Python extension module.

This was done on 2014-08-25 16:30:05.734000.
*/
/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#include "board_obj.h"
#include "scheduler_obj.h"
#include "openwsn_obj.h"


int mote_main(OpenMote* self) {
 board_init(self);
 scheduler_init(self);
 openwsn_init(self);
 scheduler_start(self);
   return 0; // this line should never be reached
}
