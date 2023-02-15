/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

/*! Import the module header file */
#include "prescribedTrans.h"

/* Other required files to import */
#include <stdbool.h>

/*! This method initializes the output message for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_prescribedTrans(PrescribedTransConfig *configData, int64_t moduleID)
{
    // Initialize the module output message
    PrescribedMotionMsg_C_init(&configData->prescribedMotionOutMsg);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values. This method also checks
 if the module input message is linked.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Reset_prescribedTrans(PrescribedTransConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // Check if the input message is connected
    if (!PrescribedTransMsg_C_isLinked(&configData->prescribedTransInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: prescribedTrans.prescribedTransInMsg wasn't connected.");
    }

    // Set the initial time
    configData->tInit = 0.0;

    // Set the initial convergence to true to enter the correct loop in the Update() method on the first pass
    configData->convergence = true;
}

/*! This method uses the given initial and reference attitudes to compute the required attitude maneuver as
a function of time. The profiled translational trajectory is updated in time and written to the module's prescribed
motion output message.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] Time the method is called
 @param moduleID The module identifier
*/
void Update_prescribedTrans(PrescribedTransConfig *configData, uint64_t callTime, int64_t moduleID)
{
}
