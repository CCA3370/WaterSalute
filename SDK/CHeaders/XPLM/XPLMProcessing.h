/*
 * X-Plane Processing API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMPROCESSING_H
#define XPLMPROCESSING_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * FLIGHT LOOP CALLBACKS
 ***************************************************************************/

/* Flight loop phase */
typedef int XPLMFlightLoopPhaseType;
#define xplm_FlightLoop_Phase_BeforeFlightModel 0
#define xplm_FlightLoop_Phase_AfterFlightModel  1

/* Flight loop ID */
typedef void * XPLMFlightLoopID;

/* Create flight loop params */
typedef struct {
    int                      structSize;
    XPLMFlightLoopPhaseType  phase;
    XPLMFlightLoop_f         callbackFunc;
    void *                   refcon;
} XPLMCreateFlightLoop_t;

/* Create a flight loop callback */
XPLM_API XPLMFlightLoopID XPLMCreateFlightLoop(
    XPLMCreateFlightLoop_t * inParams);

/* Destroy a flight loop callback */
XPLM_API void XPLMDestroyFlightLoop(XPLMFlightLoopID inFlightLoopID);

/* Schedule a flight loop callback */
XPLM_API void XPLMScheduleFlightLoop(
    XPLMFlightLoopID     inFlightLoopID,
    float                inInterval,
    int                  inRelativeToNow);

/***************************************************************************
 * LEGACY FLIGHT LOOP API
 ***************************************************************************/

XPLM_API void XPLMRegisterFlightLoopCallback(
    XPLMFlightLoop_f     inFlightLoop,
    float                inInterval,
    void *               inRefcon);

XPLM_API void XPLMUnregisterFlightLoopCallback(
    XPLMFlightLoop_f     inFlightLoop,
    void *               inRefcon);

XPLM_API void XPLMSetFlightLoopCallbackInterval(
    XPLMFlightLoop_f     inFlightLoop,
    float                inInterval,
    int                  inRelativeToNow,
    void *               inRefcon);

/***************************************************************************
 * TIME
 ***************************************************************************/

XPLM_API float XPLMGetElapsedTime(void);

XPLM_API int XPLMGetCycleNumber(void);

#ifdef __cplusplus
}
#endif

#endif /* XPLMPROCESSING_H */
