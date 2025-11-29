/*
 * X-Plane Instance API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMINSTANCE_H
#define XPLMINSTANCE_H

#include "XPLMDefs.h"
#include "XPLMScenery.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * INSTANCING
 ***************************************************************************/

typedef void * XPLMInstanceRef;

/* Draw info for positioning an instance */
typedef struct {
    int                  structSize;
    float                x;
    float                y;
    float                z;
    float                pitch;
    float                heading;
    float                roll;
} XPLMDrawInfo_t;

/* Create an instance from an object */
XPLM_API XPLMInstanceRef XPLMCreateInstance(
    XPLMObjectRef        inObj,
    const char **        inDataRefs);

/* Destroy an instance */
XPLM_API void XPLMDestroyInstance(XPLMInstanceRef inInstance);

/* Set instance position */
XPLM_API void XPLMInstanceSetPosition(
    XPLMInstanceRef      inInstance,
    const XPLMDrawInfo_t * inNewPosition,
    const float *        inData);

#ifdef __cplusplus
}
#endif

#endif /* XPLMINSTANCE_H */
