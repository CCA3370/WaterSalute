/*
 * X-Plane Scenery API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMSCENERY_H
#define XPLMSCENERY_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * OBJECT LOADING
 ***************************************************************************/

typedef void * XPLMObjectRef;

/* Load an OBJ file - async callback */
typedef void (* XPLMObjectLoaded_f)(
    XPLMObjectRef        inObject,
    void *               inRefcon);

/* Load an OBJ file synchronously */
XPLM_API XPLMObjectRef XPLMLoadObject(const char * inPath);

/* Load an OBJ file asynchronously */
XPLM_API void XPLMLoadObjectAsync(
    const char *         inPath,
    XPLMObjectLoaded_f   inCallback,
    void *               inRefcon);

/* Unload an object */
XPLM_API void XPLMUnloadObject(XPLMObjectRef inObject);

/***************************************************************************
 * TERRAIN PROBE
 ***************************************************************************/

typedef void * XPLMProbeRef;

typedef int XPLMProbeType;
#define xplm_ProbeY 0

typedef int XPLMProbeResult;
#define xplm_ProbeHitTerrain 0
#define xplm_ProbeError      1
#define xplm_ProbeMissed     2

typedef struct {
    int                  structSize;
    float                locationX;
    float                locationY;
    float                locationZ;
    float                normalX;
    float                normalY;
    float                normalZ;
    float                velocityX;
    float                velocityY;
    float                velocityZ;
    int                  is_wet;
} XPLMProbeInfo_t;

XPLM_API XPLMProbeRef XPLMCreateProbe(XPLMProbeType inProbeType);

XPLM_API void XPLMDestroyProbe(XPLMProbeRef inProbe);

XPLM_API XPLMProbeResult XPLMProbeTerrainXYZ(
    XPLMProbeRef         inProbe,
    float                inX,
    float                inY,
    float                inZ,
    XPLMProbeInfo_t *    outInfo);

/***************************************************************************
 * MAGNETIC VARIATION
 ***************************************************************************/

XPLM_API float XPLMGetMagneticVariation(
    double               latitude,
    double               longitude);

XPLM_API float XPLMDegTrueToDegMagnetic(float headingTrue);

XPLM_API float XPLMDegMagneticToDegTrue(float headingMagnetic);

#ifdef __cplusplus
}
#endif

#endif /* XPLMSCENERY_H */
