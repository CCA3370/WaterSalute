/*
 * X-Plane Graphics API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMGRAPHICS_H
#define XPLMGRAPHICS_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * COORDINATE CONVERSION
 ***************************************************************************/

/* Convert local coordinates to lat/lon */
XPLM_API void XPLMLocalToWorld(
    double               inX,
    double               inY,
    double               inZ,
    double *             outLatitude,
    double *             outLongitude,
    double *             outAltitude);

/* Convert lat/lon to local coordinates */
XPLM_API void XPLMWorldToLocal(
    double               inLatitude,
    double               inLongitude,
    double               inAltitude,
    double *             outX,
    double *             outY,
    double *             outZ);

/***************************************************************************
 * DRAWING CALLBACKS
 ***************************************************************************/

typedef int XPLMDrawingPhase;
#define xplm_Phase_Modern3D     31

typedef int (* XPLMDrawCallback_f)(
    XPLMDrawingPhase     inPhase,
    int                  inIsBefore,
    void *               inRefcon);

XPLM_API int XPLMRegisterDrawCallback(
    XPLMDrawCallback_f   inCallback,
    XPLMDrawingPhase     inPhase,
    int                  inWantsBefore,
    void *               inRefcon);

XPLM_API int XPLMUnregisterDrawCallback(
    XPLMDrawCallback_f   inCallback,
    XPLMDrawingPhase     inPhase,
    int                  inWantsBefore,
    void *               inRefcon);

/***************************************************************************
 * TEXTURE MANAGEMENT
 ***************************************************************************/

typedef int XPLMTextureID;
#define xplm_Tex_GeneralInterface 0

XPLM_API void XPLMBindTexture2d(
    int                  inTextureNum,
    int                  inTextureUnit);

XPLM_API int XPLMGenerateTextureNumbers(
    int *                outTextureIDs,
    int                  inCount);

XPLM_API void XPLMSetGraphicsState(
    int                  inEnableFog,
    int                  inNumberTexUnits,
    int                  inEnableLighting,
    int                  inEnableAlphaTesting,
    int                  inEnableAlphaBlending,
    int                  inEnableDepthTesting,
    int                  inEnableDepthWriting);

/***************************************************************************
 * FOG CONTROL
 ***************************************************************************/

typedef int XPLMFogStyle;
#define xplm_Fog_Default  0
#define xplm_Fog_Disable  1

XPLM_API void XPLMSetFogStyle(XPLMFogStyle inStyle);

#ifdef __cplusplus
}
#endif

#endif /* XPLMGRAPHICS_H */
