/*
 * X-Plane SDK Definitions
 * Copyright (c) Laminar Research
 */

#ifndef XPLMDEFS_H
#define XPLMDEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Platform detection */
#if defined(_WIN32) || defined(_WIN64)
    #define IBM 1
    #define APL 0
    #define LIN 0
#elif defined(__APPLE__)
    #define IBM 0
    #define APL 1
    #define LIN 0
#else
    #define IBM 0
    #define APL 0
    #define LIN 1
#endif

/* API version */
#define XPLM_API_VERSION 420

/* Calling conventions */
#if IBM
    #define XPLM_API __declspec(dllexport)
#else
    #define XPLM_API __attribute__((visibility("default")))
#endif

/* Basic types */
typedef void * XPLMPluginID;
typedef int XPLMKeyFlags;
typedef int XPLMMouseStatus;
typedef int XPLMCursorStatus;

/* Flight loop callback */
typedef float (* XPLMFlightLoop_f)(
    float                    inElapsedSinceLastCall,
    float                    inElapsedTimeSinceLastFlightLoop,
    int                      inCounter,
    void *                   inRefcon);

/* Key flags */
#define xplm_ShiftFlag      1
#define xplm_OptionAltFlag  2
#define xplm_ControlFlag    4
#define xplm_DownFlag       8
#define xplm_UpFlag         16

#ifdef __cplusplus
}
#endif

#endif /* XPLMDEFS_H */
