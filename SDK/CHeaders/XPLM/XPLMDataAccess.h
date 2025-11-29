/*
 * X-Plane Data Access API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMDATAACCESS_H
#define XPLMDATAACCESS_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * DATA REFS
 ***************************************************************************/

typedef void * XPLMDataRef;

/* Data types for datarefs */
typedef int XPLMDataTypeID;
#define xplmType_Unknown    0
#define xplmType_Int        1
#define xplmType_Float      2
#define xplmType_Double     4
#define xplmType_FloatArray 8
#define xplmType_IntArray   16
#define xplmType_Data       32

/* Find a dataref */
XPLM_API XPLMDataRef XPLMFindDataRef(const char * inDataRefName);

/* Check if a dataref can be written */
XPLM_API int XPLMCanWriteDataRef(XPLMDataRef inDataRef);

/* Check if a dataref is good */
XPLM_API int XPLMIsDataRefGood(XPLMDataRef inDataRef);

/* Get the type of a dataref */
XPLM_API XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef inDataRef);

/***************************************************************************
 * DATA ACCESSORS
 ***************************************************************************/

/* Get/Set integer data */
XPLM_API int XPLMGetDatai(XPLMDataRef inDataRef);
XPLM_API void XPLMSetDatai(XPLMDataRef inDataRef, int inValue);

/* Get/Set float data */
XPLM_API float XPLMGetDataf(XPLMDataRef inDataRef);
XPLM_API void XPLMSetDataf(XPLMDataRef inDataRef, float inValue);

/* Get/Set double data */
XPLM_API double XPLMGetDatad(XPLMDataRef inDataRef);
XPLM_API void XPLMSetDatad(XPLMDataRef inDataRef, double inValue);

/* Get/Set integer array data */
XPLM_API int XPLMGetDatavi(
    XPLMDataRef          inDataRef,
    int *                outValues,
    int                  inOffset,
    int                  inMax);
XPLM_API void XPLMSetDatavi(
    XPLMDataRef          inDataRef,
    int *                inValues,
    int                  inOffset,
    int                  inCount);

/* Get/Set float array data */
XPLM_API int XPLMGetDatavf(
    XPLMDataRef          inDataRef,
    float *              outValues,
    int                  inOffset,
    int                  inMax);
XPLM_API void XPLMSetDatavf(
    XPLMDataRef          inDataRef,
    float *              inValues,
    int                  inOffset,
    int                  inCount);

/* Get/Set byte data */
XPLM_API int XPLMGetDatab(
    XPLMDataRef          inDataRef,
    void *               outValue,
    int                  inOffset,
    int                  inMaxBytes);
XPLM_API void XPLMSetDatab(
    XPLMDataRef          inDataRef,
    void *               inValue,
    int                  inOffset,
    int                  inLength);

/***************************************************************************
 * CUSTOM DATA ACCESSORS
 ***************************************************************************/

typedef int (* XPLMGetDatai_f)(void * inRefcon);
typedef void (* XPLMSetDatai_f)(void * inRefcon, int inValue);
typedef float (* XPLMGetDataf_f)(void * inRefcon);
typedef void (* XPLMSetDataf_f)(void * inRefcon, float inValue);
typedef double (* XPLMGetDatad_f)(void * inRefcon);
typedef void (* XPLMSetDatad_f)(void * inRefcon, double inValue);
typedef int (* XPLMGetDatavi_f)(void * inRefcon, int * outValues, int inOffset, int inMax);
typedef void (* XPLMSetDatavi_f)(void * inRefcon, int * inValues, int inOffset, int inCount);
typedef int (* XPLMGetDatavf_f)(void * inRefcon, float * outValues, int inOffset, int inMax);
typedef void (* XPLMSetDatavf_f)(void * inRefcon, float * inValues, int inOffset, int inCount);
typedef int (* XPLMGetDatab_f)(void * inRefcon, void * outValue, int inOffset, int inMaxLength);
typedef void (* XPLMSetDatab_f)(void * inRefcon, void * inValue, int inOffset, int inLength);

XPLM_API XPLMDataRef XPLMRegisterDataAccessor(
    const char *         inDataName,
    XPLMDataTypeID       inDataType,
    int                  inIsWritable,
    XPLMGetDatai_f       inReadInt,
    XPLMSetDatai_f       inWriteInt,
    XPLMGetDataf_f       inReadFloat,
    XPLMSetDataf_f       inWriteFloat,
    XPLMGetDatad_f       inReadDouble,
    XPLMSetDatad_f       inWriteDouble,
    XPLMGetDatavi_f      inReadIntArray,
    XPLMSetDatavi_f      inWriteIntArray,
    XPLMGetDatavf_f      inReadFloatArray,
    XPLMSetDatavf_f      inWriteFloatArray,
    XPLMGetDatab_f       inReadData,
    XPLMSetDatab_f       inWriteData,
    void *               inReadRefcon,
    void *               inWriteRefcon);

XPLM_API void XPLMUnregisterDataAccessor(XPLMDataRef inDataRef);

/***************************************************************************
 * DATA SHARING
 ***************************************************************************/

typedef void (* XPLMDataChanged_f)(void * inRefcon);

XPLM_API int XPLMShareData(
    const char *         inDataName,
    XPLMDataTypeID       inDataType,
    XPLMDataChanged_f    inNotificationFunc,
    void *               inNotificationRefcon);

XPLM_API int XPLMUnshareData(
    const char *         inDataName,
    XPLMDataTypeID       inDataType,
    XPLMDataChanged_f    inNotificationFunc,
    void *               inNotificationRefcon);

#ifdef __cplusplus
}
#endif

#endif /* XPLMDATAACCESS_H */
