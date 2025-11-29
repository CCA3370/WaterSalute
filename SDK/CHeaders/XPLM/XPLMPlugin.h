/*
 * X-Plane Plugin API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMPLUGIN_H
#define XPLMPLUGIN_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * PLUGIN MANAGEMENT
 ***************************************************************************/

/* Get information about a plugin */
XPLM_API void XPLMGetPluginInfo(
    XPLMPluginID         inPlugin,
    char *               outName,
    char *               outFilePath,
    char *               outSignature,
    char *               outDescription);

/* Get the plugin ID of this plugin */
XPLM_API XPLMPluginID XPLMGetMyID(void);

/* Count loaded plugins */
XPLM_API int XPLMCountPlugins(void);

/* Get a plugin by index */
XPLM_API XPLMPluginID XPLMGetNthPlugin(int inIndex);

/* Find a plugin by signature */
XPLM_API XPLMPluginID XPLMFindPluginBySignature(const char * inSignature);

/* Reload a plugin */
XPLM_API void XPLMReloadPlugins(void);

/* Send a message to a plugin */
XPLM_API void XPLMSendMessageToPlugin(
    XPLMPluginID         inPlugin,
    int                  inMessage,
    void *               inParam);

/* Check if a feature is enabled */
XPLM_API int XPLMHasFeature(const char * inFeature);

/* Check if a feature is enabled */
XPLM_API int XPLMIsFeatureEnabled(const char * inFeature);

/* Enable a feature */
XPLM_API void XPLMEnableFeature(const char * inFeature, int inEnable);

/***************************************************************************
 * PLUGIN MESSAGE IDS
 ***************************************************************************/

#define XPLM_MSG_PLANE_CRASHED          101
#define XPLM_MSG_PLANE_LOADED           102
#define XPLM_MSG_AIRPORT_LOADED         103
#define XPLM_MSG_SCENERY_LOADED         104
#define XPLM_MSG_AIRPLANE_COUNT_CHANGED 105
#define XPLM_MSG_PLANE_UNLOADED         106
#define XPLM_MSG_WILL_WRITE_PREFS       107
#define XPLM_MSG_LIVERY_LOADED          108
#define XPLM_MSG_ENTERED_VR             109
#define XPLM_MSG_EXITING_VR             110
#define XPLM_MSG_RELEASE_PLANES         111
#define XPLM_MSG_FMOD_BANK_LOADED       112
#define XPLM_MSG_FMOD_BANK_UNLOADING    113

#ifdef __cplusplus
}
#endif

#endif /* XPLMPLUGIN_H */
