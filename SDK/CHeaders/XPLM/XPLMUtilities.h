/*
 * X-Plane Utilities API
 * Copyright (c) Laminar Research
 */

#ifndef XPLMUTILITIES_H
#define XPLMUTILITIES_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
 * X-PLANE VERSION
 ***************************************************************************/

XPLM_API void XPLMGetVersions(
    int *                outXPlaneVersion,
    int *                outXPLMVersion,
    XPLMPluginID *       outHostID);

XPLM_API int XPLMGetDirectorySeparator(void);

/***************************************************************************
 * SYSTEM PATH
 ***************************************************************************/

XPLM_API void XPLMGetSystemPath(char * outSystemPath);

XPLM_API void XPLMGetPrefsPath(char * outPrefsPath);

/* Extract the directory from a full path */
XPLM_API void XPLMExtractFileAndPath(char * inFullPath);

XPLM_API const char * XPLMGetDirectorySeparatorStr(void);

/***************************************************************************
 * PLUGIN DIRECTORY
 ***************************************************************************/

XPLM_API void XPLMGetPluginDirectoryPath(char * outPath);

/***************************************************************************
 * DEBUG STRING
 ***************************************************************************/

XPLM_API void XPLMDebugString(const char * inString);

/***************************************************************************
 * ERROR CALLBACK
 ***************************************************************************/

typedef void (* XPLMError_f)(const char * inMessage);

XPLM_API void XPLMSetErrorCallback(XPLMError_f inCallback);

/***************************************************************************
 * COMMANDS
 ***************************************************************************/

typedef void * XPLMCommandRef;

typedef int XPLMCommandPhase;
#define xplm_CommandBegin    0
#define xplm_CommandContinue 1
#define xplm_CommandEnd      2

typedef int (* XPLMCommandCallback_f)(
    XPLMCommandRef       inCommand,
    XPLMCommandPhase     inPhase,
    void *               inRefcon);

XPLM_API XPLMCommandRef XPLMFindCommand(const char * inName);

XPLM_API void XPLMCommandBegin(XPLMCommandRef inCommand);

XPLM_API void XPLMCommandEnd(XPLMCommandRef inCommand);

XPLM_API void XPLMCommandOnce(XPLMCommandRef inCommand);

XPLM_API XPLMCommandRef XPLMCreateCommand(
    const char *         inName,
    const char *         inDescription);

XPLM_API void XPLMRegisterCommandHandler(
    XPLMCommandRef       inCommand,
    XPLMCommandCallback_f inHandler,
    int                  inBefore,
    void *               inRefcon);

XPLM_API void XPLMUnregisterCommandHandler(
    XPLMCommandRef       inCommand,
    XPLMCommandCallback_f inHandler,
    int                  inBefore,
    void *               inRefcon);

/***************************************************************************
 * LOCALE AND LANGUAGE
 ***************************************************************************/

typedef int XPLMLanguageCode;
#define xplm_Language_Unknown    0
#define xplm_Language_English    1
#define xplm_Language_French     2
#define xplm_Language_German     3
#define xplm_Language_Italian    4
#define xplm_Language_Spanish    5
#define xplm_Language_Korean     6
#define xplm_Language_Russian    7
#define xplm_Language_Greek      8
#define xplm_Language_Japanese   9
#define xplm_Language_Chinese   10

XPLM_API XPLMLanguageCode XPLMGetLanguage(void);

/***************************************************************************
 * SOUND
 ***************************************************************************/

XPLM_API void XPLMSpeakString(const char * inString);

/***************************************************************************
 * LOAD/RELOAD SCENERY
 ***************************************************************************/

XPLM_API void XPLMReloadScenery(void);

#ifdef __cplusplus
}
#endif

#endif /* XPLMUTILITIES_H */
