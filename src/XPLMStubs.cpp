/*
 * XPLM SDK Stub Implementations for Windows Build
 * 
 * These stub functions allow the plugin to compile and link on Windows
 * without requiring the actual XPLM SDK library. At runtime, X-Plane
 * will resolve these symbols to the actual implementations.
 * 
 * Only used on Windows where the linker requires all symbols to be resolved.
 */

#if defined(_WIN32) || defined(_WIN64)

/* Define XPLM_BUILDING so XPLM_API becomes dllexport for our stub definitions */
#define XPLM_BUILDING

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"
#include "XPLMInstance.h"

/* Stub implementations - these will be overridden by X-Plane at runtime */

/* XPLMDataAccess stubs */
XPLMDataRef XPLMFindDataRef(const char*) { return nullptr; }
int XPLMGetDatai(XPLMDataRef) { return 0; }
float XPLMGetDataf(XPLMDataRef) { return 0.0f; }
double XPLMGetDatad(XPLMDataRef) { return 0.0; }
void XPLMSetDatai(XPLMDataRef, int) {}
void XPLMSetDataf(XPLMDataRef, float) {}
void XPLMSetDatad(XPLMDataRef, double) {}

/* XPLMProcessing stubs */
XPLMFlightLoopID XPLMCreateFlightLoop(XPLMCreateFlightLoop_t*) { return nullptr; }
void XPLMDestroyFlightLoop(XPLMFlightLoopID) {}
void XPLMScheduleFlightLoop(XPLMFlightLoopID, float, int) {}

/* XPLMMenus stubs */
XPLMMenuID XPLMFindPluginsMenu(void) { return nullptr; }
XPLMMenuID XPLMCreateMenu(const char*, XPLMMenuID, int, XPLMMenuHandler_f, void*) { return nullptr; }
void XPLMDestroyMenu(XPLMMenuID) {}
int XPLMAppendMenuItem(XPLMMenuID, const char*, void*, int) { return 0; }
void XPLMEnableMenuItem(XPLMMenuID, int, int) {}

/* XPLMUtilities stubs */
void XPLMGetPluginDirectoryPath(char*) {}
void XPLMDebugString(const char*) {}
void XPLMSpeakString(const char*) {}

/* XPLMScenery stubs */
XPLMObjectRef XPLMLoadObject(const char*) { return nullptr; }
void XPLMUnloadObject(XPLMObjectRef) {}
XPLMProbeRef XPLMCreateProbe(XPLMProbeType) { return nullptr; }
void XPLMDestroyProbe(XPLMProbeRef) {}
XPLMProbeResult XPLMProbeTerrainXYZ(XPLMProbeRef, float, float, float, XPLMProbeInfo_t*) { return xplm_ProbeError; }

/* XPLMInstance stubs */
XPLMInstanceRef XPLMCreateInstance(XPLMObjectRef, const char**) { return nullptr; }
void XPLMDestroyInstance(XPLMInstanceRef) {}
void XPLMInstanceSetPosition(XPLMInstanceRef, const XPLMDrawInfo_t*, const float*) {}

/* XPLMGraphics stubs */
void XPLMSetGraphicsState(int, int, int, int, int, int, int) {}
int XPLMRegisterDrawCallback(XPLMDrawCallback_f, XPLMDrawingPhase, int, void*) { return 1; }
int XPLMUnregisterDrawCallback(XPLMDrawCallback_f, XPLMDrawingPhase, int, void*) { return 1; }

#endif /* _WIN32 || _WIN64 */
