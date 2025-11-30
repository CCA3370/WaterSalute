/*
 * Water Salute Plugin for X-Plane 12
 * 
 * This plugin simulates a water salute ceremony where two fire trucks
 * approach the aircraft and spray water arches over it.
 * 
 * Features:
 * - Menu system with Start/Stop controls
 * - Aircraft ground and speed validation
 * - Fire truck positioning based on aircraft wingspan
 * - Particle-based water effects
 * - External OBJ model loading for fire trucks
 * 
 * Copyright (c) 2024
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <vector>
#include <string>
#include <algorithm>

#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMScenery.h"
#include "XPLMInstance.h"

/* Constants */
static const float KNOTS_TO_MS = 0.514444f;        /* Knots to m/s conversion */
static const float MAX_GROUND_SPEED_KNOTS = 40.0f; /* Maximum ground speed for water salute */
static const float TRUCK_APPROACH_SPEED = 15.0f;   /* Fire truck approach speed in m/s */
static const float TRUCK_STOP_DISTANCE = 200.0f;   /* Distance in front of aircraft to stop (meters) */
static const float TRUCK_EXTRA_SPACING = 40.0f;    /* Extra spacing beyond wingspan (meters) */
static const float WATER_JET_HEIGHT = 25.0f;       /* Maximum height of water arch (meters) */
static const float WATER_JET_DURATION = 0.5f;      /* Time for particle to reach apex */
static const float PARTICLE_LIFETIME = 2.0f;       /* Particle lifetime in seconds */
static const int   NUM_PARTICLES_PER_JET = 100;    /* Number of particles per water jet */
static const float PARTICLE_EMIT_RATE = 0.02f;     /* Time between particle emissions (seconds) */
static const XPLMDrawingPhase WATER_DRAWING_PHASE = xplm_Phase_Modern3D; /* Drawing phase for water particles */

/* Debug configuration */
static const bool DEBUG_VERBOSE = true;            /* Enable verbose debug logging */
static const float DEBUG_LOG_INTERVAL = 2.0f;      /* Interval for periodic debug logs (seconds) */

/* Plugin state */
enum PluginState {
    STATE_IDLE,
    STATE_TRUCKS_APPROACHING,
    STATE_TRUCKS_POSITIONING,
    STATE_WATER_SPRAYING,
    STATE_TRUCKS_LEAVING
};

/* Water particle */
struct WaterParticle {
    float x, y, z;           /* Current position */
    float vx, vy, vz;        /* Velocity */
    float lifetime;          /* Remaining lifetime */
    float maxLifetime;       /* Original lifetime */
    bool active;
};

/* Fire truck */
struct FireTruck {
    XPLMInstanceRef instance;
    double x, y, z;          /* Current position (OpenGL coords) */
    float heading;           /* Current heading in degrees */
    float targetX, targetZ;  /* Target position */
    float targetHeading;     /* Target heading */
    bool positioned;         /* Has reached target position */
    std::vector<WaterParticle> particles;
    float lastEmitTime;      /* Time of last particle emission */
    float nozzleOffsetX;     /* Nozzle position offset from truck center */
    float nozzleOffsetY;     /* Nozzle height */
    float nozzleOffsetZ;     /* Nozzle forward offset */
};

/* Global variables */
static PluginState g_state = STATE_IDLE;
static XPLMMenuID g_menuId = nullptr;
static int g_menuStartItem = -1;
static int g_menuStopItem = -1;

static XPLMObjectRef g_truckObject = nullptr;
static FireTruck g_leftTruck;
static FireTruck g_rightTruck;
static XPLMProbeRef g_terrainProbe = nullptr;

static XPLMFlightLoopID g_flightLoopId = nullptr;

static bool g_drawCallbackRegistered = false;

/* Datarefs */
static XPLMDataRef g_drOnGround = nullptr;
static XPLMDataRef g_drGroundSpeed = nullptr;
static XPLMDataRef g_drLocalX = nullptr;
static XPLMDataRef g_drLocalY = nullptr;
static XPLMDataRef g_drLocalZ = nullptr;
static XPLMDataRef g_drHeading = nullptr;
static XPLMDataRef g_drWingspan = nullptr;

static char g_pluginPath[512];
static float g_debugLogTimer = 0.0f;               /* Timer for periodic debug logging */
static int g_drawCallbackCallCount = 0;            /* Counter for draw callback calls */
static int g_particlesEmittedTotal = 0;            /* Total particles emitted */

/* Forward declarations */
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);
static void MenuHandler(void* inMenuRef, void* inItemRef);
static void StartWaterSalute();
static void StopWaterSalute();
static void UpdateTrucks(float dt);
static void UpdateWaterParticles(float dt);
static void EmitParticle(FireTruck& truck);
static void UpdateMenuState();
static float GetTerrainHeight(float x, float z);
static void InitializeTruck(FireTruck& truck);
static void CleanupTruck(FireTruck& truck);
static bool LoadFireTruckModel();
static int DrawWaterParticles(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);
static void RegisterDrawCallback();
static void UnregisterDrawCallback();

/* Debug logging */
static void DebugLog(const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char logMsg[1100];
    snprintf(logMsg, sizeof(logMsg), "WaterSalute: %s\n", buffer);
    XPLMDebugString(logMsg);
}

/* Get state name as string for debugging */
static const char* GetStateName(PluginState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_TRUCKS_APPROACHING: return "TRUCKS_APPROACHING";
        case STATE_TRUCKS_POSITIONING: return "TRUCKS_POSITIONING";
        case STATE_WATER_SPRAYING: return "WATER_SPRAYING";
        case STATE_TRUCKS_LEAVING: return "TRUCKS_LEAVING";
        default: return "UNKNOWN";
    }
}

/* Verbose debug logging (only when DEBUG_VERBOSE is enabled) */
static void DebugLogVerbose(const char* format, ...) {
    if (!DEBUG_VERBOSE) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char logMsg[1100];
    snprintf(logMsg, sizeof(logMsg), "WaterSalute [VERBOSE]: %s\n", buffer);
    XPLMDebugString(logMsg);
}

/* Log periodic debug status */
static void LogDebugStatus() {
    DebugLog("=== DEBUG STATUS ===");
    DebugLog("  State: %s (%d)", GetStateName(g_state), g_state);
    DebugLog("  Truck Object Loaded: %s", g_truckObject ? "YES" : "NO");
    DebugLog("  Left Truck Instance: %s", g_leftTruck.instance ? "YES" : "NO");
    DebugLog("  Right Truck Instance: %s", g_rightTruck.instance ? "YES" : "NO");
    DebugLog("  Left Truck Position: (%.2f, %.2f, %.2f), Heading: %.1f, Positioned: %s",
             g_leftTruck.x, g_leftTruck.y, g_leftTruck.z, g_leftTruck.heading,
             g_leftTruck.positioned ? "YES" : "NO");
    DebugLog("  Right Truck Position: (%.2f, %.2f, %.2f), Heading: %.1f, Positioned: %s",
             g_rightTruck.x, g_rightTruck.y, g_rightTruck.z, g_rightTruck.heading,
             g_rightTruck.positioned ? "YES" : "NO");
    DebugLog("  Left Truck Particles: %zu", g_leftTruck.particles.size());
    DebugLog("  Right Truck Particles: %zu", g_rightTruck.particles.size());
    DebugLog("  Draw Callback Registered: %s", g_drawCallbackRegistered ? "YES" : "NO");
    DebugLog("  Draw Callback Call Count: %d", g_drawCallbackCallCount);
    DebugLog("  Total Particles Emitted: %d", g_particlesEmittedTotal);
    DebugLog("===================");
}

/*
 * XPluginStart - Called when the plugin is loaded
 */
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "Water Salute");
    strcpy(outSig, "com.xplane.watersalute");
    strcpy(outDesc, "Water salute ceremony with fire trucks");
    
    /* Get plugin path using XPLMGetPluginInfo */
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, g_pluginPath, nullptr, nullptr);
    /* Extract directory path from full file path */
    char* lastSlash = strrchr(g_pluginPath, '/');
    if (!lastSlash) lastSlash = strrchr(g_pluginPath, '\\');
    if (lastSlash) *lastSlash = '\0';
    
    DebugLog("Plugin starting...");
    
    /* Initialize datarefs */
    g_drOnGround = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    g_drGroundSpeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    g_drLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
    g_drLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
    g_drLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    g_drHeading = XPLMFindDataRef("sim/flightmodel/position/psi");
    
    /* Find wingspan dataref - try several possible datarefs */
    g_drWingspan = XPLMFindDataRef("sim/aircraft/parts/acf_wing_span");
    if (!g_drWingspan) {
        g_drWingspan = XPLMFindDataRef("sim/aircraft/view/acf_wing_span");
    }
    /* Note: If wingspan dataref not found, a default value of 30m is used in StartWaterSalute */
    
    /* Create terrain probe */
    g_terrainProbe = XPLMCreateProbe(xplm_ProbeY);
    
    /* Create menu */
    int menuContainerItem = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Water Salute", nullptr, 0);
    g_menuId = XPLMCreateMenu("Water Salute", XPLMFindPluginsMenu(), menuContainerItem, MenuHandler, nullptr);
    
    g_menuStartItem = XPLMAppendMenuItem(g_menuId, "Start Water Salute", (void*)"start", 0);
    g_menuStopItem = XPLMAppendMenuItem(g_menuId, "Stop", (void*)"stop", 0);
    
    UpdateMenuState();
    
    /* Create flight loop */
    XPLMCreateFlightLoop_t flightLoopParams;
    flightLoopParams.structSize = sizeof(XPLMCreateFlightLoop_t);
    flightLoopParams.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    flightLoopParams.callbackFunc = FlightLoopCallback;
    flightLoopParams.refcon = nullptr;
    
    g_flightLoopId = XPLMCreateFlightLoop(&flightLoopParams);
    XPLMScheduleFlightLoop(g_flightLoopId, -1.0f, 1);
    
    /* Load fire truck model */
    LoadFireTruckModel();
    
    DebugLog("Plugin started successfully");
    
    return 1;
}

/*
 * XPluginStop - Called when the plugin is unloaded
 */
PLUGIN_API void XPluginStop(void) {
    DebugLog("Plugin stopping...");
    
    /* Cleanup */
    UnregisterDrawCallback();
    
    if (g_flightLoopId) {
        XPLMDestroyFlightLoop(g_flightLoopId);
        g_flightLoopId = nullptr;
    }
    
    CleanupTruck(g_leftTruck);
    CleanupTruck(g_rightTruck);
    
    if (g_truckObject) {
        XPLMUnloadObject(g_truckObject);
        g_truckObject = nullptr;
    }
    
    if (g_terrainProbe) {
        XPLMDestroyProbe(g_terrainProbe);
        g_terrainProbe = nullptr;
    }
    
    if (g_menuId) {
        XPLMDestroyMenu(g_menuId);
        g_menuId = nullptr;
    }
    
    DebugLog("Plugin stopped");
}

/*
 * XPluginEnable - Called when the plugin is enabled
 */
PLUGIN_API int XPluginEnable(void) {
    DebugLog("Plugin enabled");
    return 1;
}

/*
 * XPluginDisable - Called when the plugin is disabled
 */
PLUGIN_API void XPluginDisable(void) {
    DebugLog("Plugin disabled");
    StopWaterSalute();
}

/*
 * XPluginReceiveMessage - Handle messages from X-Plane
 */
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) {
    (void)inFrom;
    (void)inParam;
    
    if (inMsg == XPLM_MSG_PLANE_LOADED || inMsg == XPLM_MSG_AIRPORT_LOADED) {
        /* Reset state when plane or airport changes */
        if (g_state != STATE_IDLE) {
            StopWaterSalute();
        }
    }
}

/*
 * LoadFireTruckModel - Load the fire truck OBJ file
 */
static bool LoadFireTruckModel() {
    /* Build path to fire truck model */
    char modelPath[1024];
    snprintf(modelPath, sizeof(modelPath), "%s/resources/firetruck.obj", g_pluginPath);
    
    DebugLog("Loading fire truck model from: %s", modelPath);
    DebugLog("Plugin path base: %s", g_pluginPath);
    
    /* Check if file exists by trying to open it */
    FILE* testFile = fopen(modelPath, "r");
    if (testFile) {
        fclose(testFile);
        DebugLog("Model file exists and is readable");
    } else {
        DebugLog("WARNING: Cannot open model file - errno may indicate cause");
    }
    
    g_truckObject = XPLMLoadObject(modelPath);
    
    if (!g_truckObject) {
        DebugLog("ERROR: XPLMLoadObject returned NULL!");
        DebugLog("  This could mean:");
        DebugLog("  1. File path is incorrect");
        DebugLog("  2. OBJ file format is invalid");
        DebugLog("  3. Texture file is missing");
        DebugLog("  4. X-Plane cannot find the resource");
        return false;
    }
    
    DebugLog("Fire truck model loaded successfully (handle: %p)", (void*)g_truckObject);
    return true;
}

/*
 * MenuHandler - Handle menu clicks
 */
static void MenuHandler(void* inMenuRef, void* inItemRef) {
    (void)inMenuRef;
    
    const char* item = static_cast<const char*>(inItemRef);
    
    if (strcmp(item, "start") == 0) {
        StartWaterSalute();
    } else if (strcmp(item, "stop") == 0) {
        StopWaterSalute();
    }
}

/*
 * UpdateMenuState - Update menu item enabled states
 */
static void UpdateMenuState() {
    bool canStart = (g_state == STATE_IDLE);
    /* Allow stopping during any active state (approaching, positioning, or spraying) */
    bool canStop = (g_state != STATE_IDLE && g_state != STATE_TRUCKS_LEAVING);
    
    XPLMEnableMenuItem(g_menuId, g_menuStartItem, canStart ? 1 : 0);
    XPLMEnableMenuItem(g_menuId, g_menuStopItem, canStop ? 1 : 0);
}

/*
 * StartWaterSalute - Begin the water salute ceremony
 */
static void StartWaterSalute() {
    DebugLog("========================================");
    DebugLog("StartWaterSalute called");
    DebugLog("========================================");
    
    /* Reset debug counters */
    g_drawCallbackCallCount = 0;
    g_particlesEmittedTotal = 0;
    g_debugLogTimer = 0.0f;
    
    if (g_state != STATE_IDLE) {
        DebugLog("Cannot start - not in idle state (current: %s)", GetStateName(g_state));
        return;
    }
    
    /* Check if truck model is loaded */
    DebugLog("Pre-flight check: Truck object loaded = %s (handle: %p)", 
             g_truckObject ? "YES" : "NO", (void*)g_truckObject);
    
    /* Check datarefs availability */
    DebugLog("Datarefs status:");
    DebugLog("  g_drOnGround: %s", g_drOnGround ? "VALID" : "NULL");
    DebugLog("  g_drGroundSpeed: %s", g_drGroundSpeed ? "VALID" : "NULL");
    DebugLog("  g_drLocalX: %s", g_drLocalX ? "VALID" : "NULL");
    DebugLog("  g_drLocalY: %s", g_drLocalY ? "VALID" : "NULL");
    DebugLog("  g_drLocalZ: %s", g_drLocalZ ? "VALID" : "NULL");
    DebugLog("  g_drHeading: %s", g_drHeading ? "VALID" : "NULL");
    DebugLog("  g_drWingspan: %s", g_drWingspan ? "VALID" : "NULL");
    
    /* Check if aircraft is on ground */
    int onGround = 0;
    if (g_drOnGround) {
        onGround = XPLMGetDatai(g_drOnGround);
    }
    DebugLog("On ground check: %d", onGround);
    
    if (!onGround) {
        DebugLog("Cannot start - aircraft not on ground");
        XPLMSpeakString("Water salute requires aircraft to be on ground");
        return;
    }
    
    /* Check ground speed */
    float groundSpeed = 0.0f;
    if (g_drGroundSpeed) {
        groundSpeed = XPLMGetDataf(g_drGroundSpeed);
    }
    
    float groundSpeedKnots = groundSpeed / KNOTS_TO_MS;
    DebugLog("Ground speed: %.2f m/s (%.1f knots)", groundSpeed, groundSpeedKnots);
    
    if (groundSpeedKnots > MAX_GROUND_SPEED_KNOTS) {
        DebugLog("Cannot start - ground speed too high: %.1f knots", groundSpeedKnots);
        XPLMSpeakString("Water salute requires ground speed below 40 knots");
        return;
    }
    
    /* Get aircraft position and heading */
    double acX = g_drLocalX ? XPLMGetDatad(g_drLocalX) : 0.0;
    double acY = g_drLocalY ? XPLMGetDatad(g_drLocalY) : 0.0;
    double acZ = g_drLocalZ ? XPLMGetDatad(g_drLocalZ) : 0.0;
    float acHeading = g_drHeading ? XPLMGetDataf(g_drHeading) : 0.0f;
    
    /* Get aircraft wingspan (default to 30 meters if not available) */
    float wingspan = 30.0f;
    if (g_drWingspan) {
        wingspan = XPLMGetDataf(g_drWingspan);
        DebugLog("Raw wingspan dataref value: %.2f", wingspan);
        if (wingspan <= 0.0f || wingspan > 100.0f) {
            DebugLog("Invalid wingspan value, using default 30m");
            wingspan = 30.0f; /* Use default for invalid values */
        }
    } else {
        DebugLog("Wingspan dataref not found, using default 30m");
    }
    
    DebugLog("Aircraft position: (%.2f, %.2f, %.2f)", acX, acY, acZ);
    DebugLog("Aircraft heading: %.1f degrees", acHeading);
    DebugLog("Aircraft wingspan: %.1f meters", wingspan);
    
    /* Calculate truck spacing (wingspan + 40 meters) */
    float truckSpacing = (wingspan / 2.0f) + (TRUCK_EXTRA_SPACING / 2.0f);
    DebugLog("Truck spacing from center: %.1f meters", truckSpacing);
    
    /* Convert heading to radians */
    float headingRad = acHeading * (3.14159265f / 180.0f);
    
    /* Calculate forward vector (X-Plane uses -Z as forward) */
    float forwardX = -sinf(headingRad);
    float forwardZ = -cosf(headingRad);
    DebugLog("Forward vector: (%.4f, %.4f)", forwardX, forwardZ);
    
    /* Calculate right vector */
    float rightX = cosf(headingRad);
    float rightZ = -sinf(headingRad);
    DebugLog("Right vector: (%.4f, %.4f)", rightX, rightZ);
    
    /* Calculate truck start positions (500 meters ahead) */
    float startDistance = 500.0f;
    float startX = static_cast<float>(acX) + forwardX * startDistance;
    float startZ = static_cast<float>(acZ) + forwardZ * startDistance;
    DebugLog("Truck start position (center): (%.2f, %.2f)", startX, startZ);
    
    /* Calculate truck target positions (200 meters ahead) */
    float targetX = static_cast<float>(acX) + forwardX * TRUCK_STOP_DISTANCE;
    float targetZ = static_cast<float>(acZ) + forwardZ * TRUCK_STOP_DISTANCE;
    DebugLog("Truck target position (center): (%.2f, %.2f)", targetX, targetZ);
    
    /* Initialize left truck */
    InitializeTruck(g_leftTruck);
    g_leftTruck.x = startX - rightX * truckSpacing;
    g_leftTruck.z = startZ - rightZ * truckSpacing;
    g_leftTruck.y = GetTerrainHeight(static_cast<float>(g_leftTruck.x), static_cast<float>(g_leftTruck.z));
    g_leftTruck.heading = acHeading + 180.0f; /* Facing toward aircraft */
    g_leftTruck.targetX = targetX - rightX * truckSpacing;
    g_leftTruck.targetZ = targetZ - rightZ * truckSpacing;
    g_leftTruck.targetHeading = acHeading + 90.0f; /* Face toward center */
    g_leftTruck.positioned = false;
    g_leftTruck.nozzleOffsetX = 0.0f;
    g_leftTruck.nozzleOffsetY = 3.5f; /* Nozzle height on truck */
    g_leftTruck.nozzleOffsetZ = 2.0f; /* Forward offset */
    
    DebugLog("Left truck initialized:");
    DebugLog("  Start: (%.2f, %.2f, %.2f)", g_leftTruck.x, g_leftTruck.y, g_leftTruck.z);
    DebugLog("  Target: (%.2f, %.2f)", g_leftTruck.targetX, g_leftTruck.targetZ);
    DebugLog("  Heading: %.1f -> %.1f", g_leftTruck.heading, g_leftTruck.targetHeading);
    
    /* Initialize right truck */
    InitializeTruck(g_rightTruck);
    g_rightTruck.x = startX + rightX * truckSpacing;
    g_rightTruck.z = startZ + rightZ * truckSpacing;
    g_rightTruck.y = GetTerrainHeight(static_cast<float>(g_rightTruck.x), static_cast<float>(g_rightTruck.z));
    g_rightTruck.heading = acHeading + 180.0f; /* Facing toward aircraft */
    g_rightTruck.targetX = targetX + rightX * truckSpacing;
    g_rightTruck.targetZ = targetZ + rightZ * truckSpacing;
    g_rightTruck.targetHeading = acHeading - 90.0f; /* Face toward center */
    g_rightTruck.positioned = false;
    g_rightTruck.nozzleOffsetX = 0.0f;
    g_rightTruck.nozzleOffsetY = 3.5f;
    g_rightTruck.nozzleOffsetZ = 2.0f;
    
    DebugLog("Right truck initialized:");
    DebugLog("  Start: (%.2f, %.2f, %.2f)", g_rightTruck.x, g_rightTruck.y, g_rightTruck.z);
    DebugLog("  Target: (%.2f, %.2f)", g_rightTruck.targetX, g_rightTruck.targetZ);
    DebugLog("  Heading: %.1f -> %.1f", g_rightTruck.heading, g_rightTruck.targetHeading);
    
    /* Create truck instances if model loaded */
    if (g_truckObject) {
        DebugLog("Creating truck instances from loaded model...");
        const char* noDataRefs[] = { nullptr };
        g_leftTruck.instance = XPLMCreateInstance(g_truckObject, noDataRefs);
        g_rightTruck.instance = XPLMCreateInstance(g_truckObject, noDataRefs);
        DebugLog("Left truck instance: %s (handle: %p)", 
                 g_leftTruck.instance ? "CREATED" : "FAILED", (void*)g_leftTruck.instance);
        DebugLog("Right truck instance: %s (handle: %p)", 
                 g_rightTruck.instance ? "CREATED" : "FAILED", (void*)g_rightTruck.instance);
        
        /* Set initial positions for instances */
        if (g_leftTruck.instance) {
            XPLMDrawInfo_t drawInfo;
            drawInfo.structSize = sizeof(XPLMDrawInfo_t);
            drawInfo.x = static_cast<float>(g_leftTruck.x);
            drawInfo.y = static_cast<float>(g_leftTruck.y);
            drawInfo.z = static_cast<float>(g_leftTruck.z);
            drawInfo.pitch = 0.0f;
            drawInfo.heading = g_leftTruck.heading;
            drawInfo.roll = 0.0f;
            XPLMInstanceSetPosition(g_leftTruck.instance, &drawInfo, nullptr);
            DebugLog("Left truck initial position set");
        }
        if (g_rightTruck.instance) {
            XPLMDrawInfo_t drawInfo;
            drawInfo.structSize = sizeof(XPLMDrawInfo_t);
            drawInfo.x = static_cast<float>(g_rightTruck.x);
            drawInfo.y = static_cast<float>(g_rightTruck.y);
            drawInfo.z = static_cast<float>(g_rightTruck.z);
            drawInfo.pitch = 0.0f;
            drawInfo.heading = g_rightTruck.heading;
            drawInfo.roll = 0.0f;
            XPLMInstanceSetPosition(g_rightTruck.instance, &drawInfo, nullptr);
            DebugLog("Right truck initial position set");
        }
    } else {
        DebugLog("WARNING: No truck model loaded - trucks will NOT be visible!");
        DebugLog("  Check that firetruck.obj exists in the resources folder");
    }
    
    g_state = STATE_TRUCKS_APPROACHING;
    UpdateMenuState();
    
    DebugLog("State changed to: %s", GetStateName(g_state));
    DebugLog("Water salute started - trucks approaching");
    DebugLog("========================================");
}

/*
 * StopWaterSalute - Stop the water salute and have trucks leave
 */
static void StopWaterSalute() {
    DebugLog("StopWaterSalute called, current state: %d", g_state);
    
    if (g_state == STATE_WATER_SPRAYING) {
        UnregisterDrawCallback();
        g_state = STATE_TRUCKS_LEAVING;
        DebugLog("Stopping water and trucks leaving");
    } else if (g_state != STATE_IDLE) {
        /* Immediate cleanup for other states */
        CleanupTruck(g_leftTruck);
        CleanupTruck(g_rightTruck);
        UnregisterDrawCallback();
        g_state = STATE_IDLE;
        DebugLog("Water salute cancelled");
    }
    
    UpdateMenuState();
}

/*
 * InitializeTruck - Initialize a fire truck structure
 */
static void InitializeTruck(FireTruck& truck) {
    truck.instance = nullptr;
    truck.x = truck.y = truck.z = 0.0;
    truck.heading = 0.0f;
    truck.targetX = truck.targetZ = 0.0f;
    truck.targetHeading = 0.0f;
    truck.positioned = false;
    truck.particles.clear();
    truck.particles.reserve(NUM_PARTICLES_PER_JET);
    truck.lastEmitTime = 0.0f;
}

/*
 * CleanupTruck - Clean up a fire truck
 */
static void CleanupTruck(FireTruck& truck) {
    if (truck.instance) {
        XPLMDestroyInstance(truck.instance);
        truck.instance = nullptr;
    }
    truck.particles.clear();
}

/*
 * GetTerrainHeight - Get terrain height at a position
 */
static float GetTerrainHeight(float x, float z) {
    if (!g_terrainProbe) {
        return 0.0f;
    }
    
    XPLMProbeInfo_t probeInfo;
    probeInfo.structSize = sizeof(XPLMProbeInfo_t);
    
    XPLMProbeResult result = XPLMProbeTerrainXYZ(g_terrainProbe, x, 10000.0f, z, &probeInfo);
    
    if (result == xplm_ProbeHitTerrain) {
        return probeInfo.locationY;
    }
    
    return 0.0f;
}

/*
 * FlightLoopCallback - Main update loop
 */
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
    (void)inElapsedTimeSinceLastFlightLoop;
    (void)inCounter;
    (void)inRefcon;
    
    if (g_state == STATE_IDLE) {
        return -1.0f; /* Continue running, but nothing to do */
    }
    
    float dt = inElapsedSinceLastCall;
    if (dt > 0.1f) dt = 0.1f; /* Clamp large time steps */
    
    /* Periodic debug logging */
    g_debugLogTimer += dt;
    if (g_debugLogTimer >= DEBUG_LOG_INTERVAL) {
        g_debugLogTimer = 0.0f;
        LogDebugStatus();
    }
    
    UpdateTrucks(dt);
    
    if (g_state == STATE_WATER_SPRAYING) {
        UpdateWaterParticles(dt);
    }
    
    return -1.0f; /* Run every frame */
}

/*
 * UpdateTrucks - Update fire truck positions and states
 */
static void UpdateTrucks(float dt) {
    auto updateTruckPosition = [dt](FireTruck& truck, bool& allPositioned) {
        if (truck.positioned) {
            return;
        }
        
        /* Calculate distance to target */
        float dx = truck.targetX - static_cast<float>(truck.x);
        float dz = truck.targetZ - static_cast<float>(truck.z);
        float distance = sqrtf(dx * dx + dz * dz);
        
        if (distance > 2.0f) {
            /* Move toward target */
            float speed = TRUCK_APPROACH_SPEED * dt;
            if (speed > distance) speed = distance;
            
            float dirX = dx / distance;
            float dirZ = dz / distance;
            
            truck.x += dirX * speed;
            truck.z += dirZ * speed;
            truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
            
            /* Update heading to face movement direction */
            truck.heading = atan2f(-dirX, -dirZ) * (180.0f / 3.14159265f);
            
            allPositioned = false;
        } else {
            /* Reached position, now turn to face target heading */
            float headingDiff = truck.targetHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            if (fabsf(headingDiff) > 1.0f) {
                float turnRate = 45.0f * dt; /* degrees per second */
                if (headingDiff > 0) {
                    truck.heading += fminf(turnRate, headingDiff);
                } else {
                    truck.heading -= fminf(turnRate, -headingDiff);
                }
                allPositioned = false;
            } else {
                truck.heading = truck.targetHeading;
                truck.positioned = true;
            }
        }
        
        /* Update instance position */
        if (truck.instance) {
            XPLMDrawInfo_t drawInfo;
            drawInfo.structSize = sizeof(XPLMDrawInfo_t);
            drawInfo.x = static_cast<float>(truck.x);
            drawInfo.y = static_cast<float>(truck.y);
            drawInfo.z = static_cast<float>(truck.z);
            drawInfo.pitch = 0.0f;
            drawInfo.heading = truck.heading;
            drawInfo.roll = 0.0f;
            XPLMInstanceSetPosition(truck.instance, &drawInfo, nullptr);
        }
    };
    
    auto updateTruckLeaving = [dt](FireTruck& truck) -> bool {
        /* Move away from current position */
        float headingRad = truck.heading * (3.14159265f / 180.0f);
        float dirX = -sinf(headingRad);
        float dirZ = -cosf(headingRad);
        
        /* First turn around */
        truck.heading += 90.0f * dt;
        
        /* Then move forward */
        float speed = TRUCK_APPROACH_SPEED * dt * 2.0f;
        truck.x += dirX * speed;
        truck.z += dirZ * speed;
        truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
        
        /* Update instance position */
        if (truck.instance) {
            XPLMDrawInfo_t drawInfo;
            drawInfo.structSize = sizeof(XPLMDrawInfo_t);
            drawInfo.x = static_cast<float>(truck.x);
            drawInfo.y = static_cast<float>(truck.y);
            drawInfo.z = static_cast<float>(truck.z);
            drawInfo.pitch = 0.0f;
            drawInfo.heading = truck.heading;
            drawInfo.roll = 0.0f;
            XPLMInstanceSetPosition(truck.instance, &drawInfo, nullptr);
        }
        
        /* Check if far enough away (300 meters from start) */
        double acX = g_drLocalX ? XPLMGetDatad(g_drLocalX) : 0.0;
        double acZ = g_drLocalZ ? XPLMGetDatad(g_drLocalZ) : 0.0;
        float dx = static_cast<float>(truck.x - acX);
        float dz = static_cast<float>(truck.z - acZ);
        float distFromAircraft = sqrtf(dx * dx + dz * dz);
        
        return distFromAircraft > 600.0f;
    };
    
    switch (g_state) {
        case STATE_TRUCKS_APPROACHING:
        case STATE_TRUCKS_POSITIONING: {
            bool allPositioned = true;
            updateTruckPosition(g_leftTruck, allPositioned);
            updateTruckPosition(g_rightTruck, allPositioned);
            
            if (g_leftTruck.positioned && g_rightTruck.positioned) {
                DebugLog("========================================");
                DebugLog("STATE TRANSITION: %s -> WATER_SPRAYING", GetStateName(g_state));
                DebugLog("Both trucks are now positioned");
                DebugLog("Left truck final pos: (%.2f, %.2f, %.2f), heading: %.1f",
                         g_leftTruck.x, g_leftTruck.y, g_leftTruck.z, g_leftTruck.heading);
                DebugLog("Right truck final pos: (%.2f, %.2f, %.2f), heading: %.1f",
                         g_rightTruck.x, g_rightTruck.y, g_rightTruck.z, g_rightTruck.heading);
                g_state = STATE_WATER_SPRAYING;
                RegisterDrawCallback();
                UpdateMenuState();
                DebugLog("Draw callback registered, water spray starting");
                DebugLog("========================================");
            } else if (g_state == STATE_TRUCKS_APPROACHING) {
                /* Check if close enough to start positioning */
                float dx = g_leftTruck.targetX - static_cast<float>(g_leftTruck.x);
                float dz = g_leftTruck.targetZ - static_cast<float>(g_leftTruck.z);
                float dist = sqrtf(dx * dx + dz * dz);
                if (dist < 50.0f) {
                    DebugLog("STATE TRANSITION: TRUCKS_APPROACHING -> TRUCKS_POSITIONING");
                    DebugLog("Trucks are within 50m of target, beginning positioning phase");
                    g_state = STATE_TRUCKS_POSITIONING;
                }
            }
            break;
        }
        
        case STATE_TRUCKS_LEAVING: {
            bool leftGone = updateTruckLeaving(g_leftTruck);
            bool rightGone = updateTruckLeaving(g_rightTruck);
            
            /* Clear particles */
            g_leftTruck.particles.clear();
            g_rightTruck.particles.clear();
            
            if (leftGone && rightGone) {
                DebugLog("STATE TRANSITION: TRUCKS_LEAVING -> IDLE");
                CleanupTruck(g_leftTruck);
                CleanupTruck(g_rightTruck);
                UnregisterDrawCallback();
                g_state = STATE_IDLE;
                UpdateMenuState();
                DebugLog("Water salute complete - all trucks departed");
            }
            break;
        }
        
        default:
            break;
    }
}

/*
 * UpdateWaterParticles - Update and emit water particles
 */
static void UpdateWaterParticles(float dt) {
    static float totalTime = 0.0f;
    totalTime += dt;
    
    auto updateParticles = [dt](FireTruck& truck, float currentTime, float /* targetX */, float /* targetZ */) {
        /* Emit new particles */
        if (currentTime - truck.lastEmitTime >= PARTICLE_EMIT_RATE) {
            truck.lastEmitTime = currentTime;
            EmitParticle(truck);
        }
        
        /* Update existing particles */
        for (auto& particle : truck.particles) {
            if (!particle.active) continue;
            
            /* Apply gravity */
            particle.vy -= 9.8f * dt;
            
            /* Update position */
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;
            particle.z += particle.vz * dt;
            
            /* Update lifetime */
            particle.lifetime -= dt;
            
            /* Check if particle should deactivate */
            if (particle.lifetime <= 0.0f || particle.y < GetTerrainHeight(particle.x, particle.z)) {
                particle.active = false;
            }
        }
        
        /* Remove inactive particles (compact the vector) */
        truck.particles.erase(
            std::remove_if(truck.particles.begin(), truck.particles.end(),
                          [](const WaterParticle& p) { return !p.active; }),
            truck.particles.end()
        );
    };
    
    /* Get center point (between trucks) for targeting */
    float centerX = (static_cast<float>(g_leftTruck.x) + static_cast<float>(g_rightTruck.x)) / 2.0f;
    float centerZ = (static_cast<float>(g_leftTruck.z) + static_cast<float>(g_rightTruck.z)) / 2.0f;
    
    updateParticles(g_leftTruck, totalTime, centerX, centerZ);
    updateParticles(g_rightTruck, totalTime, centerX, centerZ);
}

/*
 * EmitParticle - Emit a new water particle from a truck
 */
static void EmitParticle(FireTruck& truck) {
    if (truck.particles.size() >= static_cast<size_t>(NUM_PARTICLES_PER_JET)) {
        DebugLogVerbose("Particle limit reached for truck (%zu/%d)", truck.particles.size(), NUM_PARTICLES_PER_JET);
        return;
    }
    
    /* Calculate nozzle world position */
    float headingRad = truck.heading * (3.14159265f / 180.0f);
    float cosH = cosf(headingRad);
    float sinH = sinf(headingRad);
    
    float nozzleX = static_cast<float>(truck.x) + truck.nozzleOffsetX * cosH + truck.nozzleOffsetZ * (-sinH);
    float nozzleY = static_cast<float>(truck.y) + truck.nozzleOffsetY;
    float nozzleZ = static_cast<float>(truck.z) + truck.nozzleOffsetX * sinH + truck.nozzleOffsetZ * (-cosH);
    
    /* Calculate target point (center between trucks, at height) */
    float centerX = (static_cast<float>(g_leftTruck.x) + static_cast<float>(g_rightTruck.x)) / 2.0f;
    float centerZ = (static_cast<float>(g_leftTruck.z) + static_cast<float>(g_rightTruck.z)) / 2.0f;
    float centerY = static_cast<float>(truck.y) + WATER_JET_HEIGHT;
    
    /* Calculate direction to center/top of arc */
    float dx = centerX - nozzleX;
    float dz = centerZ - nozzleZ;
    float dist = sqrtf(dx * dx + dz * dz);
    
    /* Calculate initial velocity for parabolic arc */
    float t = WATER_JET_DURATION;
    float vx = dx / (2.0f * t);
    float vz = dz / (2.0f * t);
    float vy = (centerY - nozzleY + 0.5f * 9.8f * t * t) / t;
    
    /* Add some randomness */
    float randScale = 0.1f;
    vx += (static_cast<float>(rand()) / RAND_MAX - 0.5f) * randScale * dist;
    vz += (static_cast<float>(rand()) / RAND_MAX - 0.5f) * randScale * dist;
    vy += (static_cast<float>(rand()) / RAND_MAX - 0.5f) * randScale * 5.0f;
    
    /* Create particle */
    WaterParticle particle;
    particle.x = nozzleX;
    particle.y = nozzleY;
    particle.z = nozzleZ;
    particle.vx = vx;
    particle.vy = vy;
    particle.vz = vz;
    particle.lifetime = PARTICLE_LIFETIME;
    particle.maxLifetime = PARTICLE_LIFETIME;
    particle.active = true;
    
    truck.particles.push_back(particle);
    g_particlesEmittedTotal++;
    
    /* Log first particle emission for debugging */
    if (g_particlesEmittedTotal == 1) {
        DebugLog("First particle emitted:");
        DebugLog("  Nozzle pos: (%.2f, %.2f, %.2f)", nozzleX, nozzleY, nozzleZ);
        DebugLog("  Target center: (%.2f, %.2f, %.2f)", centerX, centerY, centerZ);
        DebugLog("  Velocity: (%.2f, %.2f, %.2f)", vx, vy, vz);
        DebugLog("  Distance to center: %.2f", dist);
    }
}

/*
 * RegisterDrawCallback - Register the draw callback for water particles
 */
static void RegisterDrawCallback() {
    if (!g_drawCallbackRegistered) {
        int result = XPLMRegisterDrawCallback(DrawWaterParticles, WATER_DRAWING_PHASE, 0, nullptr);
        g_drawCallbackRegistered = true;
        g_drawCallbackCallCount = 0; /* Reset counter */
        DebugLog("Draw callback registered (result=%d)", result);
        DebugLog("  Drawing phase: xplm_Phase_Modern3D (%d)", WATER_DRAWING_PHASE);
        DebugLog("  NOTE: Callback will be invoked but water particles won't be visible");
        DebugLog("  because the DrawWaterParticles function lacks OpenGL rendering code.");
    }
}

/*
 * UnregisterDrawCallback - Unregister the draw callback for water particles
 */
static void UnregisterDrawCallback() {
    if (g_drawCallbackRegistered) {
        XPLMUnregisterDrawCallback(DrawWaterParticles, WATER_DRAWING_PHASE, 0, nullptr);
        g_drawCallbackRegistered = false;
        DebugLog("Draw callback unregistered");
        DebugLog("Final draw callback call count: %d", g_drawCallbackCallCount);
    }
}

/* 
 * Draw callback for rendering water particles
 * 
 * IMPORTANT DEBUG NOTE: This function is called but does NOT actually render 
 * any visible particles. The particle data is computed but no OpenGL drawing
 * commands are executed. This is a known limitation.
 * 
 * To implement visible water effects, one of the following approaches is needed:
 * 1. Use OpenGL immediate mode or VBOs to draw billboarded quads
 * 2. Use X-Plane's particle system (if available in SDK version)
 * 3. Use instanced OBJ rendering for water droplet models
 */
static int DrawWaterParticles(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    (void)inPhase;
    (void)inIsBefore;
    (void)inRefcon;
    
    g_drawCallbackCallCount++;
    
    /* Log first callback and then periodically */
    if (g_drawCallbackCallCount == 1) {
        DebugLog("DrawWaterParticles callback first invocation");
        DebugLog("  Phase: %d, IsBefore: %d", inPhase, inIsBefore);
    }
    
    if (g_state != STATE_WATER_SPRAYING) {
        if (g_drawCallbackCallCount <= 5) {
            DebugLog("DrawWaterParticles: State is not WATER_SPRAYING (state=%s)", GetStateName(g_state));
        }
        return 1;
    }
    
    /* Set up graphics state for particle rendering */
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);
    
    /* Count active particles for debugging */
    int leftActiveCount = 0;
    int rightActiveCount = 0;
    
    /* Draw particles as points/billboards */
    /* 
     * =====================================================================
     * WARNING: NO ACTUAL RENDERING IS HAPPENING HERE!
     * =====================================================================
     * The code below iterates through particles but does NOT draw anything.
     * This is why no water effect is visible.
     * 
     * To fix this, OpenGL drawing commands need to be added, such as:
     * - glBegin(GL_POINTS) / glVertex3f() / glEnd() for point particles
     * - Or textured quads for billboarded water droplets
     * - Or use XPLMDrawString for debug visualization
     * =====================================================================
     */
    
    auto drawTruckParticles = [](const FireTruck& truck, int& activeCount) {
        for (const auto& particle : truck.particles) {
            if (!particle.active) continue;
            activeCount++;
            
            /* Calculate alpha based on lifetime */
            float alpha = particle.lifetime / particle.maxLifetime;
            
            /* 
             * PLACEHOLDER: No actual drawing is performed here!
             * This is where OpenGL calls should go to render the particles.
             * Currently the particles exist in memory but are not visualized.
             */
            (void)alpha;
            (void)particle; /* Silence unused variable warning */
        }
    };
    
    drawTruckParticles(g_leftTruck, leftActiveCount);
    drawTruckParticles(g_rightTruck, rightActiveCount);
    
    /* Periodic logging of particle counts during drawing */
    if (g_drawCallbackCallCount % 100 == 0) {
        DebugLog("DrawWaterParticles stats (call #%d):", g_drawCallbackCallCount);
        DebugLog("  Left truck active particles: %d", leftActiveCount);
        DebugLog("  Right truck active particles: %d", rightActiveCount);
        DebugLog("  WARNING: Particles are being tracked but NOT rendered!");
        DebugLog("  The DrawWaterParticles function needs OpenGL drawing code.");
    }
    
    return 1;
}
