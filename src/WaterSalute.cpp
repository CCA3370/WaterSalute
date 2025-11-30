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

/*
 * XPluginStart - Called when the plugin is loaded
 */
XPLM_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "Water Salute");
    strcpy(outSig, "com.xplane.watersalute");
    strcpy(outDesc, "Water salute ceremony with fire trucks");
    
    /* Get plugin path */
    XPLMGetPluginDirectoryPath(g_pluginPath);
    
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
XPLM_API void XPluginStop(void) {
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
XPLM_API int XPluginEnable(void) {
    DebugLog("Plugin enabled");
    return 1;
}

/*
 * XPluginDisable - Called when the plugin is disabled
 */
XPLM_API void XPluginDisable(void) {
    DebugLog("Plugin disabled");
    StopWaterSalute();
}

/*
 * XPluginReceiveMessage - Handle messages from X-Plane
 */
XPLM_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) {
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
    
    g_truckObject = XPLMLoadObject(modelPath);
    
    if (!g_truckObject) {
        DebugLog("Failed to load fire truck model, will use default rendering");
        return false;
    }
    
    DebugLog("Fire truck model loaded successfully");
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
    DebugLog("StartWaterSalute called");
    
    if (g_state != STATE_IDLE) {
        DebugLog("Cannot start - not in idle state");
        return;
    }
    
    /* Check if aircraft is on ground */
    int onGround = 0;
    if (g_drOnGround) {
        onGround = XPLMGetDatai(g_drOnGround);
    }
    
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
        if (wingspan <= 0.0f || wingspan > 100.0f) {
            wingspan = 30.0f; /* Use default for invalid values */
        }
    }
    
    DebugLog("Aircraft position: (%.2f, %.2f, %.2f), heading: %.1f, wingspan: %.1f",
             acX, acY, acZ, acHeading, wingspan);
    
    /* Calculate truck spacing (wingspan + 40 meters) */
    float truckSpacing = (wingspan / 2.0f) + (TRUCK_EXTRA_SPACING / 2.0f);
    
    /* Convert heading to radians */
    float headingRad = acHeading * (3.14159265f / 180.0f);
    
    /* Calculate forward vector (X-Plane uses -Z as forward) */
    float forwardX = -sinf(headingRad);
    float forwardZ = -cosf(headingRad);
    
    /* Calculate right vector */
    float rightX = cosf(headingRad);
    float rightZ = -sinf(headingRad);
    
    /* Calculate truck start positions (500 meters ahead) */
    float startDistance = 500.0f;
    float startX = static_cast<float>(acX) + forwardX * startDistance;
    float startZ = static_cast<float>(acZ) + forwardZ * startDistance;
    
    /* Calculate truck target positions (200 meters ahead) */
    float targetX = static_cast<float>(acX) + forwardX * TRUCK_STOP_DISTANCE;
    float targetZ = static_cast<float>(acZ) + forwardZ * TRUCK_STOP_DISTANCE;
    
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
    
    /* Create truck instances if model loaded */
    if (g_truckObject) {
        const char* noDataRefs[] = { nullptr };
        g_leftTruck.instance = XPLMCreateInstance(g_truckObject, noDataRefs);
        g_rightTruck.instance = XPLMCreateInstance(g_truckObject, noDataRefs);
    }
    
    g_state = STATE_TRUCKS_APPROACHING;
    UpdateMenuState();
    
    DebugLog("Water salute started - trucks approaching");
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
                g_state = STATE_WATER_SPRAYING;
                RegisterDrawCallback();
                UpdateMenuState();
                DebugLog("Trucks positioned, starting water spray");
            } else if (g_state == STATE_TRUCKS_APPROACHING) {
                /* Check if close enough to start positioning */
                float dx = g_leftTruck.targetX - static_cast<float>(g_leftTruck.x);
                float dz = g_leftTruck.targetZ - static_cast<float>(g_leftTruck.z);
                float dist = sqrtf(dx * dx + dz * dz);
                if (dist < 50.0f) {
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
                CleanupTruck(g_leftTruck);
                CleanupTruck(g_rightTruck);
                UnregisterDrawCallback();
                g_state = STATE_IDLE;
                UpdateMenuState();
                DebugLog("Water salute complete");
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
}

/*
 * RegisterDrawCallback - Register the draw callback for water particles
 */
static void RegisterDrawCallback() {
    if (!g_drawCallbackRegistered) {
        XPLMRegisterDrawCallback(DrawWaterParticles, WATER_DRAWING_PHASE, 0, nullptr);
        g_drawCallbackRegistered = true;
        DebugLog("Draw callback registered");
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
    }
}

/* 
 * Draw callback for rendering water particles
 */
static int DrawWaterParticles(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    (void)inPhase;
    (void)inIsBefore;
    (void)inRefcon;
    
    if (g_state != STATE_WATER_SPRAYING) {
        return 1;
    }
    
    /* Set up graphics state for particle rendering */
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);
    
    /* Draw particles as points/billboards */
    /* Note: In a real implementation, this would use OpenGL to render particles
     * as textured billboards with proper alpha blending */
    
    auto drawTruckParticles = [](const FireTruck& truck) {
        for (const auto& particle : truck.particles) {
            if (!particle.active) continue;
            
            /* Calculate alpha based on lifetime */
            float alpha = particle.lifetime / particle.maxLifetime;
            
            /* In real implementation:
             * - Bind water droplet texture
             * - Draw billboard quad at particle.x, particle.y, particle.z
             * - Apply alpha fading
             * - Use blue/cyan color
             */
            (void)alpha;
        }
    };
    
    drawTruckParticles(g_leftTruck);
    drawTruckParticles(g_rightTruck);
    
    return 1;
}
