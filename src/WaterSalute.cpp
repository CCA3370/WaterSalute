/*
 * WaterSalute.cpp - Main plugin file for Water Salute X-Plane 12 Plugin
 * 
 * This plugin simulates a water salute ceremony where two fire trucks
 * approach the aircraft and spray water arches over it.
 * 
 * Features:
 * - Menu system with Start/Stop controls
 * - Aircraft ground and speed validation
 * - Fire truck positioning based on aircraft wingspan
 * - Particle-based water effects
 * - Road network path planning from apt.dat
 * - External OBJ model loading for fire trucks
 * 
 * Copyright (c) 2024
 */

#include "Common.h"
#include "RoadNetwork.h"
#include "PathPlanning.h"
#include "FireTruck.h"

/* Global state */
PluginState g_state = STATE_IDLE;
static XPLMMenuID g_menuId = nullptr;
static int g_menuStartItem = -1;
static int g_menuStopItem = -1;

XPLMObjectRef g_truckObject = nullptr;
XPLMObjectRef g_waterDropObject = nullptr;
XPLMProbeRef g_terrainProbe = nullptr;

static XPLMFlightLoopID g_flightLoopId = nullptr;
static bool g_drawCallbackRegistered = false;

/* Fast random number generator for particle effects */
std::mt19937 g_rng;
std::uniform_real_distribution<float> g_randomDist(-0.5f, 0.5f);

/* Datarefs from X-Plane */
static XPLMDataRef g_drOnGround = nullptr;
static XPLMDataRef g_drGroundSpeed = nullptr;
static XPLMDataRef g_drLocalX = nullptr;
static XPLMDataRef g_drLocalY = nullptr;
static XPLMDataRef g_drLocalZ = nullptr;
static XPLMDataRef g_drHeading = nullptr;
static XPLMDataRef g_drWingspan = nullptr;
static XPLMDataRef g_drLatitude = nullptr;
static XPLMDataRef g_drLongitude = nullptr;

/* Custom datarefs published by this plugin */
static XPLMDataRef g_drTruckFrontSteeringAngle = nullptr;
static XPLMDataRef g_drTruckRearSteeringAngle = nullptr;
static XPLMDataRef g_drTruckWheelAngle = nullptr;
static XPLMDataRef g_drTruckCannonPitch = nullptr;
static XPLMDataRef g_drTruckCannonYaw = nullptr;
static XPLMDataRef g_drTruckSpeed = nullptr;

static char g_pluginPath[512];
static char g_resourcePath[512];
static float g_debugLogTimer = 0.0f;
static int g_drawCallbackCallCount = 0;
int g_particlesEmittedTotal = 0;

static const char* g_noDataRefs[] = { nullptr };

/* Forward declarations */
static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);
static void MenuHandler(void* inMenuRef, void* inItemRef);
static void StartWaterSalute();
static void StopWaterSalute();
static void UpdateTrucks(float dt);
static void UpdateMenuState();
float GetTerrainHeight(float x, float z);
static bool FindResourcePath();
static bool LoadFireTruckModel();
static bool LoadWaterDropModel();
static int DrawWaterParticles(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);
static void RegisterDrawCallback();
static void UnregisterDrawCallback();
static void RegisterCustomDataRefs();
static void UnregisterCustomDataRefs();

/* Custom dataref accessors */
static int GetFrontSteeringAngle(void* inRefcon, float* outValues, int inOffset, int inMax) {
    if (outValues == nullptr) return 2;
    
    int count = 0;
    for (int i = inOffset; i < inOffset + inMax && i < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(i);
        if (truck) {
            outValues[count++] = truck->frontSteeringAngle;
        }
    }
    return count;
}

static void SetFrontSteeringAngle(void* inRefcon, float* inValues, int inOffset, int inCount) {
    if (inValues == nullptr) return;
    
    for (int i = 0; i < inCount && (inOffset + i) < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            truck->frontSteeringAngle = ClampSteeringAngle(inValues[i]);
            truck->rearSteeringAngle = CalculateRearSteeringAngle(truck->frontSteeringAngle);
        }
    }
}

static int GetRearSteeringAngle(void* inRefcon, float* outValues, int inOffset, int inMax) {
    if (outValues == nullptr) return 2;
    
    int count = 0;
    for (int i = inOffset; i < inOffset + inMax && i < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(i);
        if (truck) {
            outValues[count++] = truck->rearSteeringAngle;
        }
    }
    return count;
}

static void SetRearSteeringAngle(void* inRefcon, float* inValues, int inOffset, int inCount) {
    if (inValues == nullptr) return;
    
    for (int i = 0; i < inCount && (inOffset + i) < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            truck->rearSteeringAngle = ClampSteeringAngle(inValues[i]);
        }
    }
}

static int GetWheelRotationAngle(void* inRefcon, float* outValues, int inOffset, int inMax) {
    if (outValues == nullptr) return 2;
    
    int count = 0;
    for (int i = inOffset; i < inOffset + inMax && i < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(i);
        if (truck) {
            outValues[count++] = truck->wheelRotationAngle;
        }
    }
    return count;
}

static int GetCannonPitch(void* inRefcon, float* outValues, int inOffset, int inMax) {
    if (outValues == nullptr) return 2;
    
    int count = 0;
    for (int i = inOffset; i < inOffset + inMax && i < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(i);
        if (truck) {
            outValues[count++] = truck->cannonPitch;
        }
    }
    return count;
}

static void SetCannonPitch(void* inRefcon, float* inValues, int inOffset, int inCount) {
    if (inValues == nullptr) return;
    
    for (int i = 0; i < inCount && (inOffset + i) < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            float value = inValues[i];
            if (value < MIN_CANNON_PITCH) value = MIN_CANNON_PITCH;
            if (value > MAX_CANNON_PITCH) value = MAX_CANNON_PITCH;
            truck->cannonPitch = value;
        }
    }
}

static int GetCannonYaw(void* inRefcon, float* outValues, int inOffset, int inMax) {
    if (outValues == nullptr) return 2;
    
    int count = 0;
    for (int i = inOffset; i < inOffset + inMax && i < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(i);
        if (truck) {
            outValues[count++] = truck->cannonYaw;
        }
    }
    return count;
}

static void SetCannonYaw(void* inRefcon, float* inValues, int inOffset, int inCount) {
    if (inValues == nullptr) return;
    
    for (int i = 0; i < inCount && (inOffset + i) < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            float value = inValues[i];
            while (value > 180.0f) value -= 360.0f;
            while (value < -180.0f) value += 360.0f;
            truck->cannonYaw = value;
        }
    }
}

static int GetTruckSpeed(void* inRefcon, float* outValues, int inOffset, int inMax) {
    if (outValues == nullptr) return 2;
    
    int count = 0;
    for (int i = inOffset; i < inOffset + inMax && i < 2; ++i) {
        FireTruck* truck = GetTruckByIndex(i);
        if (truck) {
            outValues[count++] = truck->speed;
        }
    }
    return count;
}

static void RegisterCustomDataRefs() {
    DebugLog("Registering custom datarefs...");
    
    g_drTruckFrontSteeringAngle = XPLMRegisterDataAccessor(
        "watersalute/truck/front_steering_angle",
        xplmType_FloatArray,
        1,  /* Writable */
        nullptr, nullptr,  /* Int accessors */
        nullptr, nullptr,  /* Float accessors */
        nullptr, nullptr,  /* Double accessors */
        nullptr, nullptr,  /* Int array accessors */
        GetFrontSteeringAngle, SetFrontSteeringAngle,  /* Float array accessors */
        nullptr, nullptr,  /* Data accessors */
        nullptr, nullptr   /* Refcon */
    );
    
    g_drTruckRearSteeringAngle = XPLMRegisterDataAccessor(
        "watersalute/truck/rear_steering_angle",
        xplmType_FloatArray,
        1,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        GetRearSteeringAngle, SetRearSteeringAngle,
        nullptr, nullptr,
        nullptr, nullptr
    );
    
    g_drTruckWheelAngle = XPLMRegisterDataAccessor(
        "watersalute/truck/wheel_rotation_angle",
        xplmType_FloatArray,
        0,  /* Read-only */
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        GetWheelRotationAngle, nullptr,
        nullptr, nullptr,
        nullptr, nullptr
    );
    
    g_drTruckCannonPitch = XPLMRegisterDataAccessor(
        "watersalute/truck/cannon_pitch",
        xplmType_FloatArray,
        1,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        GetCannonPitch, SetCannonPitch,
        nullptr, nullptr,
        nullptr, nullptr
    );
    
    g_drTruckCannonYaw = XPLMRegisterDataAccessor(
        "watersalute/truck/cannon_yaw",
        xplmType_FloatArray,
        1,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        GetCannonYaw, SetCannonYaw,
        nullptr, nullptr,
        nullptr, nullptr
    );
    
    g_drTruckSpeed = XPLMRegisterDataAccessor(
        "watersalute/truck/speed",
        xplmType_FloatArray,
        0,  /* Read-only */
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        nullptr, nullptr,
        GetTruckSpeed, nullptr,
        nullptr, nullptr,
        nullptr, nullptr
    );
    
    DebugLog("Custom datarefs registered");
}

static void UnregisterCustomDataRefs() {
    DebugLog("Unregistering custom datarefs...");
    
    if (g_drTruckFrontSteeringAngle) {
        XPLMUnregisterDataAccessor(g_drTruckFrontSteeringAngle);
        g_drTruckFrontSteeringAngle = nullptr;
    }
    if (g_drTruckRearSteeringAngle) {
        XPLMUnregisterDataAccessor(g_drTruckRearSteeringAngle);
        g_drTruckRearSteeringAngle = nullptr;
    }
    if (g_drTruckWheelAngle) {
        XPLMUnregisterDataAccessor(g_drTruckWheelAngle);
        g_drTruckWheelAngle = nullptr;
    }
    if (g_drTruckCannonPitch) {
        XPLMUnregisterDataAccessor(g_drTruckCannonPitch);
        g_drTruckCannonPitch = nullptr;
    }
    if (g_drTruckCannonYaw) {
        XPLMUnregisterDataAccessor(g_drTruckCannonYaw);
        g_drTruckCannonYaw = nullptr;
    }
    if (g_drTruckSpeed) {
        XPLMUnregisterDataAccessor(g_drTruckSpeed);
        g_drTruckSpeed = nullptr;
    }
    
    DebugLog("Custom datarefs unregistered");
}

static bool FindResourcePath() {
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, g_pluginPath, nullptr, nullptr);
    DebugLog("Plugin path: %s", g_pluginPath);
    
    /* Try standard plugin folder structure: WaterSalute/64/WaterSalute.xpl */
    char* lastSlash = strrchr(g_pluginPath, '/');
    if (!lastSlash) lastSlash = strrchr(g_pluginPath, '\\');
    
    if (lastSlash) {
        *lastSlash = '\0';  /* Remove executable name */
        
        /* Check if we're in a "64" or "mac_x64" subdirectory */
        char* parentSlash = strrchr(g_pluginPath, '/');
        if (!parentSlash) parentSlash = strrchr(g_pluginPath, '\\');
        
        if (parentSlash) {
            char parentPath[512];
            strncpy(parentPath, g_pluginPath, sizeof(parentPath) - 1);
            parentPath[sizeof(parentPath) - 1] = '\0';
            
            char* parentDirName = parentSlash + 1;
            
            /* If we're in "64", "mac_x64", "win_x64", or "lin_x64", go up one more level */
            if (strcmp(parentDirName, "64") == 0 || 
                strcmp(parentDirName, "mac_x64") == 0 ||
                strcmp(parentDirName, "win_x64") == 0 ||
                strcmp(parentDirName, "lin_x64") == 0) {
                *parentSlash = '\0';  /* Go up to parent directory */
                snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", g_pluginPath);
            } else {
                snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", g_pluginPath);
            }
            
            /* Check if resources directory exists */
            char testPath[512];
            snprintf(testPath, sizeof(testPath), "%s/firetruck.obj", g_resourcePath);
            FILE* testFile = fopen(testPath, "r");
            if (testFile) {
                fclose(testFile);
                DebugLog("Resource path found: %s", g_resourcePath);
                return true;
            }
            
            /* Try parent's resources folder */
            *parentSlash = '\0';
            snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", parentPath);
            snprintf(testPath, sizeof(testPath), "%s/firetruck.obj", g_resourcePath);
            testFile = fopen(testPath, "r");
            if (testFile) {
                fclose(testFile);
                DebugLog("Resource path found (parent): %s", g_resourcePath);
                return true;
            }
        }
    }
    
    /* Fallback: use plugin path with /resources */
    snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", g_pluginPath);
    DebugLog("Using fallback resource path: %s", g_resourcePath);
    return false;
}

static bool LoadFireTruckModel() {
    char objPath[512];
    snprintf(objPath, sizeof(objPath), "%s/firetruck.obj", g_resourcePath);
    
    DebugLog("Loading fire truck model from: %s", objPath);
    
    /* Convert to relative path from X-Plane root */
    char xplanePath[512];
    XPLMGetSystemPath(xplanePath);
    
    /* Try loading from absolute path first */
    g_truckObject = XPLMLoadObject(objPath);
    
    if (!g_truckObject) {
        DebugLog("Failed to load fire truck model from absolute path, trying relative...");
        
        /* Try relative path from X-Plane folder */
        if (strstr(objPath, xplanePath) == objPath) {
            const char* relativePath = objPath + strlen(xplanePath);
            g_truckObject = XPLMLoadObject(relativePath);
        }
    }
    
    if (g_truckObject) {
        DebugLog("Fire truck model loaded successfully");
        return true;
    } else {
        DebugLog("ERROR: Failed to load fire truck model");
        return false;
    }
}

static bool LoadWaterDropModel() {
    char objPath[512];
    snprintf(objPath, sizeof(objPath), "%s/waterdrop.obj", g_resourcePath);
    
    DebugLog("Loading water drop model from: %s", objPath);
    
    char xplanePath[512];
    XPLMGetSystemPath(xplanePath);
    
    g_waterDropObject = XPLMLoadObject(objPath);
    
    if (!g_waterDropObject) {
        if (strstr(objPath, xplanePath) == objPath) {
            const char* relativePath = objPath + strlen(xplanePath);
            g_waterDropObject = XPLMLoadObject(relativePath);
        }
    }
    
    if (g_waterDropObject) {
        DebugLog("Water drop model loaded successfully");
        return true;
    } else {
        DebugLog("WARNING: Failed to load water drop model - particles will not be visible");
        return false;
    }
}

static void MenuHandler(void* inMenuRef, void* inItemRef) {
    int itemId = (int)(intptr_t)inItemRef;
    
    if (itemId == 1) {
        StartWaterSalute();
    } else if (itemId == 2) {
        StopWaterSalute();
    }
}

static void UpdateMenuState() {
    if (g_menuId) {
        XPLMEnableMenuItem(g_menuId, g_menuStartItem, g_state == STATE_IDLE ? 1 : 0);
        XPLMEnableMenuItem(g_menuId, g_menuStopItem, g_state != STATE_IDLE ? 1 : 0);
    }
}

static void StartWaterSalute() {
    DebugLog("========================================");
    DebugLog("StartWaterSalute called");
    DebugLog("========================================");
    
    g_drawCallbackCallCount = 0;
    g_particlesEmittedTotal = 0;
    g_debugLogTimer = 0.0f;
    
    if (g_state != STATE_IDLE) {
        DebugLog("Cannot start - not in idle state (current: %s)", GetStateName(g_state));
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
    
    /* Get wingspan */
    float wingspan = DEFAULT_WINGSPAN_METERS;
    if (g_drWingspan) {
        float rawValue = XPLMGetDataf(g_drWingspan);
        if (rawValue >= MIN_SEMISPAN_METERS && rawValue <= MAX_SEMISPAN_METERS) {
            wingspan = rawValue * 2.0f;
        } else {
            float wingspanMeters = rawValue * FEET_TO_METERS;
            if (wingspanMeters >= MIN_WINGSPAN_METERS && wingspanMeters <= MAX_WINGSPAN_METERS) {
                wingspan = wingspanMeters;
            } else if (rawValue >= MIN_WINGSPAN_METERS && rawValue <= MAX_WINGSPAN_METERS) {
                wingspan = rawValue;
            }
        }
    }
    
    DebugLog("Aircraft position: (%.2f, %.2f, %.2f)", acX, acY, acZ);
    DebugLog("Aircraft heading: %.1f degrees, wingspan: %.1f meters", acHeading, wingspan);
    
    /* Try to load road network */
    double acLat = 0.0, acLon = 0.0;
    if (g_drLatitude && g_drLongitude) {
        acLat = XPLMGetDatad(g_drLatitude);
        acLon = XPLMGetDatad(g_drLongitude);
        DebugLog("Aircraft lat/lon: (%.6f, %.6f)", acLat, acLon);
    }
    
    bool roadNetworkLoaded = LoadAptDat(acLat, acLon);
    if (roadNetworkLoaded) {
        DebugLog("Road network loaded successfully for airport %s", g_roadNetwork.airportId.c_str());
    } else {
        DebugLog("Road network not available, using direct approach");
    }
    
    /* Calculate truck spacing */
    float truckSpacing = (wingspan / 2.0f) + (TRUCK_EXTRA_SPACING / 2.0f);
    
    float headingRad = acHeading * DEG_TO_RAD;
    float forwardX = sinf(headingRad);
    float forwardZ = -cosf(headingRad);
    float rightX = cosf(headingRad);
    float rightZ = sinf(headingRad);
    
    /* Calculate positions */
    float startDistance = 500.0f;
    float startX = static_cast<float>(acX) + forwardX * startDistance;
    float startZ = static_cast<float>(acZ) + forwardZ * startDistance;
    
    float targetX = static_cast<float>(acX) + forwardX * TRUCK_STOP_DISTANCE;
    float targetZ = static_cast<float>(acZ) + forwardZ * TRUCK_STOP_DISTANCE;
    
    /* Initialize left truck */
    InitializeTruck(g_leftTruck);
    g_leftTruck.x = startX - rightX * truckSpacing;
    g_leftTruck.z = startZ - rightZ * truckSpacing;
    g_leftTruck.y = GetTerrainHeight(static_cast<float>(g_leftTruck.x), static_cast<float>(g_leftTruck.z));
    g_leftTruck.heading = acHeading + 180.0f;
    g_leftTruck.targetX = targetX - rightX * truckSpacing;
    g_leftTruck.targetZ = targetZ - rightZ * truckSpacing;
    g_leftTruck.targetHeading = acHeading + 90.0f;
    g_leftTruck.nozzleOffsetY = 3.5f;
    g_leftTruck.nozzleOffsetZ = 2.0f;
    
    /* Initialize right truck */
    InitializeTruck(g_rightTruck);
    g_rightTruck.x = startX + rightX * truckSpacing;
    g_rightTruck.z = startZ + rightZ * truckSpacing;
    g_rightTruck.y = GetTerrainHeight(static_cast<float>(g_rightTruck.x), static_cast<float>(g_rightTruck.z));
    g_rightTruck.heading = acHeading + 180.0f;
    g_rightTruck.targetX = targetX + rightX * truckSpacing;
    g_rightTruck.targetZ = targetZ + rightZ * truckSpacing;
    g_rightTruck.targetHeading = acHeading - 90.0f;
    g_rightTruck.nozzleOffsetY = 3.5f;
    g_rightTruck.nozzleOffsetZ = 2.0f;
    
    /* Plan routes if road network available */
    if (roadNetworkLoaded) {
        size_t leftSpawnNode = FindNearestNode(g_leftTruck.x, g_leftTruck.z, true);
        size_t rightSpawnNode = FindNearestNode(g_rightTruck.x, g_rightTruck.z, true);
        
        if (leftSpawnNode != SIZE_MAX) {
            const RoadNode& node = g_roadNetwork.nodes[leftSpawnNode];
            g_leftTruck.x = node.x;
            g_leftTruck.z = node.z;
            g_leftTruck.y = GetTerrainHeight(static_cast<float>(node.x), static_cast<float>(node.z));
        }
        
        if (rightSpawnNode != SIZE_MAX) {
            const RoadNode& node = g_roadNetwork.nodes[rightSpawnNode];
            g_rightTruck.x = node.x;
            g_rightTruck.z = node.z;
            g_rightTruck.y = GetTerrainHeight(static_cast<float>(node.x), static_cast<float>(node.z));
        }
        
        g_leftTruck.route = PlanRouteToTarget(g_leftTruck.x, g_leftTruck.z, 
                                               g_leftTruck.targetX, g_leftTruck.targetZ,
                                               g_leftTruck.targetHeading);
        g_leftTruck.useRoadNetwork = g_leftTruck.route.isValid;
        
        g_rightTruck.route = PlanRouteToTarget(g_rightTruck.x, g_rightTruck.z,
                                                g_rightTruck.targetX, g_rightTruck.targetZ,
                                                g_rightTruck.targetHeading);
        g_rightTruck.useRoadNetwork = g_rightTruck.route.isValid;
        
        if (g_leftTruck.route.isValid && g_leftTruck.route.waypoints.size() > 1) {
            g_leftTruck.heading = g_leftTruck.route.waypoints[0].targetHeading;
        }
        if (g_rightTruck.route.isValid && g_rightTruck.route.waypoints.size() > 1) {
            g_rightTruck.heading = g_rightTruck.route.waypoints[0].targetHeading;
        }
    }
    
    /* Create truck instances */
    if (g_truckObject) {
        g_leftTruck.instance = XPLMCreateInstance(g_truckObject, g_noDataRefs);
        g_rightTruck.instance = XPLMCreateInstance(g_truckObject, g_noDataRefs);
        
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
        }
    }
    
    g_state = STATE_TRUCKS_APPROACHING;
    UpdateMenuState();
    
    DebugLog("State changed to: %s", GetStateName(g_state));
    DebugLog("Water salute started - trucks approaching");
    XPLMSpeakString("Water salute started");
}

static void StopWaterSalute() {
    DebugLog("StopWaterSalute called");
    
    if (g_state == STATE_IDLE) {
        DebugLog("Already stopped");
        return;
    }
    
    if (g_state == STATE_TRUCKS_LEAVING) {
        DebugLog("Trucks already leaving");
        return;
    }
    
    /* Set trucks to leave */
    g_leftTruck.isTurningBeforeLeave = true;
    g_leftTruck.leaveHeading = NormalizeAngle360(g_leftTruck.heading - 45.0f);
    
    g_rightTruck.isTurningBeforeLeave = true;
    g_rightTruck.leaveHeading = NormalizeAngle360(g_rightTruck.heading + 45.0f);
    
    g_state = STATE_TRUCKS_LEAVING;
    UpdateMenuState();
    
    DebugLog("State changed to: %s", GetStateName(g_state));
    XPLMSpeakString("Water salute ending");
}

float GetTerrainHeight(float x, float z) {
    if (!g_terrainProbe) return 0.0f;
    
    XPLMProbeInfo_t probeInfo;
    probeInfo.structSize = sizeof(XPLMProbeInfo_t);
    
    XPLMProbeResult result = XPLMProbeTerrainXYZ(g_terrainProbe, x, 10000.0f, z, &probeInfo);
    
    if (result == xplm_ProbeHitTerrain) {
        return probeInfo.locationY;
    }
    
    return 0.0f;
}

static float FlightLoopCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {
    if (g_state == STATE_IDLE) {
        return -1.0f;  /* Check less frequently when idle */
    }
    
    float dt = inElapsedSinceLastCall;
    if (dt > 0.1f) dt = 0.1f;  /* Cap delta time */
    
    g_debugLogTimer += dt;
    
    if (g_state == STATE_TRUCKS_APPROACHING || g_state == STATE_TRUCKS_POSITIONING) {
        UpdateTrucks(dt);
    } else if (g_state == STATE_WATER_SPRAYING) {
        UpdateWaterParticles(dt);
    } else if (g_state == STATE_TRUCKS_LEAVING) {
        UpdateTrucks(dt);
    }
    
    return -1.0f;  /* Call every frame */
}

static void UpdateTrucks(float dt) {
    auto updateTruckPosition = [dt](FireTruck& truck, bool& allPositioned) {
        if (truck.positioned) {
            truck.targetSpeed = 0.0f;
            truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
            truck.frontSteeringAngle = 0.0f;
            truck.rearSteeringAngle = 0.0f;
            return;
        }
        
        /* Use road network path following if available */
        if (truck.useRoadNetwork && truck.route.isValid && !truck.route.isCompleted) {
            UpdateTruckFollowingPath(truck, dt);
            if (truck.route.isCompleted) {
                truck.positioned = true;
            } else {
                allPositioned = false;
            }
            return;
        }
        
        /* Fallback: Direct approach */
        float dx = truck.targetX - static_cast<float>(truck.x);
        float dz = truck.targetZ - static_cast<float>(truck.z);
        float distance = sqrtf(dx * dx + dz * dz);
        
        if (distance > 2.0f) {
            float dirX = dx / distance;
            float dirZ = dz / distance;
            float desiredHeading = atan2f(dirX, -dirZ) * RAD_TO_DEG;
            
            float headingDiff = desiredHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
            truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
            
            if (distance < TRUCK_SLOWDOWN_DISTANCE) {
                truck.targetSpeed = TRUCK_APPROACH_SPEED * (distance / TRUCK_SLOWDOWN_DISTANCE);
                if (truck.targetSpeed < TRUCK_TURN_IN_PLACE_SPEED) {
                    truck.targetSpeed = TRUCK_TURN_IN_PLACE_SPEED;
                }
            } else {
                truck.targetSpeed = TRUCK_APPROACH_SPEED;
            }
            
            truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
            
            float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
            truck.heading += turningRate * dt;
            truck.heading = NormalizeAngle360(truck.heading);
            
            float headingRad = truck.heading * DEG_TO_RAD;
            float moveDistance = truck.speed * dt;
            truck.x += sinf(headingRad) * moveDistance;
            truck.z += -cosf(headingRad) * moveDistance;
            truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
            
            UpdateWheelRotationAngle(truck, moveDistance);
            
            allPositioned = false;
        } else {
            /* Start positioning phase */
            float headingDiff = truck.targetHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            if (fabsf(headingDiff) > HEADING_TOLERANCE_DEG) {
                truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
                truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
                truck.targetSpeed = TRUCK_TURN_IN_PLACE_SPEED;
                truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
                
                float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
                truck.heading += turningRate * dt;
                truck.heading = NormalizeAngle360(truck.heading);
                
                allPositioned = false;
            } else {
                truck.positioned = true;
                truck.targetSpeed = 0.0f;
                truck.speed = 0.0f;
                truck.frontSteeringAngle = 0.0f;
                truck.rearSteeringAngle = 0.0f;
            }
        }
        
        /* Update instance */
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
    
    auto updateTruckLeaving = [dt](FireTruck& truck, bool& allLeft) {
        /* First turn, then leave */
        if (truck.isTurningBeforeLeave) {
            float headingDiff = truck.leaveHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            if (fabsf(headingDiff) > HEADING_TOLERANCE_DEG) {
                truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
                truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
                truck.targetSpeed = TRUCK_TURN_IN_PLACE_SPEED;
                truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
                
                float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
                truck.heading += turningRate * dt;
                truck.heading = NormalizeAngle360(truck.heading);
            } else {
                truck.isTurningBeforeLeave = false;
                truck.frontSteeringAngle = 0.0f;
                truck.rearSteeringAngle = 0.0f;
            }
        } else {
            /* Drive away */
            truck.targetSpeed = TRUCK_APPROACH_SPEED * TRUCK_LEAVING_SPEED_MULT;
            truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
            
            float headingRad = truck.heading * DEG_TO_RAD;
            float moveDistance = truck.speed * dt;
            truck.x += sinf(headingRad) * moveDistance;
            truck.z += -cosf(headingRad) * moveDistance;
            truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
            
            UpdateWheelRotationAngle(truck, moveDistance);
        }
        
        /* Check if truck has left the area */
        double acX = g_drLocalX ? XPLMGetDatad(g_drLocalX) : 0.0;
        double acZ = g_drLocalZ ? XPLMGetDatad(g_drLocalZ) : 0.0;
        
        double dx = truck.x - acX;
        double dz = truck.z - acZ;
        double dist = sqrt(dx * dx + dz * dz);
        
        if (dist > TRUCK_LEAVING_DISTANCE) {
            allLeft = true;
        }
        
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
    
    if (g_state == STATE_TRUCKS_APPROACHING || g_state == STATE_TRUCKS_POSITIONING) {
        bool allPositioned = true;
        updateTruckPosition(g_leftTruck, allPositioned);
        updateTruckPosition(g_rightTruck, allPositioned);
        
        if (allPositioned && g_leftTruck.positioned && g_rightTruck.positioned) {
            g_state = STATE_WATER_SPRAYING;
            RegisterDrawCallback();
            DebugLog("State changed to: %s", GetStateName(g_state));
        }
    } else if (g_state == STATE_TRUCKS_LEAVING) {
        bool leftDone = false, rightDone = false;
        updateTruckLeaving(g_leftTruck, leftDone);
        updateTruckLeaving(g_rightTruck, rightDone);
        
        if (leftDone && rightDone) {
            UnregisterDrawCallback();
            CleanupTruck(g_leftTruck);
            CleanupTruck(g_rightTruck);
            g_state = STATE_IDLE;
            UpdateMenuState();
            DebugLog("State changed to: %s", GetStateName(g_state));
        }
    }
}

static void RegisterDrawCallback() {
    if (!g_drawCallbackRegistered) {
        XPLMRegisterDrawCallback(DrawWaterParticles, WATER_DRAWING_PHASE, 0, nullptr);
        g_drawCallbackRegistered = true;
        DebugLog("Draw callback registered");
    }
}

static void UnregisterDrawCallback() {
    if (g_drawCallbackRegistered) {
        XPLMUnregisterDrawCallback(DrawWaterParticles, WATER_DRAWING_PHASE, 0, nullptr);
        g_drawCallbackRegistered = false;
        DebugLog("Draw callback unregistered");
    }
}

static int DrawWaterParticles(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    g_drawCallbackCallCount++;
    
    if (g_state != STATE_WATER_SPRAYING) {
        return 1;
    }
    
    /* Particles are already positioned via instances in UpdateWaterParticles */
    /* This callback exists for any additional drawing if needed */
    
    return 1;
}

/* X-Plane Plugin Entry Points */
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "Water Salute");
    strcpy(outSig, "com.watersalute.plugin");
    strcpy(outDesc, "Simulates water salute ceremony with fire trucks");
    
    DebugLog("========================================");
    DebugLog("WaterSalute plugin starting...");
    DebugLog("========================================");
    
    /* Initialize random number generator */
    g_rng.seed(static_cast<unsigned int>(time(nullptr)));
    
    /* Initialize datarefs */
    g_drOnGround = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    g_drGroundSpeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    g_drLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
    g_drLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
    g_drLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    g_drHeading = XPLMFindDataRef("sim/flightmodel/position/psi");
    g_drLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    g_drLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    
    g_drWingspan = XPLMFindDataRef("sim/aircraft/view/acf_semi_len_m");
    if (!g_drWingspan) {
        g_drWingspan = XPLMFindDataRef("sim/aircraft/overflow/acf_span");
    }
    
    /* Create terrain probe */
    g_terrainProbe = XPLMCreateProbe(xplm_ProbeY);
    
    /* Create menu */
    int menuContainerItem = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Water Salute", nullptr, 0);
    g_menuId = XPLMCreateMenu("Water Salute", XPLMFindPluginsMenu(), menuContainerItem, MenuHandler, nullptr);
    
    g_menuStartItem = XPLMAppendMenuItem(g_menuId, "Start Water Salute", (void*)1, 0);
    g_menuStopItem = XPLMAppendMenuItem(g_menuId, "Stop Water Salute", (void*)2, 0);
    
    UpdateMenuState();
    
    /* Register custom datarefs */
    RegisterCustomDataRefs();
    
    /* Find resources and load models */
    FindResourcePath();
    LoadFireTruckModel();
    LoadWaterDropModel();
    
    /* Register flight loop */
    XPLMCreateFlightLoop_t flightLoopParams;
    flightLoopParams.structSize = sizeof(XPLMCreateFlightLoop_t);
    flightLoopParams.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    flightLoopParams.callbackFunc = FlightLoopCallback;
    flightLoopParams.refcon = nullptr;
    
    g_flightLoopId = XPLMCreateFlightLoop(&flightLoopParams);
    XPLMScheduleFlightLoop(g_flightLoopId, -1.0f, 1);
    
    DebugLog("WaterSalute plugin started successfully");
    
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    DebugLog("WaterSalute plugin stopping...");
    
    /* Unregister callbacks */
    UnregisterDrawCallback();
    
    if (g_flightLoopId) {
        XPLMDestroyFlightLoop(g_flightLoopId);
        g_flightLoopId = nullptr;
    }
    
    /* Cleanup trucks */
    CleanupTruck(g_leftTruck);
    CleanupTruck(g_rightTruck);
    
    /* Unload models */
    if (g_truckObject) {
        XPLMUnloadObject(g_truckObject);
        g_truckObject = nullptr;
    }
    if (g_waterDropObject) {
        XPLMUnloadObject(g_waterDropObject);
        g_waterDropObject = nullptr;
    }
    
    /* Destroy probe */
    if (g_terrainProbe) {
        XPLMDestroyProbe(g_terrainProbe);
        g_terrainProbe = nullptr;
    }
    
    /* Unregister custom datarefs */
    UnregisterCustomDataRefs();
    
    DebugLog("WaterSalute plugin stopped");
}

PLUGIN_API int XPluginEnable(void) {
    DebugLog("WaterSalute plugin enabled");
    return 1;
}

PLUGIN_API void XPluginDisable(void) {
    DebugLog("WaterSalute plugin disabled");
    
    if (g_state != STATE_IDLE) {
        StopWaterSalute();
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMessage, void* inParam) {
    /* Handle any plugin messages if needed */
}
