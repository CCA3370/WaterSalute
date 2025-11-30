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
#include <cerrno>
#include <ctime>
#include <random>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <queue>
#include <unordered_map>
#include <sstream>
#include <fstream>
#include <limits>

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
static const float FEET_TO_METERS = 0.3048f;       /* Feet to meters conversion */
static const float MAX_GROUND_SPEED_KNOTS = 40.0f; /* Maximum ground speed for water salute */
static const float TRUCK_APPROACH_SPEED = 15.0f;   /* Fire truck approach speed in m/s */
static const float TRUCK_TURN_IN_PLACE_SPEED = 2.0f; /* Speed for turning in place (m/s) */
static const float TRUCK_LEAVING_SPEED_MULT = 2.0f / 3.0f;  /* Speed multiplier when leaving (2/3 of approach speed) */
static const float TRUCK_ACCELERATION = 3.0f;      /* Truck acceleration in m/s^2 for smooth speed transitions */
static const float TRUCK_DECELERATION = 4.0f;      /* Truck deceleration in m/s^2 for smooth braking */
static const float TRUCK_SLOWDOWN_DISTANCE = 30.0f; /* Distance at which truck starts slowing down (meters) */
static const float HEADING_TOLERANCE_DEG = 2.0f;   /* Tolerance for heading alignment (degrees) */
static const float TRUCK_LEAVING_DISTANCE = 600.0f;  /* Distance from aircraft to complete leaving (meters) */
static const float TRUCK_STOP_DISTANCE = 200.0f;   /* Distance in front of aircraft to stop (meters) */
static const float TRUCK_EXTRA_SPACING = 40.0f;    /* Extra spacing beyond wingspan (meters) */
static const float TRUCK_POSITIONING_THRESHOLD = 50.0f; /* Distance threshold to start positioning phase (meters) */
static const float WATER_JET_HEIGHT = 25.0f;       /* Maximum height of water arch (meters) */
static const float WATER_JET_DURATION = 0.8f;      /* Time for particle to reach apex */
static const float PARTICLE_LIFETIME = 4.0f;       /* Particle lifetime in seconds (longer for water spray effect) */
static const int   NUM_PARTICLES_PER_JET = 200;    /* Number of particles per water jet (more for denser spray) */
static const float PARTICLE_EMIT_RATE = 0.015f;    /* Time between particle emissions (faster for continuous stream) */
static const float PARTICLE_DRAG = 0.15f;          /* Air drag coefficient for particles */
static const float PARTICLE_TURBULENCE = 0.02f;    /* Turbulence amount for particle movement */
static const XPLMDrawingPhase WATER_DRAWING_PHASE = xplm_Phase_Modern3D; /* Drawing phase for water particles - use Modern3D for proper depth handling */

/* Wingspan validation constants */
static const float MIN_SEMISPAN_METERS = 2.5f;     /* Minimum semispan (half wingspan) in meters (small aircraft ~5m wingspan) */
static const float MAX_SEMISPAN_METERS = 45.0f;    /* Maximum semispan in meters (A380 wingspan ~80m / 2) */
static const float MIN_WINGSPAN_METERS = 5.0f;     /* Minimum valid wingspan in meters */
static const float MAX_WINGSPAN_METERS = 90.0f;    /* Maximum valid wingspan in meters (A380 ~80m) */
static const float DEFAULT_WINGSPAN_METERS = 30.0f; /* Default wingspan if not available */

/* Debug configuration */
#ifndef WATERSALUTE_DEBUG_VERBOSE
static const bool DEBUG_VERBOSE = false;           /* Enable verbose debug logging (set to true for debugging) */
#else
static const bool DEBUG_VERBOSE = true;            /* Verbose logging enabled via build flag */
#endif
static const float DEBUG_LOG_INTERVAL = 2.0f;      /* Interval for periodic debug logs (seconds) */
static const size_t DEBUG_BUFFER_SIZE = 1024;      /* Buffer size for debug message formatting */
static const size_t DEBUG_LOG_PREFIX_SIZE = 32;    /* Max size of log prefix "WaterSalute [VERBOSE]: " */
static const size_t DEBUG_LOG_MSG_SIZE = DEBUG_BUFFER_SIZE + DEBUG_LOG_PREFIX_SIZE + 2; /* +2 for newline and null terminator */

/* Road network constants */
static const float ROAD_SEARCH_RADIUS = 5000.0f;   /* Maximum distance to search for roads (meters) */
static const float PATH_NODE_DISTANCE = 30.0f;     /* Minimum distance between path nodes (meters) */
static const float TRUCK_SPAWN_DISTANCE = 500.0f;  /* Distance from aircraft to spawn trucks (meters) */
static const float TURN_ANTICIPATION = 15.0f;      /* Distance ahead to start turning for smooth path following */
static const float MIN_TURN_RADIUS = 8.0f;         /* Minimum turning radius for 8x8 truck (meters) */
static const float PATH_REACH_THRESHOLD = 5.0f;    /* Distance to consider a waypoint reached (meters) */
static const float BEZIER_SMOOTHING_FACTOR = 0.25f; /* Control point distance as fraction of segment length */
static const int MAX_PATH_NODES = 500;             /* Maximum number of nodes in a planned path */
static const double EARTH_RADIUS_METERS = 6371000.0; /* Earth radius for lat/lon to meters conversion */
static const float MIN_STEERING_TANGENT = 0.01f;   /* Minimum tangent value to prevent division by zero in turn radius calc */

/*
 * Road Network Data Structures for apt.dat parsing
 * 
 * apt.dat format for ground routes:
 * - 1201: Node (waypoint) - latitude longitude type name
 * - 1202: Edge (basic taxiway connection) - node1 node2 direction surface
 * - 1204: Edge (with runway info) - node1 node2 direction active_zone
 * - 1206: Ground truck route edge - node1 node2 direction truck_types active_zone
 */

/* Road network node (from apt.dat 1201 records) */
struct RoadNode {
    std::string name;        /* Node identifier */
    double lat;              /* Latitude in degrees */
    double lon;              /* Longitude in degrees */
    double x, z;             /* Local OpenGL coordinates (calculated) */
    std::string nodeType;    /* "both", "dest", "junction" */
    std::vector<size_t> connectedEdges; /* Indices into edge array */
};

/* Road network edge (from apt.dat 1202/1204/1206 records) */
struct RoadEdge {
    size_t node1Idx;         /* Index into nodes array */
    size_t node2Idx;         /* Index into nodes array */
    bool isOneWay;           /* True if edge is one-way */
    bool isFireTruckRoute;   /* True if this edge allows fire trucks (1206) */
    float length;            /* Edge length in meters (calculated) */
    std::string surfaceType; /* "taxiway", "runway", "ramp", etc. */
};

/* Road network for an airport */
struct RoadNetwork {
    std::string airportId;   /* ICAO code */
    double refLat, refLon;   /* Reference point for coordinate conversion */
    std::vector<RoadNode> nodes;
    std::vector<RoadEdge> edges;
    std::unordered_map<std::string, size_t> nodeNameToIndex; /* Fast lookup */
    bool isLoaded;
};

/* Path waypoint for truck navigation */
struct PathWaypoint {
    double x, z;             /* Local OpenGL coordinates */
    float targetHeading;     /* Desired heading when reaching this point */
    float speed;             /* Target speed at this waypoint */
    bool isSmoothed;         /* Whether this waypoint has been Bezier-smoothed */
};

/* Planned route for a truck */
struct PlannedRoute {
    std::vector<PathWaypoint> waypoints;
    size_t currentWaypointIndex;
    bool isValid;
    bool isCompleted;
};

/* A* pathfinding node */
struct AStarNode {
    size_t nodeIndex;
    float gScore;            /* Cost from start to this node */
    float fScore;            /* gScore + heuristic estimate to goal */
    size_t parentIndex;      /* Parent node index for path reconstruction */
    bool operator>(const AStarNode& other) const {
        return fScore > other.fScore;
    }
};

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
    XPLMInstanceRef instance; /* Instance for rendering this particle */
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
    float frontSteeringAngle;     /* Front axles steering angle in degrees (-45 to 45), negative when turning left */
    float rearSteeringAngle;      /* Rear axle steering angle in degrees (-45 to 45), opposite sign to frontSteeringAngle for counter-steering */
    float wheelRotationAngle;     /* Wheel rotation angle in degrees (0-360°), updated based on vehicle speed */
    float cannonPitch;       /* Water cannon pitch angle in degrees (0 to 90) */
    float cannonYaw;         /* Water cannon yaw angle in degrees (-180 to 180) */
    float speed;             /* Current speed in m/s */
    float targetSpeed;       /* Target speed in m/s (for smooth acceleration/deceleration) */
    bool isTurningBeforeLeave; /* Flag: true when truck is turning before leaving */
    float leaveHeading;      /* Heading to use when leaving */
    PlannedRoute route;      /* Planned path from road network */
    bool useRoadNetwork;     /* Whether to follow road network or direct approach */
};

/* Global variables */
static PluginState g_state = STATE_IDLE;
static XPLMMenuID g_menuId = nullptr;
static int g_menuStartItem = -1;
static int g_menuStopItem = -1;

static XPLMObjectRef g_truckObject = nullptr;
static XPLMObjectRef g_waterDropObject = nullptr;  /* Water droplet model for particle instances */
static FireTruck g_leftTruck;
static FireTruck g_rightTruck;
static XPLMProbeRef g_terrainProbe = nullptr;

/* Road network for current airport */
static RoadNetwork g_roadNetwork;

static XPLMFlightLoopID g_flightLoopId = nullptr;

static bool g_drawCallbackRegistered = false;

/* Fast random number generator for particle effects */
static std::mt19937 g_rng;
static std::uniform_real_distribution<float> g_randomDist(-0.5f, 0.5f);

/* Datarefs from X-Plane */
static XPLMDataRef g_drOnGround = nullptr;
static XPLMDataRef g_drGroundSpeed = nullptr;
static XPLMDataRef g_drLocalX = nullptr;
static XPLMDataRef g_drLocalY = nullptr;
static XPLMDataRef g_drLocalZ = nullptr;
static XPLMDataRef g_drHeading = nullptr;
static XPLMDataRef g_drWingspan = nullptr;
static XPLMDataRef g_drLatitude = nullptr;             /* Aircraft latitude in degrees */
static XPLMDataRef g_drLongitude = nullptr;            /* Aircraft longitude in degrees */

/* Custom datarefs published by this plugin */
/* Fire truck control datarefs - left truck (index 0) and right truck (index 1) */
static XPLMDataRef g_drTruckFrontSteeringAngle = nullptr;  /* watersalute/truck/front_steering_angle - float array[2] */
static XPLMDataRef g_drTruckRearSteeringAngle = nullptr;   /* watersalute/truck/rear_steering_angle - float array[2] */
static XPLMDataRef g_drTruckWheelAngle = nullptr;          /* watersalute/truck/wheel_rotation_angle - float array[2] */
static XPLMDataRef g_drTruckCannonPitch = nullptr;         /* watersalute/truck/cannon_pitch - float array[2] */
static XPLMDataRef g_drTruckCannonYaw = nullptr;           /* watersalute/truck/cannon_yaw - float array[2] */
static XPLMDataRef g_drTruckSpeed = nullptr;               /* watersalute/truck/speed - float array[2] (read-only) */

/* Wheel physics constants */
static const float WHEEL_RADIUS = 0.5f;                 /* Wheel radius in meters */
static const float MAX_STEERING_ANGLE = 45.0f;          /* Maximum steering angle in degrees */
static const float WHEELBASE = 6.0f;                    /* Distance between front and rear axles in meters (for 8x8 truck) */
static const float MIN_CANNON_PITCH = 0.0f;             /* Minimum cannon pitch angle */
static const float MAX_CANNON_PITCH = 90.0f;            /* Maximum cannon pitch angle */
static const float DEFAULT_CANNON_PITCH = 45.0f;        /* Default cannon pitch angle for water arc */
static const float PI = 3.14159265f;                    /* Pi constant */
static const float DEG_TO_RAD = PI / 180.0f;            /* Degrees to radians conversion */
static const float RAD_TO_DEG = 180.0f / PI;            /* Radians to degrees conversion */
static const float REAR_STEER_RATIO = 0.4f;             /* Ratio for rear axle counter-steering */

/*
 * NormalizeAngle360 - Normalize angle to 0-360° range
 */
static float NormalizeAngle360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

/*
 * UpdateWheelRotationAngle - Calculate and update wheel rotation angle based on distance moved
 * @param truck Reference to the truck to update
 * @param distanceMoved Distance moved in meters
 */
static void UpdateWheelRotationAngle(FireTruck& truck, float distanceMoved) {
    if (distanceMoved > 0.0f) {
        float angularDisplacementRad = distanceMoved / WHEEL_RADIUS;
        float angularDisplacementDeg = angularDisplacementRad * RAD_TO_DEG;
        truck.wheelRotationAngle = NormalizeAngle360(truck.wheelRotationAngle + angularDisplacementDeg);
    }
}

/*
 * CalculateTurningRate - Calculate vehicle turning rate using Ackermann steering geometry
 *
 * Uses the Ackermann method for vehicles with front and rear wheel steering:
 *   turning_radius R = wheelbase / (tan(front_angle) + tan(|rear_angle|))
 *   turning_rate ω = speed / R = speed * (tan(front_angle) + tan(|rear_angle|)) / wheelbase
 *
 * For 8x8 trucks with counter-steering rear axle:
 *   - Front axle steering angle: δ_f (primary control)
 *   - Rear axle steering angle: δ_r = -δ_f * REAR_STEER_RATIO (counter-steering)
 *   - Combined effect increases maneuverability by reducing turn radius
 *
 * Note: This simplified Ackermann model treats all wheels as having the same
 * steering angle, without differentiating inner/outer wheel angles and speeds.
 *
 * @param speed Vehicle speed in m/s
 * @param frontSteerAngleDeg Front wheel steering angle in degrees (-45 to 45)
 * @param rearSteerAngleDeg Rear wheel steering angle in degrees (-45 to 45)
 * @return Turning rate in degrees per second
 */
static float CalculateTurningRate(float speed, float frontSteerAngleDeg, float rearSteerAngleDeg) {
    if (fabsf(speed) < 0.01f || fabsf(frontSteerAngleDeg) < 0.1f) {
        return 0.0f;
    }
    
    /* Convert steering angles to radians */
    float frontSteerAngleRad = frontSteerAngleDeg * DEG_TO_RAD;
    float rearSteerAngleRad = rearSteerAngleDeg * DEG_TO_RAD;
    
    /* Ackermann formula for front and rear steering:
     * turning_rate = speed * (tan(front_angle) + tan(|rear_angle|)) / wheelbase
     * 
     * Since rear axle uses counter-steering (rearSteerAngleDeg is negative when 
     * frontSteerAngleDeg is positive), we have:
     *   tan(rearSteerAngleRad) = tan(-|angle|) = -tan(|angle|)
     * Therefore: tanFront - tanRear = tanFront - (-tan(|rear|)) = tanFront + tan(|rear|)
     */
    float tanFront = tanf(frontSteerAngleRad);
    float tanRear = tanf(rearSteerAngleRad);
    float combinedTan = tanFront - tanRear;
    
    /* Calculate turning rate in rad/s using Ackermann method */
    float turningRateRad = (speed * combinedTan) / WHEELBASE;
    
    return turningRateRad * RAD_TO_DEG;
}

/*
 * CalculateRearSteeringAngle - Calculate rear axle steering angle from front axle angle
 * Rear axle uses counter-steering (opposite direction) for better maneuverability
 * @param frontSteerAngle Front wheel steering angle in degrees
 * @return Rear axle steering angle in degrees (clamped to ±MAX_STEERING_ANGLE)
 */
static float CalculateRearSteeringAngle(float frontSteerAngle) {
    /* Rear axle counter-steers: opposite direction at REAR_STEER_RATIO */
    float rearAngle = -frontSteerAngle * REAR_STEER_RATIO;
    if (rearAngle > MAX_STEERING_ANGLE) rearAngle = MAX_STEERING_ANGLE;
    if (rearAngle < -MAX_STEERING_ANGLE) rearAngle = -MAX_STEERING_ANGLE;
    return rearAngle;
}

/*
 * ClampSteeringAngle - Clamp steering angle to valid range (-45 to 45 degrees)
 * @param angle Steering angle in degrees
 * @return Clamped steering angle
 */
static float ClampSteeringAngle(float angle) {
    if (angle > MAX_STEERING_ANGLE) return MAX_STEERING_ANGLE;
    if (angle < -MAX_STEERING_ANGLE) return -MAX_STEERING_ANGLE;
    return angle;
}

/*
 * UpdateSpeedSmooth - Smoothly transition current speed to target speed
 * Uses acceleration for speeding up and deceleration for slowing down
 * @param currentSpeed Current speed in m/s
 * @param targetSpeed Target speed in m/s
 * @param dt Delta time in seconds
 * @return New current speed
 */
static float UpdateSpeedSmooth(float currentSpeed, float targetSpeed, float dt) {
    if (currentSpeed < targetSpeed) {
        /* Accelerating */
        currentSpeed += TRUCK_ACCELERATION * dt;
        if (currentSpeed > targetSpeed) {
            currentSpeed = targetSpeed;
        }
    } else if (currentSpeed > targetSpeed) {
        /* Decelerating */
        currentSpeed -= TRUCK_DECELERATION * dt;
        if (currentSpeed < targetSpeed) {
            currentSpeed = targetSpeed;
        }
    }
    return currentSpeed;
}

static char g_pluginPath[512];
static char g_resourcePath[512];  /* Path to resources directory */
static float g_debugLogTimer = 0.0f;               /* Timer for periodic debug logging */
static int g_drawCallbackCallCount = 0;            /* Counter for draw callback calls */
static int g_particlesEmittedTotal = 0;            /* Total particles emitted */

/* Static array for instance creation (no datarefs needed for water droplets) */
static const char* g_noDataRefs[] = { nullptr };

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
static bool FindResourcePath();
static bool LoadFireTruckModel();
static bool LoadWaterDropModel();
static int DrawWaterParticles(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);
static void RegisterDrawCallback();
static void UnregisterDrawCallback();
static void RegisterCustomDataRefs();
static void UnregisterCustomDataRefs();

/* Road network forward declarations */
static bool LoadAptDat(double acLat, double acLon);
static void LatLonToLocal(double lat, double lon, double refLat, double refLon, double& x, double& z);
static void LocalToLatLon(double x, double z, double refLat, double refLon, double& lat, double& lon);
static size_t FindNearestNode(double x, double z, bool firetruckRoutesOnly);
static bool FindPath(size_t startNode, size_t goalNode, std::vector<size_t>& path);
static PlannedRoute PlanRouteToTarget(double startX, double startZ, double targetX, double targetZ, float targetHeading);
static void SmoothPath(PlannedRoute& route);
static void UpdateTruckFollowingPath(FireTruck& truck, float dt);

/* Helper function to get truck by index (0 = left, 1 = right) */
static FireTruck* GetTruckByIndex(int index) {
    if (index == 0) return &g_leftTruck;
    if (index == 1) return &g_rightTruck;
    return nullptr;
}

/* Custom dataref callback functions */

/* Front axles steering angle accessor (float array[2]) - for 8x8 truck, front two bogies share this */
static int GetFrontSteeringAngle(void* inRefcon, float* outValues, int inOffset, int inMax) {
    (void)inRefcon;
    if (outValues == nullptr) {
        return 2; /* Return array size */
    }
    int count = 0;
    for (int i = inOffset; i < 2 && count < inMax; i++, count++) {
        FireTruck* truck = GetTruckByIndex(i);
        outValues[count] = truck ? truck->frontSteeringAngle : 0.0f;
    }
    return count;
}

static void SetFrontSteeringAngle(void* inRefcon, float* inValues, int inOffset, int inCount) {
    (void)inRefcon;
    for (int i = 0; i < inCount && (inOffset + i) < 2; i++) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            /* Clamp steering angle to valid range */
            float angle = inValues[i];
            if (angle < -MAX_STEERING_ANGLE) angle = -MAX_STEERING_ANGLE;
            if (angle > MAX_STEERING_ANGLE) angle = MAX_STEERING_ANGLE;
            truck->frontSteeringAngle = angle;
            
            /* Mirror steering to the other truck (opposite direction for symmetrical movement) */
            if (inOffset + i == 0) {
                g_rightTruck.frontSteeringAngle = -angle;
            } else if (inOffset + i == 1) {
                g_leftTruck.frontSteeringAngle = -angle;
            }
        }
    }
}

/* Rear axle steering angle accessor (float array[2]) - for 8x8 truck counter-steering */
static int GetRearSteeringAngle(void* inRefcon, float* outValues, int inOffset, int inMax) {
    (void)inRefcon;
    if (outValues == nullptr) {
        return 2; /* Return array size */
    }
    int count = 0;
    for (int i = inOffset; i < 2 && count < inMax; i++, count++) {
        FireTruck* truck = GetTruckByIndex(i);
        outValues[count] = truck ? truck->rearSteeringAngle : 0.0f;
    }
    return count;
}

static void SetRearSteeringAngle(void* inRefcon, float* inValues, int inOffset, int inCount) {
    (void)inRefcon;
    for (int i = 0; i < inCount && (inOffset + i) < 2; i++) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            /* Clamp steering angle to valid range */
            float angle = inValues[i];
            if (angle < -MAX_STEERING_ANGLE) angle = -MAX_STEERING_ANGLE;
            if (angle > MAX_STEERING_ANGLE) angle = MAX_STEERING_ANGLE;
            truck->rearSteeringAngle = angle;
            
            /* Mirror steering to the other truck (opposite direction for symmetrical movement) */
            if (inOffset + i == 0) {
                g_rightTruck.rearSteeringAngle = -angle;
            } else if (inOffset + i == 1) {
                g_leftTruck.rearSteeringAngle = -angle;
            }
        }
    }
}

/* Wheel rotation angle accessor (float array[2]) - read-only, calculated from speed in real-time (0-360°) */
static int GetWheelRotationAngle(void* inRefcon, float* outValues, int inOffset, int inMax) {
    (void)inRefcon;
    if (outValues == nullptr) {
        return 2; /* Return array size */
    }
    int count = 0;
    for (int i = inOffset; i < 2 && count < inMax; i++, count++) {
        FireTruck* truck = GetTruckByIndex(i);
        outValues[count] = truck ? truck->wheelRotationAngle : 0.0f;
    }
    return count;
}

/* Cannon pitch accessor (float array[2]) */
static int GetCannonPitch(void* inRefcon, float* outValues, int inOffset, int inMax) {
    (void)inRefcon;
    if (outValues == nullptr) {
        return 2; /* Return array size */
    }
    int count = 0;
    for (int i = inOffset; i < 2 && count < inMax; i++, count++) {
        FireTruck* truck = GetTruckByIndex(i);
        outValues[count] = truck ? truck->cannonPitch : DEFAULT_CANNON_PITCH;
    }
    return count;
}

static void SetCannonPitch(void* inRefcon, float* inValues, int inOffset, int inCount) {
    (void)inRefcon;
    for (int i = 0; i < inCount && (inOffset + i) < 2; i++) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            /* Clamp cannon pitch to valid range */
            float pitch = inValues[i];
            if (pitch < MIN_CANNON_PITCH) pitch = MIN_CANNON_PITCH;
            if (pitch > MAX_CANNON_PITCH) pitch = MAX_CANNON_PITCH;
            truck->cannonPitch = pitch;
            
            /* Sync cannon pitch to the other truck (same value) */
            if (inOffset + i == 0) {
                g_rightTruck.cannonPitch = pitch;
            } else if (inOffset + i == 1) {
                g_leftTruck.cannonPitch = pitch;
            }
        }
    }
}

/* Cannon yaw accessor (float array[2]) */
static int GetCannonYaw(void* inRefcon, float* outValues, int inOffset, int inMax) {
    (void)inRefcon;
    if (outValues == nullptr) {
        return 2; /* Return array size */
    }
    int count = 0;
    for (int i = inOffset; i < 2 && count < inMax; i++, count++) {
        FireTruck* truck = GetTruckByIndex(i);
        outValues[count] = truck ? truck->cannonYaw : 0.0f;
    }
    return count;
}

static void SetCannonYaw(void* inRefcon, float* inValues, int inOffset, int inCount) {
    (void)inRefcon;
    for (int i = 0; i < inCount && (inOffset + i) < 2; i++) {
        FireTruck* truck = GetTruckByIndex(inOffset + i);
        if (truck) {
            /* Normalize yaw to -180 to 180 range */
            float yaw = inValues[i];
            while (yaw > 180.0f) yaw -= 360.0f;
            while (yaw < -180.0f) yaw += 360.0f;
            truck->cannonYaw = yaw;
            
            /* Sync cannon yaw to the other truck (same value) */
            if (inOffset + i == 0) {
                g_rightTruck.cannonYaw = yaw;
            } else if (inOffset + i == 1) {
                g_leftTruck.cannonYaw = yaw;
            }
        }
    }
}

/* Truck speed accessor (float array[2]) - read-only */
static int GetTruckSpeed(void* inRefcon, float* outValues, int inOffset, int inMax) {
    (void)inRefcon;
    if (outValues == nullptr) {
        return 2; /* Return array size */
    }
    int count = 0;
    for (int i = inOffset; i < 2 && count < inMax; i++, count++) {
        FireTruck* truck = GetTruckByIndex(i);
        outValues[count] = truck ? truck->speed : 0.0f;
    }
    return count;
}

/* Debug logging */
static void DebugLog(const char* format, ...) {
    char buffer[DEBUG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char logMsg[DEBUG_LOG_MSG_SIZE];
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
    
    char buffer[DEBUG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char logMsg[DEBUG_LOG_MSG_SIZE];
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
    DebugLog("  Left Truck: Speed=%.2f m/s, WheelAngle=%.1f deg, FrontSteer=%.1f deg, RearSteer=%.1f deg",
             g_leftTruck.speed, g_leftTruck.wheelRotationAngle, g_leftTruck.frontSteeringAngle, g_leftTruck.rearSteeringAngle);
    DebugLog("  Right Truck: Speed=%.2f m/s, WheelAngle=%.1f deg, FrontSteer=%.1f deg, RearSteer=%.1f deg",
             g_rightTruck.speed, g_rightTruck.wheelRotationAngle, g_rightTruck.frontSteeringAngle, g_rightTruck.rearSteeringAngle);
    DebugLog("  Left Cannon: Pitch=%.1f deg, Yaw=%.1f deg", 
             g_leftTruck.cannonPitch, g_leftTruck.cannonYaw);
    DebugLog("  Right Cannon: Pitch=%.1f deg, Yaw=%.1f deg", 
             g_rightTruck.cannonPitch, g_rightTruck.cannonYaw);
    DebugLog("  Left Truck Particles: %zu", g_leftTruck.particles.size());
    DebugLog("  Right Truck Particles: %zu", g_rightTruck.particles.size());
    DebugLog("  Draw Callback Registered: %s", g_drawCallbackRegistered ? "YES" : "NO");
    DebugLog("  Draw Callback Call Count: %d", g_drawCallbackCallCount);
    DebugLog("  Total Particles Emitted: %d", g_particlesEmittedTotal);
    DebugLog("===================");
}

/*
 * RegisterCustomDataRefs - Register all custom datarefs for fire truck control
 * 
 * Datarefs created:
 * - watersalute/truck/front_steering_angle: float array[2], read/write
 *   Front axles steering angle in degrees (-45 to 45)
 *   For 8x8 truck, front two bogies share this angle
 *   Negative when turning left, positive when turning right
 *   Index 0 = left truck, Index 1 = right truck
 * 
 * - watersalute/truck/rear_steering_angle: float array[2], read/write
 *   Rear axle steering angle in degrees (-45 to 45)
 *   Uses counter-steering (opposite sign to front axles) for better maneuverability
 *   Index 0 = left truck, Index 1 = right truck
 * 
 * - watersalute/truck/wheel_rotation_angle: float array[2], read-only
 *   Wheel rotation angle in degrees (0-360°)
 *   Calculated in real-time based on truck speed and wheel radius
 * 
 * - watersalute/truck/cannon_pitch: float array[2], read/write
 *   Water cannon pitch angle in degrees (0 to 90)
 *   Used for controlling the elevation of water jet
 * 
 * - watersalute/truck/cannon_yaw: float array[2], read/write
 *   Water cannon yaw angle in degrees (-180 to 180)
 *   Used for controlling the horizontal direction of water jet
 * 
 * - watersalute/truck/speed: float array[2], read-only
 *   Current truck speed in meters per second
 */
static void RegisterCustomDataRefs() {
    DebugLog("Registering custom datarefs...");
    
    /* Front axles steering angle - writable float array */
    g_drTruckFrontSteeringAngle = XPLMRegisterDataAccessor(
        "watersalute/truck/front_steering_angle",
        xplmType_FloatArray,
        1, /* writable */
        nullptr, nullptr,   /* int accessors */
        nullptr, nullptr,   /* float accessors */
        nullptr, nullptr,   /* double accessors */
        nullptr, nullptr,   /* int array accessors */
        GetFrontSteeringAngle, SetFrontSteeringAngle,  /* float array accessors */
        nullptr, nullptr,   /* data accessors */
        nullptr, nullptr    /* refcons */
    );
    DebugLog("  Registered: watersalute/truck/front_steering_angle");
    
    /* Rear axle steering angle - writable float array */
    g_drTruckRearSteeringAngle = XPLMRegisterDataAccessor(
        "watersalute/truck/rear_steering_angle",
        xplmType_FloatArray,
        1, /* writable */
        nullptr, nullptr,   /* int accessors */
        nullptr, nullptr,   /* float accessors */
        nullptr, nullptr,   /* double accessors */
        nullptr, nullptr,   /* int array accessors */
        GetRearSteeringAngle, SetRearSteeringAngle,  /* float array accessors */
        nullptr, nullptr,   /* data accessors */
        nullptr, nullptr    /* refcons */
    );
    DebugLog("  Registered: watersalute/truck/rear_steering_angle");
    
    /* Wheel rotation angle - read-only float array (calculated from speed) */
    g_drTruckWheelAngle = XPLMRegisterDataAccessor(
        "watersalute/truck/wheel_rotation_angle",
        xplmType_FloatArray,
        0, /* read-only */
        nullptr, nullptr,   /* int accessors */
        nullptr, nullptr,   /* float accessors */
        nullptr, nullptr,   /* double accessors */
        nullptr, nullptr,   /* int array accessors */
        GetWheelRotationAngle, nullptr,  /* float array accessors - read only */
        nullptr, nullptr,   /* data accessors */
        nullptr, nullptr    /* refcons */
    );
    DebugLog("  Registered: watersalute/truck/wheel_rotation_angle");
    
    /* Cannon pitch - writable float array */
    g_drTruckCannonPitch = XPLMRegisterDataAccessor(
        "watersalute/truck/cannon_pitch",
        xplmType_FloatArray,
        1, /* writable */
        nullptr, nullptr,   /* int accessors */
        nullptr, nullptr,   /* float accessors */
        nullptr, nullptr,   /* double accessors */
        nullptr, nullptr,   /* int array accessors */
        GetCannonPitch, SetCannonPitch,  /* float array accessors */
        nullptr, nullptr,   /* data accessors */
        nullptr, nullptr    /* refcons */
    );
    DebugLog("  Registered: watersalute/truck/cannon_pitch");
    
    /* Cannon yaw - writable float array */
    g_drTruckCannonYaw = XPLMRegisterDataAccessor(
        "watersalute/truck/cannon_yaw",
        xplmType_FloatArray,
        1, /* writable */
        nullptr, nullptr,   /* int accessors */
        nullptr, nullptr,   /* float accessors */
        nullptr, nullptr,   /* double accessors */
        nullptr, nullptr,   /* int array accessors */
        GetCannonYaw, SetCannonYaw,  /* float array accessors */
        nullptr, nullptr,   /* data accessors */
        nullptr, nullptr    /* refcons */
    );
    DebugLog("  Registered: watersalute/truck/cannon_yaw");
    
    /* Truck speed - read-only float array */
    g_drTruckSpeed = XPLMRegisterDataAccessor(
        "watersalute/truck/speed",
        xplmType_FloatArray,
        0, /* read-only */
        nullptr, nullptr,   /* int accessors */
        nullptr, nullptr,   /* float accessors */
        nullptr, nullptr,   /* double accessors */
        nullptr, nullptr,   /* int array accessors */
        GetTruckSpeed, nullptr,  /* float array accessors - read only */
        nullptr, nullptr,   /* data accessors */
        nullptr, nullptr    /* refcons */
    );
    DebugLog("  Registered: watersalute/truck/speed");
    
    DebugLog("Custom datarefs registered successfully");
}

/*
 * UnregisterCustomDataRefs - Unregister all custom datarefs
 */
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
    DebugLog("Plugin directory: %s", g_pluginPath);
    
    /* Find the resources directory (handles different plugin layouts) */
    FindResourcePath();
    
    /* Initialize datarefs */
    g_drOnGround = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    g_drGroundSpeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    g_drLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
    g_drLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
    g_drLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    g_drHeading = XPLMFindDataRef("sim/flightmodel/position/psi");
    g_drLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    g_drLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    
    /* Find wingspan dataref - try several possible datarefs
     * 
     * Based on analysis of BetterPushback project (olivierbutler/BetterPusbackMod):
     * - BetterPushback reads wing geometry directly from ACF file properties:
     *   - _wing/%d/_semilen_SEG (wing segment semi-length in feet)
     *   - acf/_size_x (half wingspan/semispan in feet, older XP versions)
     *   - Or calculates from wing outline point coordinates
     * 
     * For simpler dataref-based approach, we try:
     * - sim/aircraft/view/acf_semi_len_m: aircraft half-width (semispan) in meters
     * - sim/aircraft/overflow/acf_span: wingspan in feet (if available)
     * 
     * Note: sim/aircraft/view/acf_Vso is stall speed in knots, NOT wingspan!
     * 
     * Try multiple possible datarefs for compatibility with different X-Plane versions.
     */
    g_drWingspan = XPLMFindDataRef("sim/aircraft/view/acf_semi_len_m");  /* Semispan in meters (XP12 primary) */
    if (!g_drWingspan) {
        g_drWingspan = XPLMFindDataRef("sim/aircraft/overflow/acf_span");  /* Wingspan in feet (fallback) */
    }
    if (!g_drWingspan) {
        g_drWingspan = XPLMFindDataRef("sim/aircraft/parts/acf_wing_span");
    }
    if (!g_drWingspan) {
        g_drWingspan = XPLMFindDataRef("sim/aircraft/view/acf_wing_span");
    }
    /* Note: If wingspan dataref not found, a default value of 30m is used in StartWaterSalute */
    if (g_drWingspan) {
        DebugLog("Wingspan dataref found (will verify value at water salute start)");
    } else {
        DebugLog("WARNING: No wingspan dataref found, will use default 30m");
    }
    
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
    
    /* Load water droplet model for particle rendering */
    LoadWaterDropModel();
    
    /* Register custom datarefs for fire truck control */
    RegisterCustomDataRefs();
    
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
    
    /* Unregister custom datarefs */
    UnregisterCustomDataRefs();
    
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
    
    if (g_waterDropObject) {
        XPLMUnloadObject(g_waterDropObject);
        g_waterDropObject = nullptr;
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
    /* Seed random number generators for particle effects */
    unsigned int seed = static_cast<unsigned int>(time(nullptr));
    srand(seed);
    g_rng.seed(seed);
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
 * FindResourcePath - Find the resources directory
 * 
 * X-Plane plugins can be installed in different directory structures:
 * 1. Flat: WaterSalute/WaterSalute.xpl with resources at WaterSalute/resources/
 * 2. Platform-specific: WaterSalute/win_64/WaterSalute.xpl (or mac_x64/, lin_x64/)
 *    with resources at WaterSalute/resources/
 * 
 * This function tries both layouts and sets g_resourcePath to the correct path.
 */
static bool FindResourcePath() {
    char testPath[1024];
    FILE* testFile = nullptr;
    
    /* First, try the direct path (resources in same directory as plugin) */
    snprintf(testPath, sizeof(testPath), "%s/resources/firetruck.obj", g_pluginPath);
    DebugLog("Trying resource path 1: %s", testPath);
    
    testFile = fopen(testPath, "r");
    if (testFile) {
        fclose(testFile);
        snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", g_pluginPath);
        DebugLog("Found resources at: %s", g_resourcePath);
        return true;
    }
    
    /* Second, try parent directory (for platform-specific subdirectory layout) */
    /* Use snprintf for safer string copying */
    char parentPath[512];
    snprintf(parentPath, sizeof(parentPath), "%s", g_pluginPath);
    
    char* lastSlash = strrchr(parentPath, '/');
    if (!lastSlash) lastSlash = strrchr(parentPath, '\\');
    if (lastSlash) {
        *lastSlash = '\0';
        
        snprintf(testPath, sizeof(testPath), "%s/resources/firetruck.obj", parentPath);
        DebugLog("Trying resource path 2: %s", testPath);
        
        testFile = fopen(testPath, "r");
        if (testFile) {
            fclose(testFile);
            snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", parentPath);
            DebugLog("Found resources at: %s", g_resourcePath);
            return true;
        }
        
        /* Third, try going up two directories (for deeply nested structures) */
        /* Only attempt if previous directory traversal was successful */
        char* secondLastSlash = strrchr(parentPath, '/');
        if (!secondLastSlash) secondLastSlash = strrchr(parentPath, '\\');
        if (secondLastSlash) {
            *secondLastSlash = '\0';
            
            snprintf(testPath, sizeof(testPath), "%s/resources/firetruck.obj", parentPath);
            DebugLog("Trying resource path 3: %s", testPath);
            
            testFile = fopen(testPath, "r");
            if (testFile) {
                fclose(testFile);
                snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", parentPath);
                DebugLog("Found resources at: %s", g_resourcePath);
                return true;
            }
        }
    }
    
    DebugLog("ERROR: Could not find resources directory!");
    DebugLog("  Searched in plugin path and parent directories");
    DebugLog("  Make sure 'resources/firetruck.obj' exists relative to plugin location");
    
    /* Fallback to default (will likely fail, but provides debug info) */
    snprintf(g_resourcePath, sizeof(g_resourcePath), "%s/resources", g_pluginPath);
    return false;
}

/*
 * LoadFireTruckModel - Load the fire truck OBJ file
 */
static bool LoadFireTruckModel() {
    /* Build path to fire truck model */
    char modelPath[1024];
    snprintf(modelPath, sizeof(modelPath), "%s/firetruck.obj", g_resourcePath);
    
    DebugLog("Loading fire truck model from: %s", modelPath);
    DebugLog("Resource path base: %s", g_resourcePath);
    
    /* Check if file exists by trying to open it */
    FILE* testFile = fopen(modelPath, "r");
    if (testFile) {
        fclose(testFile);
        DebugLog("Model file exists and is readable");
    } else {
        int savedErrno = errno;
        DebugLog("WARNING: Cannot open model file");
        DebugLog("  Error: %s (errno=%d)", strerror(savedErrno), savedErrno);
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
 * LoadWaterDropModel - Load the water droplet OBJ file for particle instances
 */
static bool LoadWaterDropModel() {
    /* Build path to water droplet model */
    char modelPath[1024];
    snprintf(modelPath, sizeof(modelPath), "%s/waterdrop.obj", g_resourcePath);
    
    DebugLog("Loading water droplet model from: %s", modelPath);
    
    /* Check if file exists by trying to open it */
    FILE* testFile = fopen(modelPath, "r");
    if (testFile) {
        fclose(testFile);
        DebugLog("Water droplet model file exists and is readable");
    } else {
        int savedErrno = errno;
        DebugLog("WARNING: Cannot open water droplet model file");
        DebugLog("  Error: %s (errno=%d)", strerror(savedErrno), savedErrno);
    }
    
    g_waterDropObject = XPLMLoadObject(modelPath);
    
    if (!g_waterDropObject) {
        DebugLog("ERROR: Failed to load water droplet model!");
        DebugLog("  Water particles will not be visible");
        return false;
    }
    
    DebugLog("Water droplet model loaded successfully (handle: %p)", (void*)g_waterDropObject);
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
    
    /* Get aircraft wingspan 
     * Based on analysis of BetterPushback project (olivierbutler/BetterPusbackMod):
     * - BetterPushback reads wing geometry directly from ACF file, calculating
     *   semispan from wing segment properties or acf/_size_x (in feet)
     * 
     * Dataref units by source (priority order):
     * - sim/aircraft/view/acf_semi_len_m: semispan (half-width) in METERS
     *   -> Need to multiply by 2 to get full wingspan
     * - sim/aircraft/overflow/acf_span: wingspan in FEET
     *   -> Need FEET_TO_METERS conversion
     * - sim/aircraft/parts/acf_wing_span: if exists, may return meters
     * - sim/aircraft/view/acf_wing_span: if exists, may return meters
     * 
     * Strategy: Detect which dataref we found and apply appropriate conversion.
     * Default to 30 meters if not available or value is unreasonable.
     */
    float wingspan = DEFAULT_WINGSPAN_METERS;  /* Default wingspan in meters */
    if (g_drWingspan) {
        float rawValue = XPLMGetDataf(g_drWingspan);
        DebugLog("Raw wingspan dataref value: %.2f", rawValue);
        
        /* Check if we're using acf_semi_len_m (semispan in meters) */
        if (rawValue >= MIN_SEMISPAN_METERS && rawValue <= MAX_SEMISPAN_METERS) {
            /* Likely acf_semi_len_m (semispan in meters), multiply by 2 for full wingspan */
            wingspan = rawValue * 2.0f;
            DebugLog("Interpreted as semispan in meters, wingspan: %.2f m", wingspan);
        } else {
            /* Try feet->meters conversion (acf_span returns full wingspan in feet) */
            float wingspanMeters = rawValue * FEET_TO_METERS;
            DebugLog("Wingspan after feet->meters conversion: %.2f m", wingspanMeters);
            
            /* Sanity check: wingspan should be within valid range */
            if (wingspanMeters >= MIN_WINGSPAN_METERS && wingspanMeters <= MAX_WINGSPAN_METERS) {
                wingspan = wingspanMeters;
            } else if (rawValue >= MIN_WINGSPAN_METERS && rawValue <= MAX_WINGSPAN_METERS) {
                /* Fallback: If converted value is unreasonable, raw value might already be full wingspan in meters */
                DebugLog("Converted value unreasonable, using raw value as meters (fallback)");
                wingspan = rawValue;
            } else {
                DebugLog("Invalid wingspan value (%.2f ft / %.2f m), using default %.0fm", 
                         rawValue, wingspanMeters, DEFAULT_WINGSPAN_METERS);
            }
        }
    } else {
        DebugLog("Wingspan dataref not found, using default %.0fm", DEFAULT_WINGSPAN_METERS);
    }
    
    DebugLog("Aircraft position: (%.2f, %.2f, %.2f)", acX, acY, acZ);
    DebugLog("Aircraft heading: %.1f degrees", acHeading);
    DebugLog("Aircraft wingspan: %.1f meters", wingspan);
    
    /* Get aircraft latitude/longitude for road network loading */
    double acLat = 0.0, acLon = 0.0;
    if (g_drLatitude && g_drLongitude) {
        acLat = XPLMGetDatad(g_drLatitude);
        acLon = XPLMGetDatad(g_drLongitude);
        DebugLog("Aircraft lat/lon: (%.6f, %.6f)", acLat, acLon);
    }
    
    /* Try to load road network from apt.dat */
    bool roadNetworkLoaded = false;
    /* Always try to load road network - lat/lon 0,0 is a valid location (Gulf of Guinea) 
     * The LoadAptDat function will search for nearby airports regardless */
    roadNetworkLoaded = LoadAptDat(acLat, acLon);
    if (roadNetworkLoaded) {
        DebugLog("Road network loaded successfully for airport %s", g_roadNetwork.airportId.c_str());
    } else {
        DebugLog("Road network not available, using direct approach");
    }
    
    /* Calculate truck spacing (wingspan + 40 meters) */
    float truckSpacing = (wingspan / 2.0f) + (TRUCK_EXTRA_SPACING / 2.0f);
    DebugLog("Truck spacing from center: %.1f meters", truckSpacing);
    
    /* Convert heading to radians */
    float headingRad = acHeading * DEG_TO_RAD;
    
    /* Calculate forward vector in X-Plane coordinate system:
     * - X-Plane uses: +X = East, -Z = North
     * - Heading (psi): 0° = North, 90° = East, 180° = South, 270° = West
     * - For heading θ: forward = (sin(θ), -cos(θ)) in (X, Z)
     *   - heading 0°: forward = (0, -1) = North/-Z ✓
     *   - heading 90°: forward = (1, 0) = East/+X ✓
     */
    float forwardX = sinf(headingRad);
    float forwardZ = -cosf(headingRad);
    DebugLog("Forward vector: (%.4f, %.4f)", forwardX, forwardZ);
    
    /* Calculate right vector (perpendicular to forward, 90° clockwise):
     * - For heading θ: right = (cos(θ), sin(θ)) in (X, Z)
     *   - heading 0°: right = (1, 0) = East/+X ✓
     *   - heading 90°: right = (0, 1) = South/+Z ✓
     */
    float rightX = cosf(headingRad);
    float rightZ = sinf(headingRad);
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
    
    /* Plan routes for trucks if road network is available */
    if (roadNetworkLoaded) {
        /* Find spawn points on roads near the default start positions */
        size_t leftSpawnNode = FindNearestNode(g_leftTruck.x, g_leftTruck.z, true);
        size_t rightSpawnNode = FindNearestNode(g_rightTruck.x, g_rightTruck.z, true);
        
        if (leftSpawnNode != SIZE_MAX) {
            const RoadNode& node = g_roadNetwork.nodes[leftSpawnNode];
            g_leftTruck.x = node.x;
            g_leftTruck.z = node.z;
            g_leftTruck.y = GetTerrainHeight(static_cast<float>(node.x), static_cast<float>(node.z));
            DebugLog("Left truck spawning on road node '%s' at (%.2f, %.2f)", 
                     node.name.c_str(), node.x, node.z);
        }
        
        if (rightSpawnNode != SIZE_MAX) {
            const RoadNode& node = g_roadNetwork.nodes[rightSpawnNode];
            g_rightTruck.x = node.x;
            g_rightTruck.z = node.z;
            g_rightTruck.y = GetTerrainHeight(static_cast<float>(node.x), static_cast<float>(node.z));
            DebugLog("Right truck spawning on road node '%s' at (%.2f, %.2f)", 
                     node.name.c_str(), node.x, node.z);
        }
        
        /* Plan routes from spawn to target */
        g_leftTruck.route = PlanRouteToTarget(g_leftTruck.x, g_leftTruck.z, 
                                               g_leftTruck.targetX, g_leftTruck.targetZ,
                                               g_leftTruck.targetHeading);
        g_leftTruck.useRoadNetwork = g_leftTruck.route.isValid;
        
        g_rightTruck.route = PlanRouteToTarget(g_rightTruck.x, g_rightTruck.z,
                                                g_rightTruck.targetX, g_rightTruck.targetZ,
                                                g_rightTruck.targetHeading);
        g_rightTruck.useRoadNetwork = g_rightTruck.route.isValid;
        
        DebugLog("Left truck route: %s (%zu waypoints)", 
                 g_leftTruck.route.isValid ? "VALID" : "INVALID",
                 g_leftTruck.route.waypoints.size());
        DebugLog("Right truck route: %s (%zu waypoints)",
                 g_rightTruck.route.isValid ? "VALID" : "INVALID",
                 g_rightTruck.route.waypoints.size());
        
        /* Set initial heading based on first waypoint if route is valid */
        if (g_leftTruck.route.isValid && g_leftTruck.route.waypoints.size() > 1) {
            g_leftTruck.heading = g_leftTruck.route.waypoints[0].targetHeading;
        }
        if (g_rightTruck.route.isValid && g_rightTruck.route.waypoints.size() > 1) {
            g_rightTruck.heading = g_rightTruck.route.waypoints[0].targetHeading;
        }
    } else {
        DebugLog("No road network - trucks will use direct approach");
    }
    
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
        
        /* Get aircraft heading to calculate leave direction */
        float acHeading = g_drHeading ? XPLMGetDataf(g_drHeading) : 0.0f;
        
        /* Setup trucks to turn before leaving
         * Trucks should turn to face away from the aircraft (back the way they came)
         * Left truck turns to face back (opposite of its position heading toward center)
         * Right truck turns to face back (opposite of its position heading toward center)
         * 
         * The trucks came from behind the aircraft, so they should leave by driving 
         * back towards where they came from. The leave heading is the original approach
         * direction (which was aircraft heading + 180, now we add another 180 to go back)
         * This equals the aircraft heading direction (pointing where the aircraft is going)
         */
        
        /* Left truck should turn to leave away from center - turn left */
        g_leftTruck.isTurningBeforeLeave = true;
        g_leftTruck.leaveHeading = NormalizeAngle360(acHeading - 45.0f); /* Turn left and leave */
        g_leftTruck.targetSpeed = 0.0f; /* Start stopped, will accelerate */
        
        /* Right truck should turn to leave away from center - turn right */
        g_rightTruck.isTurningBeforeLeave = true;
        g_rightTruck.leaveHeading = NormalizeAngle360(acHeading + 45.0f); /* Turn right and leave */
        g_rightTruck.targetSpeed = 0.0f;
        
        DebugLog("Left truck will turn to heading %.1f then leave", g_leftTruck.leaveHeading);
        DebugLog("Right truck will turn to heading %.1f then leave", g_rightTruck.leaveHeading);
        
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
    /* Initialize wheel and steering control properties */
    truck.frontSteeringAngle = 0.0f;  /* Front axles steering (negative = left turn) */
    truck.rearSteeringAngle = 0.0f;   /* Rear axle steering (opposite sign to front for counter-steering) */
    truck.wheelRotationAngle = 0.0f;  /* Wheel rotation angle 0-360° */
    truck.cannonPitch = DEFAULT_CANNON_PITCH;  /* Default pitch for water arc */
    truck.cannonYaw = 0.0f;     /* Default facing forward relative to truck */
    truck.speed = 0.0f;
    truck.targetSpeed = 0.0f;   /* Target speed for smooth acceleration */
    truck.isTurningBeforeLeave = false; /* Not turning yet */
    truck.leaveHeading = 0.0f;  /* Will be set when leaving */
    /* Initialize route planning */
    truck.route.waypoints.clear();
    truck.route.currentWaypointIndex = 0;
    truck.route.isValid = false;
    truck.route.isCompleted = false;
    truck.useRoadNetwork = false;
}

/*
 * CleanupTruck - Clean up a fire truck
 */
static void CleanupTruck(FireTruck& truck) {
    if (truck.instance) {
        XPLMDestroyInstance(truck.instance);
        truck.instance = nullptr;
    }
    /* Clean up all particle instances */
    for (auto& particle : truck.particles) {
        if (particle.instance) {
            XPLMDestroyInstance(particle.instance);
            particle.instance = nullptr;
        }
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
            /* Truck is stationary - smoothly decelerate to stop */
            truck.targetSpeed = 0.0f;
            truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
            truck.frontSteeringAngle = 0.0f;
            truck.rearSteeringAngle = 0.0f;
            /* Wheel rotation angle stays at current value when stopped */
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
        
        /* Fallback: Direct approach to target (original behavior) */
        /* Calculate distance and direction to target */
        float dx = truck.targetX - static_cast<float>(truck.x);
        float dz = truck.targetZ - static_cast<float>(truck.z);
        float distance = sqrtf(dx * dx + dz * dz);
        
        if (distance > 2.0f) {
            /* Calculate desired heading to target
             * Using X-Plane coordinate system:
             * - heading 0° = North = (0, -1) in (X, Z)
             * - heading 90° = East = (1, 0) in (X, Z)
             * - For heading θ: forward = (sin(θ), -cos(θ))
             * - To get heading from direction: θ = atan2(dirX, -dirZ)
             */
            float dirX = dx / distance;
            float dirZ = dz / distance;
            float desiredHeading = atan2f(dirX, -dirZ) * RAD_TO_DEG;
            
            /* Calculate heading difference */
            float headingDiff = desiredHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            /* STEP 1: Set front wheel steering angle based on heading difference
             * The front wheel angle is the primary control (within ±45 degrees)
             * - Negative angle: turning left
             * - Positive angle: turning right
             */
            truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
            
            /* STEP 2: Calculate rear wheel angle from front wheel angle
             * Rear axle uses counter-steering (opposite direction)
             */
            truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
            
            /* Set target speed based on distance - slow down when approaching target */
            if (distance < TRUCK_SLOWDOWN_DISTANCE) {
                /* Gradually reduce speed as we approach the target */
                truck.targetSpeed = TRUCK_APPROACH_SPEED * (distance / TRUCK_SLOWDOWN_DISTANCE);
                if (truck.targetSpeed < TRUCK_TURN_IN_PLACE_SPEED) {
                    truck.targetSpeed = TRUCK_TURN_IN_PLACE_SPEED;
                }
            } else {
                truck.targetSpeed = TRUCK_APPROACH_SPEED;
            }
            
            /* Smoothly transition to target speed */
            truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
            
            /* STEP 3: Calculate turning rate from front and rear wheel angles using Ackermann method */
            float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
            
            /* STEP 4: Update heading based on turning rate */
            truck.heading += turningRate * dt;
            /* Normalize heading to 0-360 range */
            while (truck.heading >= 360.0f) truck.heading -= 360.0f;
            while (truck.heading < 0.0f) truck.heading += 360.0f;
            
            /* Move truck in the direction it's facing
             * Using X-Plane coordinate system:
             * - For heading θ: forward direction = (sin(θ), -cos(θ)) in (X, Z)
             */
            float headingRad = truck.heading * DEG_TO_RAD;
            float moveDistance = truck.speed * dt;
            if (moveDistance > distance) moveDistance = distance;
            
            /* Update position based on current heading (not target direction) */
            truck.x += sinf(headingRad) * moveDistance;
            truck.z += -cosf(headingRad) * moveDistance;
            truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
            
            /* Update wheel rotation angle based on distance moved */
            UpdateWheelRotationAngle(truck, moveDistance);
            
            allPositioned = false;
        } else {
            /* Reached position, now turn to face target heading */
            float headingDiff = truck.targetHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            if (fabsf(headingDiff) > 1.0f) {
                /* STEP 1: Set front wheel steering angle for turning in place
                 * Use a proportional approach with some minimum angle
                 */
                truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
                
                /* STEP 2: Calculate rear wheel angle */
                truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
                
                /* For turning in place, use a small speed to calculate turn rate */
                truck.targetSpeed = TRUCK_TURN_IN_PLACE_SPEED;
                truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
                
                /* STEP 3: Calculate and apply turning rate using Ackermann method */
                float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
                truck.heading += turningRate * dt;
                while (truck.heading >= 360.0f) truck.heading -= 360.0f;
                while (truck.heading < 0.0f) truck.heading += 360.0f;
                
                allPositioned = false;
            } else {
                truck.heading = truck.targetHeading;
                truck.positioned = true;
                truck.targetSpeed = 0.0f;
                truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
                truck.frontSteeringAngle = 0.0f;
                truck.rearSteeringAngle = 0.0f;
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
        /* Phase 1: Turn before leaving */
        if (truck.isTurningBeforeLeave) {
            /* Calculate heading difference to leave heading */
            float headingDiff = truck.leaveHeading - truck.heading;
            while (headingDiff > 180.0f) headingDiff -= 360.0f;
            while (headingDiff < -180.0f) headingDiff += 360.0f;
            
            if (fabsf(headingDiff) > HEADING_TOLERANCE_DEG) {
                /* Still turning - set steering angles */
                truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
                truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
                
                /* Turn slowly */
                truck.targetSpeed = TRUCK_TURN_IN_PLACE_SPEED;
                truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
                
                /* Calculate and apply turning rate */
                float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
                truck.heading += turningRate * dt;
                while (truck.heading >= 360.0f) truck.heading -= 360.0f;
                while (truck.heading < 0.0f) truck.heading += 360.0f;
                
                /* Update wheel rotation */
                float moveDistance = truck.speed * dt;
                UpdateWheelRotationAngle(truck, moveDistance);
            } else {
                /* Finished turning, start driving away */
                truck.heading = truck.leaveHeading;
                truck.isTurningBeforeLeave = false;
                truck.frontSteeringAngle = 0.0f;
                truck.rearSteeringAngle = 0.0f;
                DebugLog("Truck finished turning, now driving away at heading %.1f", truck.heading);
            }
        } else {
            /* Phase 2: Drive straight when leaving */
            truck.frontSteeringAngle = 0.0f;
            truck.rearSteeringAngle = 0.0f;
            
            /* Set target speed for leaving (2/3 of approach speed) */
            truck.targetSpeed = TRUCK_APPROACH_SPEED * TRUCK_LEAVING_SPEED_MULT;
            truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
            
            /* Move forward in the direction the truck is facing
             * Using X-Plane coordinate system:
             * - For heading θ: forward direction = (sin(θ), -cos(θ)) in (X, Z)
             */
            float headingRad = truck.heading * DEG_TO_RAD;
            float moveDistance = truck.speed * dt;
            
            /* Update wheel rotation angle based on distance moved */
            UpdateWheelRotationAngle(truck, moveDistance);
            
            truck.x += sinf(headingRad) * moveDistance;
            truck.z += -cosf(headingRad) * moveDistance;
            truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
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
        
        /* Check if far enough away from aircraft */
        double acX = g_drLocalX ? XPLMGetDatad(g_drLocalX) : 0.0;
        double acZ = g_drLocalZ ? XPLMGetDatad(g_drLocalZ) : 0.0;
        float distDx = static_cast<float>(truck.x - acX);
        float distDz = static_cast<float>(truck.z - acZ);
        float distFromAircraft = sqrtf(distDx * distDx + distDz * distDz);
        
        return distFromAircraft > TRUCK_LEAVING_DISTANCE;
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
                if (dist < TRUCK_POSITIONING_THRESHOLD) {
                    DebugLog("STATE TRANSITION: TRUCKS_APPROACHING -> TRUCKS_POSITIONING");
                    DebugLog("Trucks are within %.0fm of target, beginning positioning phase", TRUCK_POSITIONING_THRESHOLD);
                    g_state = STATE_TRUCKS_POSITIONING;
                }
            }
            break;
        }
        
        case STATE_TRUCKS_LEAVING: {
            bool leftGone = updateTruckLeaving(g_leftTruck);
            bool rightGone = updateTruckLeaving(g_rightTruck);
            
            /* Destroy particle instances before clearing (prevent memory leak) */
            for (auto& particle : g_leftTruck.particles) {
                if (particle.instance) {
                    XPLMDestroyInstance(particle.instance);
                    particle.instance = nullptr;
                }
            }
            for (auto& particle : g_rightTruck.particles) {
                if (particle.instance) {
                    XPLMDestroyInstance(particle.instance);
                    particle.instance = nullptr;
                }
            }
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
            
            /* Apply air drag (like PSS DRAG_CURVE) */
            float speed = sqrtf(particle.vx * particle.vx + particle.vy * particle.vy + particle.vz * particle.vz);
            if (speed > 0.01f) {
                float dragFactor = 1.0f - PARTICLE_DRAG * dt;
                if (dragFactor < 0.0f) dragFactor = 0.0f;
                particle.vx *= dragFactor;
                particle.vy *= dragFactor;
                particle.vz *= dragFactor;
            }
            
            /* Apply turbulence (like PSS TURBULENCE) - using fast mt19937 RNG */
            float turbScale = PARTICLE_TURBULENCE * dt * 60.0f; /* Scale for frame rate independence */
            particle.vx += g_randomDist(g_rng) * turbScale;
            particle.vy += g_randomDist(g_rng) * turbScale * 0.5f;
            particle.vz += g_randomDist(g_rng) * turbScale;
            
            /* Update position */
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;
            particle.z += particle.vz * dt;
            
            /* Update instance position if it exists */
            if (particle.instance) {
                XPLMDrawInfo_t drawInfo;
                drawInfo.structSize = sizeof(XPLMDrawInfo_t);
                drawInfo.x = particle.x;
                drawInfo.y = particle.y;
                drawInfo.z = particle.z;
                drawInfo.pitch = 0.0f;
                drawInfo.heading = 0.0f;
                drawInfo.roll = 0.0f;
                XPLMInstanceSetPosition(particle.instance, &drawInfo, nullptr);
            }
            
            /* Update lifetime */
            particle.lifetime -= dt;
            
            /* Check if particle should deactivate */
            if (particle.lifetime <= 0.0f || particle.y < GetTerrainHeight(particle.x, particle.z)) {
                particle.active = false;
                /* Destroy the instance when particle becomes inactive */
                if (particle.instance) {
                    XPLMDestroyInstance(particle.instance);
                    particle.instance = nullptr;
                }
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
    
    /* Calculate nozzle world position 
     * nozzleOffsetX: lateral offset (positive = right side of truck)
     * nozzleOffsetZ: forward offset (positive = front of truck)
     * 
     * Using X-Plane coordinate system with rotation:
     * - right = (cos(θ), sin(θ)) in (X, Z)
     * - forward = (sin(θ), -cos(θ)) in (X, Z)
     */
    float headingRad = truck.heading * DEG_TO_RAD;
    float cosH = cosf(headingRad);
    float sinH = sinf(headingRad);
    
    float nozzleX = static_cast<float>(truck.x) + truck.nozzleOffsetX * cosH + truck.nozzleOffsetZ * sinH;
    float nozzleY = static_cast<float>(truck.y) + truck.nozzleOffsetY;
    float nozzleZ = static_cast<float>(truck.z) + truck.nozzleOffsetX * sinH + truck.nozzleOffsetZ * (-cosH);
    
    /* Calculate water jet direction based on cannon pitch and yaw
     * The cannon aims based on truck heading + cannonYaw for horizontal direction
     * and cannonPitch for vertical angle
     */
    float cannonHeadingRad = (truck.heading + truck.cannonYaw) * DEG_TO_RAD;
    float cannonPitchRad = truck.cannonPitch * DEG_TO_RAD;
    
    /* Initial speed for water jet (based on PSS INITIAL_SPEED ~18-25 m/s) */
    float baseSpeed = 22.0f + g_randomDist(g_rng) * 6.0f;
    
    /* Calculate velocity components from cannon angles
     * Pitch: 0 = horizontal, 90 = straight up
     * Heading: direction in horizontal plane
     */
    float horizontalSpeed = baseSpeed * cosf(cannonPitchRad);
    float verticalSpeed = baseSpeed * sinf(cannonPitchRad);
    
    /* Add spread like PSS INITIAL_HEADING variation (±10 degrees heading, ±5 degrees pitch)
     * g_randomDist returns [-0.5, 0.5], so multiply by 20 for ±10 range and 10 for ±5 range
     */
    float headingSpread = g_randomDist(g_rng) * 20.0f * DEG_TO_RAD;
    float pitchSpread = g_randomDist(g_rng) * 10.0f * DEG_TO_RAD;
    
    float finalHeading = cannonHeadingRad + headingSpread;
    float finalPitch = cannonPitchRad + pitchSpread;
    
    /* Recalculate velocity with spread */
    horizontalSpeed = baseSpeed * cosf(finalPitch);
    verticalSpeed = baseSpeed * sinf(finalPitch);
    
    /* Convert to X-Plane coordinate system
     * Heading 0 = North = -Z direction
     * Heading 90 = East = +X direction
     */
    float vx = horizontalSpeed * sinf(finalHeading);
    float vz = -horizontalSpeed * cosf(finalHeading);
    float vy = verticalSpeed;
    
    /* Create particle */
    WaterParticle particle;
    particle.x = nozzleX;
    particle.y = nozzleY;
    particle.z = nozzleZ;
    particle.vx = vx;
    particle.vy = vy;
    particle.vz = vz;
    /* Add slight lifetime variation for more natural spray */
    particle.lifetime = PARTICLE_LIFETIME + g_randomDist(g_rng) * 1.0f;
    particle.maxLifetime = particle.lifetime;
    particle.active = true;
    particle.instance = nullptr;
    
    /* Create instance for this particle if water drop model is loaded */
    if (g_waterDropObject) {
        particle.instance = XPLMCreateInstance(g_waterDropObject, g_noDataRefs);
        
        if (particle.instance) {
            /* Set initial position */
            XPLMDrawInfo_t drawInfo;
            drawInfo.structSize = sizeof(XPLMDrawInfo_t);
            drawInfo.x = particle.x;
            drawInfo.y = particle.y;
            drawInfo.z = particle.z;
            drawInfo.pitch = 0.0f;
            drawInfo.heading = 0.0f;
            drawInfo.roll = 0.0f;
            XPLMInstanceSetPosition(particle.instance, &drawInfo, nullptr);
        } else {
            /* Log instance creation failure (only once to avoid log spam) */
            static bool instanceFailureLogged = false;
            if (!instanceFailureLogged) {
                DebugLog("WARNING: Failed to create water particle instance");
                DebugLog("  Water droplet model handle: %p", (void*)g_waterDropObject);
                instanceFailureLogged = true;
            }
        }
    }
    
    truck.particles.push_back(particle);
    g_particlesEmittedTotal++;
    
    /* Log first particle emission for debugging */
    if (g_particlesEmittedTotal == 1) {
        DebugLog("First particle emitted:");
        DebugLog("  Nozzle pos: (%.2f, %.2f, %.2f)", nozzleX, nozzleY, nozzleZ);
        DebugLog("  Cannon angles: pitch=%.1f yaw=%.1f", truck.cannonPitch, truck.cannonYaw);
        DebugLog("  Velocity: (%.2f, %.2f, %.2f)", vx, vy, vz);
        DebugLog("  Speed: %.2f m/s", baseSpeed);
    }
}

/*
 * RegisterDrawCallback - Register the draw callback for water particles
 * Note: With instance-based rendering, this callback is mainly for debugging.
 * The actual rendering is handled by X-Plane's instance system.
 */
static void RegisterDrawCallback() {
    if (!g_drawCallbackRegistered) {
        int result = XPLMRegisterDrawCallback(DrawWaterParticles, WATER_DRAWING_PHASE, 0, nullptr);
        g_drawCallbackRegistered = true;
        g_drawCallbackCallCount = 0; /* Reset counter */
        DebugLog("Draw callback registered (result=%d)", result);
        DebugLog("  Drawing phase: xplm_Phase_Modern3D (%d)", WATER_DRAWING_PHASE);
        DebugLog("  Water particles now use instance-based rendering");
        DebugLog("  Water droplet model loaded: %s", g_waterDropObject ? "YES" : "NO");
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
 * Note: With instance-based rendering, the actual particle rendering is handled
 * by X-Plane's instance system (XPLMCreateInstance/XPLMInstanceSetPosition).
 * This callback is kept for debugging and statistics purposes.
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
        DebugLog("  Using instance-based rendering for water particles");
    }
    
    if (g_state != STATE_WATER_SPRAYING) {
        if (g_drawCallbackCallCount <= 5) {
            DebugLog("DrawWaterParticles: State is not WATER_SPRAYING (state=%s)", GetStateName(g_state));
        }
        return 1;
    }
    
    /* Count active particles for debugging */
    int leftActiveCount = 0;
    int rightActiveCount = 0;
    int leftInstanceCount = 0;
    int rightInstanceCount = 0;
    
    /* Count particles and instances */
    for (const auto& particle : g_leftTruck.particles) {
        if (particle.active) {
            leftActiveCount++;
            if (particle.instance) leftInstanceCount++;
        }
    }
    for (const auto& particle : g_rightTruck.particles) {
        if (particle.active) {
            rightActiveCount++;
            if (particle.instance) rightInstanceCount++;
        }
    }
    
    /* Periodic logging of particle counts */
    if (g_drawCallbackCallCount % 100 == 0) {
        DebugLog("DrawWaterParticles stats (call #%d):", g_drawCallbackCallCount);
        DebugLog("  Left truck: %d particles, %d instances", leftActiveCount, leftInstanceCount);
        DebugLog("  Right truck: %d particles, %d instances", rightActiveCount, rightInstanceCount);
        DebugLog("  Total particles emitted: %d", g_particlesEmittedTotal);
    }
    
    return 1;
}

/*
 * ================================================================================
 * Road Network and Path Planning Implementation
 * ================================================================================
 * 
 * This section implements airport ground route parsing from apt.dat files,
 * A* pathfinding for route planning, and Bezier curve smoothing for natural
 * truck movement along planned paths.
 */

/*
 * LatLonToLocal - Convert latitude/longitude to local OpenGL coordinates
 * Uses simple equirectangular projection centered on reference point
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param refLat Reference latitude (center point)
 * @param refLon Reference longitude (center point)
 * @param x Output X coordinate (meters, East positive)
 * @param z Output Z coordinate (meters, South positive in X-Plane)
 */
static void LatLonToLocal(double lat, double lon, double refLat, double refLon, double& x, double& z) {
    double latRad = refLat * DEG_TO_RAD;
    /* X is East-West distance */
    x = (lon - refLon) * DEG_TO_RAD * EARTH_RADIUS_METERS * cos(latRad);
    /* Z in X-Plane is negative North, positive South */
    z = -(lat - refLat) * DEG_TO_RAD * EARTH_RADIUS_METERS;
}

/*
 * LocalToLatLon - Convert local OpenGL coordinates to latitude/longitude
 * Inverse of LatLonToLocal
 */
static void LocalToLatLon(double x, double z, double refLat, double refLon, double& lat, double& lon) {
    double latRad = refLat * DEG_TO_RAD;
    lon = refLon + x / (EARTH_RADIUS_METERS * cos(latRad)) * RAD_TO_DEG;
    lat = refLat - z / EARTH_RADIUS_METERS * RAD_TO_DEG;
}

/*
 * ParseAptDatLine - Parse a single line from apt.dat
 * @param line The line to parse
 * @param tokens Output vector of space-separated tokens
 */
static void ParseAptDatLine(const std::string& line, std::vector<std::string>& tokens) {
    tokens.clear();
    std::istringstream iss(line);
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
}

/*
 * LoadAptDat - Load airport ground routes from apt.dat
 * Searches for apt.dat in X-Plane's Resources/default scenery folder
 * and loads the ground route network for the nearest airport
 * 
 * @param acLat Aircraft latitude in degrees
 * @param acLon Aircraft longitude in degrees
 * @return true if airport found and loaded successfully
 */
static bool LoadAptDat(double acLat, double acLon) {
    DebugLog("LoadAptDat: Searching for airport near (%.6f, %.6f)", acLat, acLon);
    
    /* Reset road network */
    g_roadNetwork.nodes.clear();
    g_roadNetwork.edges.clear();
    g_roadNetwork.nodeNameToIndex.clear();
    g_roadNetwork.isLoaded = false;
    
    /* Get X-Plane system path */
    char systemPath[512];
    XPLMGetSystemPath(systemPath);
    
    /* Build path to apt.dat - try several common locations */
    std::vector<std::string> aptDatPaths;
    
    /* Standard apt.dat locations for X-Plane 11/12 */
    aptDatPaths.push_back(std::string(systemPath) + "Resources/default scenery/default apt data/Earth nav data/apt.dat");
    aptDatPaths.push_back(std::string(systemPath) + "Custom Scenery/Global Airports/Earth nav data/apt.dat");
    aptDatPaths.push_back(std::string(systemPath) + "Resources/default data/apt.dat");
    
    std::ifstream aptFile;
    std::string usedPath;
    
    for (const auto& path : aptDatPaths) {
        aptFile.open(path);
        if (aptFile.is_open()) {
            usedPath = path;
            DebugLog("LoadAptDat: Opened apt.dat at %s", path.c_str());
            break;
        }
    }
    
    if (!aptFile.is_open()) {
        DebugLog("LoadAptDat: Could not open any apt.dat file");
        return false;
    }
    
    /* Variables for parsing */
    std::string line;
    std::vector<std::string> tokens;
    bool foundNearbyAirport = false;
    bool inAirport = false;
    bool inGroundNetwork = false;
    double airportLat = 0.0, airportLon = 0.0;
    std::string currentAirportId;
    double bestDistance = ROAD_SEARCH_RADIUS;
    std::string bestAirportId;
    double bestAirportLat = 0.0, bestAirportLon = 0.0;
    
    /* Temporary storage for current airport's ground network */
    std::vector<RoadNode> tempNodes;
    std::vector<RoadEdge> tempEdges;
    std::unordered_map<std::string, size_t> tempNodeNameToIndex;
    
    /* First pass: find the nearest airport */
    while (std::getline(aptFile, line)) {
        ParseAptDatLine(line, tokens);
        if (tokens.empty()) continue;
        
        int rowCode = 0;
        try {
            rowCode = std::stoi(tokens[0]);
        } catch (...) {
            continue;
        }
        
        /* Airport header (1 = land airport, 16 = seaplane base, 17 = heliport) */
        if (rowCode == 1 || rowCode == 16 || rowCode == 17) {
            /* Check if we were in an airport with ground network */
            if (inAirport && inGroundNetwork && !tempNodes.empty()) {
                /* Calculate distance to this airport */
                double dx, dz;
                LatLonToLocal(acLat, acLon, airportLat, airportLon, dx, dz);
                double dist = sqrt(dx * dx + dz * dz);
                if (dist < bestDistance) {
                    bestDistance = dist;
                    bestAirportId = currentAirportId;
                    bestAirportLat = airportLat;
                    bestAirportLon = airportLon;
                    /* Save this airport's ground network */
                    g_roadNetwork.nodes = tempNodes;
                    g_roadNetwork.edges = tempEdges;
                    g_roadNetwork.nodeNameToIndex = tempNodeNameToIndex;
                    foundNearbyAirport = true;
                }
            }
            
            /* Reset for new airport */
            inAirport = true;
            inGroundNetwork = false;
            tempNodes.clear();
            tempEdges.clear();
            tempNodeNameToIndex.clear();
            
            if (tokens.size() >= 5) {
                currentAirportId = tokens[4];
                /* Use first runway/taxiway coordinates as reference */
                airportLat = acLat;  /* Will be updated when we find a node */
                airportLon = acLon;
            }
        }
        /* Taxi routing network header */
        else if (rowCode == 1200) {
            inGroundNetwork = true;
        }
        /* Taxi routing node (1201) */
        else if (rowCode == 1201 && inGroundNetwork && tokens.size() >= 5) {
            RoadNode node;
            try {
                node.lat = std::stod(tokens[1]);
                node.lon = std::stod(tokens[2]);
                /* Use first node as airport reference point if not set */
                if (tempNodes.empty()) {
                    airportLat = node.lat;
                    airportLon = node.lon;
                }
            } catch (...) {
                continue;
            }
            node.nodeType = tokens[3];
            node.name = tokens[4];
            /* For nodes with multi-word names, concatenate remaining tokens */
            for (size_t i = 5; i < tokens.size(); ++i) {
                node.name += "_" + tokens[i];
            }
            
            tempNodeNameToIndex[node.name] = tempNodes.size();
            tempNodes.push_back(node);
        }
        /* Taxi routing edge (1202) - basic taxiway connection */
        else if (rowCode == 1202 && inGroundNetwork && tokens.size() >= 4) {
            RoadEdge edge;
            std::string node1Name = tokens[1];
            std::string node2Name = tokens[2];
            
            auto it1 = tempNodeNameToIndex.find(node1Name);
            auto it2 = tempNodeNameToIndex.find(node2Name);
            if (it1 != tempNodeNameToIndex.end() && it2 != tempNodeNameToIndex.end()) {
                edge.node1Idx = it1->second;
                edge.node2Idx = it2->second;
                edge.isOneWay = (tokens[3] == "oneway");
                edge.isFireTruckRoute = true;  /* Assume taxiways can be used by fire trucks */
                edge.surfaceType = (tokens.size() > 4) ? tokens[4] : "taxiway";
                edge.length = 0.0f;  /* Will be calculated later */
                
                tempNodes[edge.node1Idx].connectedEdges.push_back(tempEdges.size());
                if (!edge.isOneWay) {
                    tempNodes[edge.node2Idx].connectedEdges.push_back(tempEdges.size());
                }
                tempEdges.push_back(edge);
            }
        }
        /* Ground truck route edge (1206) - specific vehicle types */
        else if (rowCode == 1206 && inGroundNetwork && tokens.size() >= 4) {
            RoadEdge edge;
            std::string node1Name = tokens[1];
            std::string node2Name = tokens[2];
            std::string direction = (tokens.size() > 3) ? tokens[3] : "twoway";
            std::string truckTypes = (tokens.size() > 4) ? tokens[4] : "";
            
            auto it1 = tempNodeNameToIndex.find(node1Name);
            auto it2 = tempNodeNameToIndex.find(node2Name);
            if (it1 != tempNodeNameToIndex.end() && it2 != tempNodeNameToIndex.end()) {
                edge.node1Idx = it1->second;
                edge.node2Idx = it2->second;
                edge.isOneWay = (direction == "oneway");
                /* Check if fire_truck is allowed - match exact "fire_truck" or empty (all trucks) */
                edge.isFireTruckRoute = (truckTypes.find("fire_truck") != std::string::npos) ||
                                        truckTypes.empty();  /* Empty means all trucks allowed */
                edge.surfaceType = "service_road";
                edge.length = 0.0f;
                
                tempNodes[edge.node1Idx].connectedEdges.push_back(tempEdges.size());
                if (!edge.isOneWay) {
                    tempNodes[edge.node2Idx].connectedEdges.push_back(tempEdges.size());
                }
                tempEdges.push_back(edge);
            }
        }
    }
    
    /* Check final airport if any */
    if (inAirport && inGroundNetwork && !tempNodes.empty()) {
        double dx, dz;
        LatLonToLocal(acLat, acLon, airportLat, airportLon, dx, dz);
        double dist = sqrt(dx * dx + dz * dz);
        if (dist < bestDistance) {
            bestDistance = dist;
            bestAirportId = currentAirportId;
            bestAirportLat = airportLat;
            bestAirportLon = airportLon;
            g_roadNetwork.nodes = tempNodes;
            g_roadNetwork.edges = tempEdges;
            g_roadNetwork.nodeNameToIndex = tempNodeNameToIndex;
            foundNearbyAirport = true;
        }
    }
    
    aptFile.close();
    
    if (!foundNearbyAirport) {
        DebugLog("LoadAptDat: No nearby airport with ground routes found");
        return false;
    }
    
    /* Set up the road network */
    g_roadNetwork.airportId = bestAirportId;
    g_roadNetwork.refLat = bestAirportLat;
    g_roadNetwork.refLon = bestAirportLon;
    
    /* Convert all node coordinates to local OpenGL coordinates using X-Plane's conversion */
    for (auto& node : g_roadNetwork.nodes) {
        double outY;
        XPLMWorldToLocal(node.lat, node.lon, 0.0, &node.x, &outY, &node.z);
    }
    
    /* Calculate edge lengths */
    for (auto& edge : g_roadNetwork.edges) {
        const RoadNode& n1 = g_roadNetwork.nodes[edge.node1Idx];
        const RoadNode& n2 = g_roadNetwork.nodes[edge.node2Idx];
        float dx = static_cast<float>(n2.x - n1.x);
        float dz = static_cast<float>(n2.z - n1.z);
        edge.length = sqrtf(dx * dx + dz * dz);
    }
    
    g_roadNetwork.isLoaded = true;
    
    DebugLog("LoadAptDat: Loaded airport %s with %zu nodes and %zu edges",
             bestAirportId.c_str(), g_roadNetwork.nodes.size(), g_roadNetwork.edges.size());
    
    return true;
}

/*
 * FindNearestNode - Find the nearest road network node to a given position
 * @param x Local X coordinate
 * @param z Local Z coordinate
 * @param firetruckRoutesOnly If true, only consider nodes connected to fire truck routes
 * @return Index of nearest node, or SIZE_MAX if not found
 */
static size_t FindNearestNode(double x, double z, bool firetruckRoutesOnly) {
    if (!g_roadNetwork.isLoaded || g_roadNetwork.nodes.empty()) {
        return SIZE_MAX;
    }
    
    size_t bestIndex = SIZE_MAX;
    float bestDist = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < g_roadNetwork.nodes.size(); ++i) {
        const RoadNode& node = g_roadNetwork.nodes[i];
        
        /* Check if node is connected to any fire truck routes */
        if (firetruckRoutesOnly) {
            bool hasFiretruckRoute = false;
            for (size_t edgeIdx : node.connectedEdges) {
                if (g_roadNetwork.edges[edgeIdx].isFireTruckRoute) {
                    hasFiretruckRoute = true;
                    break;
                }
            }
            if (!hasFiretruckRoute) continue;
        }
        
        float dx = static_cast<float>(node.x - x);
        float dz = static_cast<float>(node.z - z);
        float dist = sqrtf(dx * dx + dz * dz);
        
        if (dist < bestDist) {
            bestDist = dist;
            bestIndex = i;
        }
    }
    
    return bestIndex;
}

/*
 * FindPath - A* pathfinding between two nodes
 * @param startNode Starting node index
 * @param goalNode Goal node index
 * @param path Output vector of node indices forming the path
 * @return true if path found
 */
static bool FindPath(size_t startNode, size_t goalNode, std::vector<size_t>& path) {
    path.clear();
    
    if (!g_roadNetwork.isLoaded || startNode >= g_roadNetwork.nodes.size() ||
        goalNode >= g_roadNetwork.nodes.size()) {
        return false;
    }
    
    const RoadNode& goalNodeRef = g_roadNetwork.nodes[goalNode];
    
    /* A* algorithm */
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
    std::unordered_map<size_t, float> gScores;
    std::unordered_map<size_t, size_t> cameFrom;
    std::unordered_map<size_t, bool> closedSet;
    
    /* Initialize start node */
    AStarNode startANode;
    startANode.nodeIndex = startNode;
    startANode.gScore = 0.0f;
    const RoadNode& startNodeRef = g_roadNetwork.nodes[startNode];
    float heuristic = sqrtf(static_cast<float>((goalNodeRef.x - startNodeRef.x) * (goalNodeRef.x - startNodeRef.x) +
                                                (goalNodeRef.z - startNodeRef.z) * (goalNodeRef.z - startNodeRef.z)));
    startANode.fScore = heuristic;
    startANode.parentIndex = SIZE_MAX;
    
    openSet.push(startANode);
    gScores[startNode] = 0.0f;
    
    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();
        
        if (current.nodeIndex == goalNode) {
            /* Reconstruct path - walk from goal back to start */
            size_t node = goalNode;
            while (node != startNode && cameFrom.find(node) != cameFrom.end()) {
                path.push_back(node);
                node = cameFrom[node];
            }
            path.push_back(startNode);
            std::reverse(path.begin(), path.end());
            
            DebugLog("FindPath: Found path with %zu nodes", path.size());
            return true;
        }
        
        if (closedSet.find(current.nodeIndex) != closedSet.end()) {
            continue;
        }
        closedSet[current.nodeIndex] = true;
        
        const RoadNode& currentNode = g_roadNetwork.nodes[current.nodeIndex];
        
        /* Explore neighbors */
        for (size_t edgeIdx : currentNode.connectedEdges) {
            const RoadEdge& edge = g_roadNetwork.edges[edgeIdx];
            
            /* Skip non-fire truck routes */
            if (!edge.isFireTruckRoute) continue;
            
            /* Determine neighbor node */
            size_t neighborIdx;
            if (edge.node1Idx == current.nodeIndex) {
                neighborIdx = edge.node2Idx;
            } else if (!edge.isOneWay && edge.node2Idx == current.nodeIndex) {
                neighborIdx = edge.node1Idx;
            } else {
                continue; /* Can't traverse this edge */
            }
            
            if (closedSet.find(neighborIdx) != closedSet.end()) {
                continue;
            }
            
            float tentativeG = gScores[current.nodeIndex] + edge.length;
            
            if (gScores.find(neighborIdx) == gScores.end() || tentativeG < gScores[neighborIdx]) {
                gScores[neighborIdx] = tentativeG;
                cameFrom[neighborIdx] = current.nodeIndex;
                
                const RoadNode& neighborNode = g_roadNetwork.nodes[neighborIdx];
                float h = sqrtf(static_cast<float>((goalNodeRef.x - neighborNode.x) * (goalNodeRef.x - neighborNode.x) +
                                                    (goalNodeRef.z - neighborNode.z) * (goalNodeRef.z - neighborNode.z)));
                
                AStarNode neighborANode;
                neighborANode.nodeIndex = neighborIdx;
                neighborANode.gScore = tentativeG;
                neighborANode.fScore = tentativeG + h;
                neighborANode.parentIndex = current.nodeIndex;
                
                openSet.push(neighborANode);
            }
        }
    }
    
    DebugLog("FindPath: No path found from node %zu to node %zu", startNode, goalNode);
    return false;
}

/*
 * SmoothPath - Apply Bezier curve smoothing to a planned route
 * Creates additional waypoints for smooth turns
 * @param route The route to smooth (modified in place)
 */
static void SmoothPath(PlannedRoute& route) {
    if (route.waypoints.size() < 3) return;
    
    std::vector<PathWaypoint> smoothedWaypoints;
    
    for (size_t i = 0; i < route.waypoints.size(); ++i) {
        const PathWaypoint& current = route.waypoints[i];
        
        if (i == 0 || i == route.waypoints.size() - 1) {
            /* Keep start and end points as-is */
            smoothedWaypoints.push_back(current);
        } else {
            /* Smooth middle points using Bezier interpolation */
            const PathWaypoint& prev = route.waypoints[i - 1];
            const PathWaypoint& next = route.waypoints[i + 1];
            
            /* Calculate segment lengths */
            float lenPrev = sqrtf(static_cast<float>((current.x - prev.x) * (current.x - prev.x) +
                                                      (current.z - prev.z) * (current.z - prev.z)));
            float lenNext = sqrtf(static_cast<float>((next.x - current.x) * (next.x - current.x) +
                                                      (next.z - current.z) * (next.z - current.z)));
            
            /* Only add smoothing points if segments are long enough */
            if (lenPrev > MIN_TURN_RADIUS * 2 && lenNext > MIN_TURN_RADIUS * 2) {
                /* Add approach point before the turn */
                PathWaypoint approach;
                approach.x = current.x - (current.x - prev.x) * BEZIER_SMOOTHING_FACTOR;
                approach.z = current.z - (current.z - prev.z) * BEZIER_SMOOTHING_FACTOR;
                approach.targetHeading = atan2f(static_cast<float>(current.x - prev.x),
                                                 -static_cast<float>(current.z - prev.z)) * RAD_TO_DEG;
                approach.speed = TRUCK_APPROACH_SPEED * 0.7f; /* Slow down for turn */
                approach.isSmoothed = true;
                smoothedWaypoints.push_back(approach);
                
                /* Add the actual waypoint */
                PathWaypoint wp = current;
                wp.speed = TRUCK_APPROACH_SPEED * 0.5f; /* Slowest at apex of turn */
                smoothedWaypoints.push_back(wp);
                
                /* Add exit point after the turn */
                PathWaypoint exit;
                exit.x = current.x + (next.x - current.x) * BEZIER_SMOOTHING_FACTOR;
                exit.z = current.z + (next.z - current.z) * BEZIER_SMOOTHING_FACTOR;
                exit.targetHeading = atan2f(static_cast<float>(next.x - current.x),
                                            -static_cast<float>(next.z - current.z)) * RAD_TO_DEG;
                exit.speed = TRUCK_APPROACH_SPEED * 0.7f;
                exit.isSmoothed = true;
                smoothedWaypoints.push_back(exit);
            } else {
                /* Segment too short, just keep the waypoint */
                smoothedWaypoints.push_back(current);
            }
        }
    }
    
    /* Calculate target headings for all waypoints */
    for (size_t i = 0; i < smoothedWaypoints.size() - 1; ++i) {
        const PathWaypoint& wp = smoothedWaypoints[i];
        const PathWaypoint& next = smoothedWaypoints[i + 1];
        smoothedWaypoints[i].targetHeading = atan2f(static_cast<float>(next.x - wp.x),
                                                     -static_cast<float>(next.z - wp.z)) * RAD_TO_DEG;
    }
    /* Last waypoint keeps its heading */
    if (smoothedWaypoints.size() > 1) {
        smoothedWaypoints.back().targetHeading = smoothedWaypoints[smoothedWaypoints.size() - 2].targetHeading;
    }
    
    route.waypoints = smoothedWaypoints;
    DebugLog("SmoothPath: Created %zu smoothed waypoints", route.waypoints.size());
}

/*
 * PlanRouteToTarget - Plan a route from start position to target using road network
 * Falls back to direct approach if no road network available
 * 
 * @param startX Starting X coordinate
 * @param startZ Starting Z coordinate  
 * @param targetX Target X coordinate
 * @param targetZ Target Z coordinate
 * @param targetHeading Final heading at target
 * @return Planned route (may be empty if planning fails)
 */
static PlannedRoute PlanRouteToTarget(double startX, double startZ, double targetX, double targetZ, float targetHeading) {
    PlannedRoute route;
    route.currentWaypointIndex = 0;
    route.isValid = false;
    route.isCompleted = false;
    
    DebugLog("PlanRouteToTarget: Planning from (%.2f, %.2f) to (%.2f, %.2f)",
             startX, startZ, targetX, targetZ);
    
    if (!g_roadNetwork.isLoaded) {
        DebugLog("PlanRouteToTarget: No road network loaded, using direct approach");
        
        /* Create simple direct path with intermediate waypoints */
        float dx = static_cast<float>(targetX - startX);
        float dz = static_cast<float>(targetZ - startZ);
        float dist = sqrtf(dx * dx + dz * dz);
        int numWaypoints = static_cast<int>(dist / PATH_NODE_DISTANCE) + 2;
        numWaypoints = std::min(numWaypoints, MAX_PATH_NODES);
        
        for (int i = 0; i < numWaypoints; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(numWaypoints - 1);
            PathWaypoint wp;
            wp.x = startX + dx * t;
            wp.z = startZ + dz * t;
            wp.targetHeading = atan2f(dx, -dz) * RAD_TO_DEG;
            wp.speed = TRUCK_APPROACH_SPEED;
            wp.isSmoothed = false;
            route.waypoints.push_back(wp);
        }
        
        /* Set final waypoint heading */
        if (!route.waypoints.empty()) {
            route.waypoints.back().targetHeading = targetHeading;
            route.waypoints.back().speed = 0.0f; /* Stop at destination */
        }
        
        route.isValid = true;
        return route;
    }
    
    /* Find nearest nodes to start and target */
    /* First, convert to road network coordinates (they're relative to airport ref point) */
    double startXLocal = startX, startZLocal = startZ;
    double targetXLocal = targetX, targetZLocal = targetZ;
    
    /* If using lat/lon reference, we need to adjust coordinates */
    /* For now, assume aircraft local coords are same as road network coords */
    
    size_t startNode = FindNearestNode(startXLocal, startZLocal, true);
    size_t goalNode = FindNearestNode(targetXLocal, targetZLocal, false);
    
    if (startNode == SIZE_MAX || goalNode == SIZE_MAX) {
        DebugLog("PlanRouteToTarget: Could not find start or goal node, using direct approach");
        
        /* Fallback to direct path */
        PathWaypoint start, end;
        start.x = startX; start.z = startZ;
        start.targetHeading = atan2f(static_cast<float>(targetX - startX), 
                                      -static_cast<float>(targetZ - startZ)) * RAD_TO_DEG;
        start.speed = TRUCK_APPROACH_SPEED;
        start.isSmoothed = false;
        
        end.x = targetX; end.z = targetZ;
        end.targetHeading = targetHeading;
        end.speed = 0.0f;
        end.isSmoothed = false;
        
        route.waypoints.push_back(start);
        route.waypoints.push_back(end);
        route.isValid = true;
        return route;
    }
    
    DebugLog("PlanRouteToTarget: Start node %zu, Goal node %zu", startNode, goalNode);
    
    /* Run A* pathfinding */
    std::vector<size_t> nodePath;
    if (!FindPath(startNode, goalNode, nodePath)) {
        DebugLog("PlanRouteToTarget: A* failed, using direct approach");
        
        /* Fallback to direct path */
        PathWaypoint start, end;
        start.x = startX; start.z = startZ;
        start.targetHeading = atan2f(static_cast<float>(targetX - startX),
                                      -static_cast<float>(targetZ - startZ)) * RAD_TO_DEG;
        start.speed = TRUCK_APPROACH_SPEED;
        start.isSmoothed = false;
        
        end.x = targetX; end.z = targetZ;
        end.targetHeading = targetHeading;
        end.speed = 0.0f;
        end.isSmoothed = false;
        
        route.waypoints.push_back(start);
        route.waypoints.push_back(end);
        route.isValid = true;
        return route;
    }
    
    /* Convert node path to waypoints */
    /* Add start position as first waypoint */
    PathWaypoint startWp;
    startWp.x = startX;
    startWp.z = startZ;
    startWp.speed = TRUCK_APPROACH_SPEED;
    startWp.isSmoothed = false;
    route.waypoints.push_back(startWp);
    
    /* Add road network nodes as waypoints */
    for (size_t nodeIdx : nodePath) {
        const RoadNode& node = g_roadNetwork.nodes[nodeIdx];
        PathWaypoint wp;
        /* Convert back to aircraft local coordinates if needed */
        wp.x = node.x;
        wp.z = node.z;
        wp.speed = TRUCK_APPROACH_SPEED;
        wp.isSmoothed = false;
        route.waypoints.push_back(wp);
    }
    
    /* Add target as final waypoint */
    PathWaypoint endWp;
    endWp.x = targetX;
    endWp.z = targetZ;
    endWp.targetHeading = targetHeading;
    endWp.speed = 0.0f;
    endWp.isSmoothed = false;
    route.waypoints.push_back(endWp);
    
    /* Calculate headings for all waypoints */
    for (size_t i = 0; i < route.waypoints.size() - 1; ++i) {
        const PathWaypoint& wp = route.waypoints[i];
        const PathWaypoint& next = route.waypoints[i + 1];
        route.waypoints[i].targetHeading = atan2f(static_cast<float>(next.x - wp.x),
                                                   -static_cast<float>(next.z - wp.z)) * RAD_TO_DEG;
    }
    
    /* Apply Bezier smoothing for natural turns */
    SmoothPath(route);
    
    route.isValid = true;
    DebugLog("PlanRouteToTarget: Created route with %zu waypoints", route.waypoints.size());
    
    return route;
}

/*
 * UpdateTruckFollowingPath - Update truck position/heading following planned path
 * Implements smooth steering and speed control
 * 
 * @param truck The truck to update
 * @param dt Delta time in seconds
 */
static void UpdateTruckFollowingPath(FireTruck& truck, float dt) {
    if (!truck.route.isValid || truck.route.isCompleted) {
        return;
    }
    
    /* Get current target waypoint */
    if (truck.route.currentWaypointIndex >= truck.route.waypoints.size()) {
        truck.route.isCompleted = true;
        truck.positioned = true;
        DebugLog("UpdateTruckFollowingPath: Truck completed route");
        return;
    }
    
    const PathWaypoint& target = truck.route.waypoints[truck.route.currentWaypointIndex];
    
    /* Calculate distance to current waypoint */
    float dx = static_cast<float>(target.x - truck.x);
    float dz = static_cast<float>(target.z - truck.z);
    float distance = sqrtf(dx * dx + dz * dz);
    
    /* Check if waypoint reached */
    if (distance < PATH_REACH_THRESHOLD) {
        truck.route.currentWaypointIndex++;
        
        if (truck.route.currentWaypointIndex >= truck.route.waypoints.size()) {
            truck.route.isCompleted = true;
            truck.positioned = true;
            truck.targetSpeed = 0.0f;
            DebugLogVerbose("UpdateTruckFollowingPath: Route completed");
            return;
        }
        
        DebugLogVerbose("UpdateTruckFollowingPath: Reached waypoint %zu/%zu",
                        truck.route.currentWaypointIndex, truck.route.waypoints.size());
    }
    
    /* Calculate desired heading to target */
    float desiredHeading = atan2f(dx, -dz) * RAD_TO_DEG;
    
    /* Look ahead for upcoming turns */
    float lookAheadDist = TURN_ANTICIPATION;
    float accumulatedDist = distance;
    float futureHeading = desiredHeading;
    
    for (size_t i = truck.route.currentWaypointIndex + 1; 
         i < truck.route.waypoints.size() && accumulatedDist < lookAheadDist; ++i) {
        const PathWaypoint& wp = truck.route.waypoints[i];
        const PathWaypoint& prevWp = truck.route.waypoints[i - 1];
        
        float segDx = static_cast<float>(wp.x - prevWp.x);
        float segDz = static_cast<float>(wp.z - prevWp.z);
        float segLen = sqrtf(segDx * segDx + segDz * segDz);
        accumulatedDist += segLen;
        
        if (accumulatedDist >= lookAheadDist) {
            futureHeading = atan2f(segDx, -segDz) * RAD_TO_DEG;
            break;
        }
    }
    
    /* Calculate heading difference for steering */
    float headingDiff = desiredHeading - truck.heading;
    while (headingDiff > 180.0f) headingDiff -= 360.0f;
    while (headingDiff < -180.0f) headingDiff += 360.0f;
    
    /* Anticipate turns: blend current heading target with future heading */
    float futureHeadingDiff = futureHeading - truck.heading;
    while (futureHeadingDiff > 180.0f) futureHeadingDiff -= 360.0f;
    while (futureHeadingDiff < -180.0f) futureHeadingDiff += 360.0f;
    
    /* Use the more aggressive turn if upcoming turn is sharper */
    if (fabsf(futureHeadingDiff) > fabsf(headingDiff) && distance < TURN_ANTICIPATION) {
        headingDiff = headingDiff * 0.7f + futureHeadingDiff * 0.3f;
    }
    
    /* Set steering angle based on heading difference */
    truck.frontSteeringAngle = ClampSteeringAngle(headingDiff);
    truck.rearSteeringAngle = CalculateRearSteeringAngle(truck.frontSteeringAngle);
    
    /* Calculate minimum turn radius and adjust speed */
    float turnRadius = WHEELBASE / fabsf(tanf(truck.frontSteeringAngle * DEG_TO_RAD) + MIN_STEERING_TANGENT);
    
    /* Speed control: slower for tighter turns and approaching waypoints */
    float maxSpeedForTurn = sqrtf(turnRadius * 2.0f); /* Simple physics approximation */
    maxSpeedForTurn = fminf(maxSpeedForTurn, TRUCK_APPROACH_SPEED);
    maxSpeedForTurn = fmaxf(maxSpeedForTurn, TRUCK_TURN_IN_PLACE_SPEED);
    
    /* Target speed from waypoint */
    float waypointSpeed = target.speed > 0.0f ? target.speed : TRUCK_APPROACH_SPEED;
    
    /* Slow down approaching waypoints */
    if (distance < TRUCK_SLOWDOWN_DISTANCE) {
        float slowdownFactor = distance / TRUCK_SLOWDOWN_DISTANCE;
        waypointSpeed = fmaxf(TRUCK_TURN_IN_PLACE_SPEED, waypointSpeed * slowdownFactor);
    }
    
    /* Use the minimum of turn speed and waypoint speed */
    truck.targetSpeed = fminf(maxSpeedForTurn, waypointSpeed);
    
    /* Smooth speed transition */
    truck.speed = UpdateSpeedSmooth(truck.speed, truck.targetSpeed, dt);
    
    /* Calculate turning rate using Ackermann steering */
    float turningRate = CalculateTurningRate(truck.speed, truck.frontSteeringAngle, truck.rearSteeringAngle);
    
    /* Update heading */
    truck.heading += turningRate * dt;
    while (truck.heading >= 360.0f) truck.heading -= 360.0f;
    while (truck.heading < 0.0f) truck.heading += 360.0f;
    
    /* Move truck forward */
    float headingRad = truck.heading * DEG_TO_RAD;
    float moveDistance = truck.speed * dt;
    
    truck.x += sinf(headingRad) * moveDistance;
    truck.z += -cosf(headingRad) * moveDistance;
    truck.y = GetTerrainHeight(static_cast<float>(truck.x), static_cast<float>(truck.z));
    
    /* Update wheel rotation */
    UpdateWheelRotationAngle(truck, moveDistance);
    
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
}
