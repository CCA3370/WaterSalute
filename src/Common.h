/*
 * Common.h - Common includes, constants and utilities for WaterSalute plugin
 * 
 * This file contains shared includes, constants, and utility functions
 * used across all modules of the plugin.
 */

#ifndef WATERSALUTE_COMMON_H
#define WATERSALUTE_COMMON_H

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
static const XPLMDrawingPhase WATER_DRAWING_PHASE = xplm_Phase_Modern3D; /* Drawing phase for water particles */

/* Raindrop effect on windshield constants */
static const float RAINDROP_DETECTION_RADIUS = 50.0f;  /* Radius to detect water particles near aircraft (meters) */
static const float RAINDROP_DETECTION_HEIGHT = 20.0f;  /* Height range to detect water particles (meters) */
static const float RAINDROP_EFFECT_MAX = 0.8f;         /* Maximum rain effect intensity (0.0 - 1.0) */
static const float RAINDROP_FADE_IN_TIME = 0.5f;       /* Time to fade in rain effect (seconds) */
static const float RAINDROP_FADE_OUT_TIME = 2.0f;      /* Time to fade out rain effect (seconds) */
static const float RAINDROP_UPDATE_INTERVAL = 0.1f;    /* Interval to update raindrop detection (seconds) */

/* Wingspan validation constants */
static const float MIN_SEMISPAN_METERS = 2.5f;     /* Minimum semispan (half wingspan) in meters */
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
static const size_t DEBUG_LOG_MSG_SIZE = DEBUG_BUFFER_SIZE + DEBUG_LOG_PREFIX_SIZE + 2;

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
static const float MIN_STEERING_TANGENT = 0.01f;   /* Minimum tangent value to prevent division by zero */

/* Wheel physics constants */
static const float WHEEL_RADIUS = 0.5f;                 /* Wheel radius in meters */
static const float MAX_STEERING_ANGLE = 45.0f;          /* Maximum steering angle in degrees */
static const float WHEELBASE = 6.0f;                    /* Distance between front and rear axles in meters */
static const float REAR_STEER_RATIO = 0.3f;             /* Rear axle steering ratio (counter-steering magnitude) */
static const float MIN_CANNON_PITCH = 0.0f;             /* Minimum cannon pitch angle */
static const float MAX_CANNON_PITCH = 90.0f;            /* Maximum cannon pitch angle */
static const float DEFAULT_CANNON_PITCH = 45.0f;        /* Default cannon pitch angle for water arc */
static const float PI = 3.14159265f;                    /* Pi constant */
static const float DEG_TO_RAD = PI / 180.0f;            /* Degrees to radians conversion */
static const float RAD_TO_DEG = 180.0f / PI;            /* Radians to degrees conversion */

/* Plugin state enumeration */
enum PluginState {
    STATE_IDLE,
    STATE_TRUCKS_APPROACHING,
    STATE_TRUCKS_POSITIONING,
    STATE_WATER_SPRAYING,
    STATE_TRUCKS_LEAVING
};

/* Global plugin state (defined in WaterSalute.cpp) */
extern PluginState g_state;

/* Utility function declarations */
void DebugLog(const char* format, ...);
void DebugLogVerbose(const char* format, ...);
const char* GetStateName(PluginState state);
float NormalizeAngle360(float angle);
float UpdateSpeedSmooth(float currentSpeed, float targetSpeed, float dt);
float ClampSteeringAngle(float angle);
float CalculateRearSteeringAngle(float frontSteerAngle);
float CalculateTurningRate(float speed, float frontSteerAngleDeg, float rearSteerAngleDeg);

#endif /* WATERSALUTE_COMMON_H */
