/*
 * FireTruck.h - Fire truck structure and movement logic
 * 
 * This module handles fire truck data structures, positioning,
 * movement, and steering control.
 */

#ifndef WATERSALUTE_FIRETRUCK_H
#define WATERSALUTE_FIRETRUCK_H

#include "Common.h"
#include "PathPlanning.h"

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
    float frontSteeringAngle;     /* Front axles steering angle in degrees (-45 to 45) */
    float rearSteeringAngle;      /* Rear axle steering angle in degrees (-45 to 45) */
    float wheelRotationAngle;     /* Wheel rotation angle in degrees (0-360°) */
    float cannonPitch;       /* Water cannon pitch angle in degrees (0 to 90) */
    float cannonYaw;         /* Water cannon yaw angle in degrees (-180 to 180) */
    float speed;             /* Current speed in m/s */
    float targetSpeed;       /* Target speed in m/s (for smooth acceleration/deceleration) */
    bool isTurningBeforeLeave; /* Flag: true when truck is turning before leaving */
    float leaveHeading;      /* Heading to use when leaving */
    PlannedRoute route;      /* Planned path from road network */
    bool useRoadNetwork;     /* Whether to follow road network or direct approach */
};

/* Global truck instances */
extern FireTruck g_leftTruck;
extern FireTruck g_rightTruck;

/* Fire truck functions */
void InitializeTruck(FireTruck& truck);
void CleanupTruck(FireTruck& truck);
void UpdateWheelRotationAngle(FireTruck& truck, float distanceMoved);
void UpdateTruckFollowingPath(FireTruck& truck, float dt);
FireTruck* GetTruckByIndex(int index);

/* Water particle functions */
void EmitParticle(FireTruck& truck);
void UpdateWaterParticles(float dt);

/* Raindrop effect functions */
void InitializeRaindropEffect();
void CleanupRaindropEffect();
void UpdateRaindropEffect(float dt, double acX, double acY, double acZ);
int CountNearbyParticles(double acX, double acY, double acZ);
float GetCurrentRaindropIntensity();

#endif /* WATERSALUTE_FIRETRUCK_H */
