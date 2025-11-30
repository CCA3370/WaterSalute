/*
 * FireTruck.cpp - Fire truck management and movement
 */

#include "FireTruck.h"

/* External declarations for objects defined in WaterSalute.cpp */
extern XPLMObjectRef g_waterDropObject;
extern XPLMProbeRef g_terrainProbe;
extern std::mt19937 g_rng;
extern std::uniform_real_distribution<float> g_randomDist;
extern int g_particlesEmittedTotal;

/* Global truck instances */
FireTruck g_leftTruck;
FireTruck g_rightTruck;

/* Helper function declarations */
float GetTerrainHeight(float x, float z);

/* Get truck by index (0 = left, 1 = right) */
FireTruck* GetTruckByIndex(int index) {
    if (index == 0) return &g_leftTruck;
    if (index == 1) return &g_rightTruck;
    return nullptr;
}

/* Initialize a fire truck */
void InitializeTruck(FireTruck& truck) {
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
    truck.frontSteeringAngle = 0.0f;
    truck.rearSteeringAngle = 0.0f;
    truck.wheelRotationAngle = 0.0f;
    truck.cannonPitch = DEFAULT_CANNON_PITCH;
    truck.cannonYaw = 0.0f;
    truck.speed = 0.0f;
    truck.targetSpeed = 0.0f;
    truck.isTurningBeforeLeave = false;
    truck.leaveHeading = 0.0f;
    /* Initialize route planning */
    truck.route.waypoints.clear();
    truck.route.currentWaypointIndex = 0;
    truck.route.isValid = false;
    truck.route.isCompleted = false;
    truck.useRoadNetwork = false;
}

/* Clean up a fire truck */
void CleanupTruck(FireTruck& truck) {
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

/* Update wheel rotation angle based on distance moved */
void UpdateWheelRotationAngle(FireTruck& truck, float distanceMoved) {
    /* Calculate rotation in degrees based on circumference */
    float wheelCircumference = 2.0f * PI * WHEEL_RADIUS;
    float rotationDegrees = (distanceMoved / wheelCircumference) * 360.0f;
    truck.wheelRotationAngle = NormalizeAngle360(truck.wheelRotationAngle + rotationDegrees);
}

/* Update truck following planned path */
void UpdateTruckFollowingPath(FireTruck& truck, float dt) {
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

/* Emit a water particle from truck's nozzle */
void EmitParticle(FireTruck& truck) {
    WaterParticle particle;
    
    /* Calculate nozzle world position */
    float headingRad = truck.heading * DEG_TO_RAD;
    float cosH = cosf(headingRad);
    float sinH = sinf(headingRad);
    
    /* Get cannon angles */
    float pitchRad = truck.cannonPitch * DEG_TO_RAD;
    float yawRad = (truck.cannonYaw + truck.heading) * DEG_TO_RAD;  /* Yaw is relative to truck heading */
    
    /* Nozzle position in world coordinates */
    particle.x = static_cast<float>(truck.x) + sinH * truck.nozzleOffsetZ + cosH * truck.nozzleOffsetX;
    particle.y = static_cast<float>(truck.y) + truck.nozzleOffsetY;
    particle.z = static_cast<float>(truck.z) - cosH * truck.nozzleOffsetZ + sinH * truck.nozzleOffsetX;
    
    /* Calculate initial velocity for water arc */
    float initialSpeed = WATER_JET_HEIGHT * 2.5f;  /* Speed to reach target height */
    
    /* Add some randomness for spray effect */
    float spreadAngle = 0.05f;  /* ~3 degrees spread */
    float randPitch = pitchRad + g_randomDist(g_rng) * spreadAngle;
    float randYaw = yawRad + g_randomDist(g_rng) * spreadAngle;
    
    /* Calculate velocity components */
    float cosPitch = cosf(randPitch);
    float sinPitch = sinf(randPitch);
    float cosYaw = cosf(randYaw);
    float sinYaw = sinf(randYaw);
    
    particle.vx = initialSpeed * cosPitch * sinYaw;
    particle.vy = initialSpeed * sinPitch;
    particle.vz = -initialSpeed * cosPitch * cosYaw;
    
    /* Add some turbulence */
    particle.vx += g_randomDist(g_rng) * 0.5f;
    particle.vy += g_randomDist(g_rng) * 0.3f;
    particle.vz += g_randomDist(g_rng) * 0.5f;
    
    particle.lifetime = PARTICLE_LIFETIME;
    particle.maxLifetime = PARTICLE_LIFETIME;
    particle.active = true;
    
    /* Create instance for this particle if water drop model is loaded */
    if (g_waterDropObject) {
        static const char* noDataRefs[] = { nullptr };
        particle.instance = XPLMCreateInstance(g_waterDropObject, noDataRefs);
        if (particle.instance) {
            /* Set initial position for instance */
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
    } else {
        particle.instance = nullptr;
    }
    
    truck.particles.push_back(particle);
    g_particlesEmittedTotal++;
}

/* Update all water particles */
void UpdateWaterParticles(float dt) {
    if (g_state != STATE_WATER_SPRAYING) return;
    
    auto updateParticlesForTruck = [dt](FireTruck& truck) {
        /* Emit new particles */
        float currentTime = XPLMGetElapsedTime();
        if (currentTime - truck.lastEmitTime >= PARTICLE_EMIT_RATE) {
            /* Limit total particles per truck */
            int activeCount = 0;
            for (const auto& p : truck.particles) {
                if (p.active) activeCount++;
            }
            
            if (activeCount < NUM_PARTICLES_PER_JET) {
                EmitParticle(truck);
                truck.lastEmitTime = currentTime;
            }
        }
        
        /* Update existing particles */
        for (auto& particle : truck.particles) {
            if (!particle.active) continue;
            
            /* Apply gravity and drag */
            particle.vy -= 9.81f * dt;  /* Gravity */
            
            /* Apply air resistance */
            float speed = sqrtf(particle.vx * particle.vx + 
                               particle.vy * particle.vy + 
                               particle.vz * particle.vz);
            if (speed > 0.01f) {
                float dragForce = PARTICLE_DRAG * speed * speed;
                float dragAccel = dragForce / speed;
                particle.vx -= particle.vx / speed * dragAccel * dt;
                particle.vy -= particle.vy / speed * dragAccel * dt;
                particle.vz -= particle.vz / speed * dragAccel * dt;
            }
            
            /* Add turbulence */
            particle.vx += g_randomDist(g_rng) * PARTICLE_TURBULENCE;
            particle.vy += g_randomDist(g_rng) * PARTICLE_TURBULENCE;
            particle.vz += g_randomDist(g_rng) * PARTICLE_TURBULENCE;
            
            /* Update position */
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;
            particle.z += particle.vz * dt;
            
            /* Check ground collision */
            float groundY = GetTerrainHeight(particle.x, particle.z);
            if (particle.y < groundY) {
                particle.y = groundY;
                particle.vy = 0;
                particle.vx *= 0.5f;  /* Splash friction */
                particle.vz *= 0.5f;
            }
            
            /* Update instance position */
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
            if (particle.lifetime <= 0) {
                particle.active = false;
                /* Destroy instance when particle dies */
                if (particle.instance) {
                    XPLMDestroyInstance(particle.instance);
                    particle.instance = nullptr;
                }
            }
        }
        
        /* Remove dead particles periodically */
        if (truck.particles.size() > NUM_PARTICLES_PER_JET * 2) {
            truck.particles.erase(
                std::remove_if(truck.particles.begin(), truck.particles.end(),
                    [](const WaterParticle& p) { return !p.active; }),
                truck.particles.end()
            );
        }
    };
    
    updateParticlesForTruck(g_leftTruck);
    updateParticlesForTruck(g_rightTruck);
}

/* ============================================================================
 * Raindrop Effect on Windshield Implementation
 * ============================================================================
 * When the aircraft passes through the water gate, water droplets appear
 * on the windshield to simulate the effect of water hitting the glass.
 */

/* Raindrop effect state */
static XPLMDataRef g_drRainOnAircraft = nullptr;      /* X-Plane rain on aircraft dataref */
static XPLMDataRef g_drPrecipOnAircraft = nullptr;    /* Alternative precipitation dataref */
static float g_currentRaindropIntensity = 0.0f;       /* Current raindrop intensity (0.0 - 1.0) */
static float g_targetRaindropIntensity = 0.0f;        /* Target raindrop intensity */
static float g_raindropUpdateTimer = 0.0f;            /* Timer for detection updates */
static bool g_raindropEffectActive = false;           /* Whether the effect is currently active */
static float g_originalRainValue = 0.0f;              /* Original rain value before we modified it */
static bool g_savedOriginalRain = false;              /* Whether we've saved the original rain value */

/* Initialize the raindrop effect system */
void InitializeRaindropEffect() {
    DebugLog("Initializing raindrop effect system...");
    
    /* Try to find the rain/precipitation dataref */
    /* This dataref controls how much rain/water is visible on the aircraft */
    g_drRainOnAircraft = XPLMFindDataRef("sim/private/controls/rain/precipitation_on_aircraft_ratio");
    if (!g_drRainOnAircraft) {
        /* Try alternative dataref */
        g_drRainOnAircraft = XPLMFindDataRef("sim/weather/rain_percent");
    }
    if (!g_drRainOnAircraft) {
        /* Try yet another alternative */
        g_drPrecipOnAircraft = XPLMFindDataRef("sim/graphics/effects/rain_scale");
    }
    
    g_currentRaindropIntensity = 0.0f;
    g_targetRaindropIntensity = 0.0f;
    g_raindropUpdateTimer = 0.0f;
    g_raindropEffectActive = false;
    g_savedOriginalRain = false;
    
    if (g_drRainOnAircraft) {
        DebugLog("Raindrop effect system initialized with primary dataref");
    } else if (g_drPrecipOnAircraft) {
        DebugLog("Raindrop effect system initialized with fallback dataref");
    } else {
        DebugLog("WARNING: No rain dataref found - raindrop effect may not be visible");
    }
}

/* Clean up the raindrop effect system */
void CleanupRaindropEffect() {
    /* Restore original rain value if we modified it */
    if (g_savedOriginalRain) {
        if (g_drRainOnAircraft) {
            XPLMSetDataf(g_drRainOnAircraft, g_originalRainValue);
        } else if (g_drPrecipOnAircraft) {
            XPLMSetDataf(g_drPrecipOnAircraft, g_originalRainValue);
        }
        g_savedOriginalRain = false;
    }
    
    g_currentRaindropIntensity = 0.0f;
    g_targetRaindropIntensity = 0.0f;
    g_raindropEffectActive = false;
    
    DebugLog("Raindrop effect system cleaned up");
}

/* Count water particles near the aircraft */
int CountNearbyParticles(double acX, double acY, double acZ) {
    int nearbyCount = 0;
    
    auto countForTruck = [acX, acY, acZ, &nearbyCount](const FireTruck& truck) {
        for (const auto& particle : truck.particles) {
            if (!particle.active) continue;
            
            /* Calculate 3D distance from aircraft to particle */
            double dx = particle.x - acX;
            double dy = particle.y - acY;
            double dz = particle.z - acZ;
            
            /* Check if particle is within detection radius horizontally */
            double horizontalDist = sqrt(dx * dx + dz * dz);
            if (horizontalDist > RAINDROP_DETECTION_RADIUS) continue;
            
            /* Check if particle is at approximately the same height (with tolerance) */
            if (fabs(dy) > RAINDROP_DETECTION_HEIGHT) continue;
            
            nearbyCount++;
        }
    };
    
    countForTruck(g_leftTruck);
    countForTruck(g_rightTruck);
    
    return nearbyCount;
}

/* Get current raindrop intensity */
float GetCurrentRaindropIntensity() {
    return g_currentRaindropIntensity;
}

/* Update the raindrop effect based on aircraft proximity to water */
void UpdateRaindropEffect(float dt, double acX, double acY, double acZ) {
    /* Only update when water is spraying */
    if (g_state != STATE_WATER_SPRAYING) {
        /* Fade out the effect when not spraying */
        if (g_currentRaindropIntensity > 0.0f) {
            g_targetRaindropIntensity = 0.0f;
            g_currentRaindropIntensity -= dt / RAINDROP_FADE_OUT_TIME;
            if (g_currentRaindropIntensity < 0.0f) {
                g_currentRaindropIntensity = 0.0f;
            }
            
            /* Apply the effect */
            if (g_drRainOnAircraft) {
                XPLMSetDataf(g_drRainOnAircraft, g_originalRainValue + g_currentRaindropIntensity);
            } else if (g_drPrecipOnAircraft) {
                XPLMSetDataf(g_drPrecipOnAircraft, g_originalRainValue + g_currentRaindropIntensity);
            }
            
            /* Restore original value when effect is done */
            if (g_currentRaindropIntensity == 0.0f && g_savedOriginalRain) {
                if (g_drRainOnAircraft) {
                    XPLMSetDataf(g_drRainOnAircraft, g_originalRainValue);
                } else if (g_drPrecipOnAircraft) {
                    XPLMSetDataf(g_drPrecipOnAircraft, g_originalRainValue);
                }
                g_savedOriginalRain = false;
                g_raindropEffectActive = false;
                DebugLog("Raindrop effect faded out completely");
            }
        }
        return;
    }
    
    /* Save original rain value before first modification */
    if (!g_savedOriginalRain) {
        if (g_drRainOnAircraft) {
            g_originalRainValue = XPLMGetDataf(g_drRainOnAircraft);
        } else if (g_drPrecipOnAircraft) {
            g_originalRainValue = XPLMGetDataf(g_drPrecipOnAircraft);
        }
        g_savedOriginalRain = true;
        DebugLog("Saved original rain value: %.3f", g_originalRainValue);
    }
    
    /* Update detection less frequently for performance */
    g_raindropUpdateTimer += dt;
    if (g_raindropUpdateTimer >= RAINDROP_UPDATE_INTERVAL) {
        g_raindropUpdateTimer = 0.0f;
        
        /* Count nearby particles */
        int nearbyParticles = CountNearbyParticles(acX, acY, acZ);
        
        /* Calculate target intensity based on particle count */
        /* More particles = stronger rain effect */
        float particleRatio = static_cast<float>(nearbyParticles) / static_cast<float>(NUM_PARTICLES_PER_JET * 2);
        g_targetRaindropIntensity = fminf(particleRatio * 2.0f, RAINDROP_EFFECT_MAX);
        
        if (nearbyParticles > 0 && !g_raindropEffectActive) {
            DebugLog("Aircraft entering water spray - %d particles nearby", nearbyParticles);
            g_raindropEffectActive = true;
        } else if (nearbyParticles == 0 && g_raindropEffectActive && g_currentRaindropIntensity < 0.01f) {
            DebugLog("Aircraft left water spray area");
            g_raindropEffectActive = false;
        }
    }
    
    /* Smooth transition to target intensity */
    if (g_currentRaindropIntensity < g_targetRaindropIntensity) {
        /* Fade in */
        g_currentRaindropIntensity += dt / RAINDROP_FADE_IN_TIME;
        if (g_currentRaindropIntensity > g_targetRaindropIntensity) {
            g_currentRaindropIntensity = g_targetRaindropIntensity;
        }
    } else if (g_currentRaindropIntensity > g_targetRaindropIntensity) {
        /* Fade out */
        g_currentRaindropIntensity -= dt / RAINDROP_FADE_OUT_TIME;
        if (g_currentRaindropIntensity < g_targetRaindropIntensity) {
            g_currentRaindropIntensity = g_targetRaindropIntensity;
        }
    }
    
    /* Apply the rain effect to the windshield */
    /* The rain effect is added on top of any existing weather rain */
    float effectValue = g_originalRainValue + g_currentRaindropIntensity;
    effectValue = fminf(effectValue, 1.0f);  /* Cap at maximum */
    
    if (g_drRainOnAircraft) {
        XPLMSetDataf(g_drRainOnAircraft, effectValue);
    } else if (g_drPrecipOnAircraft) {
        XPLMSetDataf(g_drPrecipOnAircraft, effectValue);
    }
}
