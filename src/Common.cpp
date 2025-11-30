/*
 * Common.cpp - Common utility functions for WaterSalute plugin
 */

#include "Common.h"

/* Debug log function - outputs to X-Plane log */
void DebugLog(const char* format, ...) {
    char buffer[DEBUG_LOG_MSG_SIZE];
    va_list args;
    va_start(args, format);
    
    /* Format the message with plugin prefix */
    int prefixLen = snprintf(buffer, DEBUG_LOG_MSG_SIZE, "WaterSalute: ");
    if (prefixLen > 0 && static_cast<size_t>(prefixLen) < DEBUG_LOG_MSG_SIZE - 1) {
        vsnprintf(buffer + prefixLen, DEBUG_LOG_MSG_SIZE - prefixLen - 1, format, args);
    }
    
    va_end(args);
    
    /* Ensure newline at end */
    size_t len = strlen(buffer);
    if (len > 0 && len < DEBUG_LOG_MSG_SIZE - 1 && buffer[len - 1] != '\n') {
        buffer[len] = '\n';
        buffer[len + 1] = '\0';
    }
    
    XPLMDebugString(buffer);
}

/* Verbose debug log - only outputs when DEBUG_VERBOSE is true */
void DebugLogVerbose(const char* format, ...) {
    if (!DEBUG_VERBOSE) return;
    
    char buffer[DEBUG_LOG_MSG_SIZE];
    va_list args;
    va_start(args, format);
    
    int prefixLen = snprintf(buffer, DEBUG_LOG_MSG_SIZE, "WaterSalute [VERBOSE]: ");
    if (prefixLen > 0 && static_cast<size_t>(prefixLen) < DEBUG_LOG_MSG_SIZE - 1) {
        vsnprintf(buffer + prefixLen, DEBUG_LOG_MSG_SIZE - prefixLen - 1, format, args);
    }
    
    va_end(args);
    
    size_t len = strlen(buffer);
    if (len > 0 && len < DEBUG_LOG_MSG_SIZE - 1 && buffer[len - 1] != '\n') {
        buffer[len] = '\n';
        buffer[len + 1] = '\0';
    }
    
    XPLMDebugString(buffer);
}

/* Get state name for debugging */
const char* GetStateName(PluginState state) {
    switch (state) {
        case STATE_IDLE: return "STATE_IDLE";
        case STATE_TRUCKS_APPROACHING: return "STATE_TRUCKS_APPROACHING";
        case STATE_TRUCKS_POSITIONING: return "STATE_TRUCKS_POSITIONING";
        case STATE_WATER_SPRAYING: return "STATE_WATER_SPRAYING";
        case STATE_TRUCKS_LEAVING: return "STATE_TRUCKS_LEAVING";
        default: return "UNKNOWN";
    }
}

/* Normalize angle to 0-360 range */
float NormalizeAngle360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

/* Smooth speed transition using acceleration/deceleration */
float UpdateSpeedSmooth(float currentSpeed, float targetSpeed, float dt) {
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

/* Clamp steering angle to valid range */
float ClampSteeringAngle(float angle) {
    if (angle > MAX_STEERING_ANGLE) return MAX_STEERING_ANGLE;
    if (angle < -MAX_STEERING_ANGLE) return -MAX_STEERING_ANGLE;
    return angle;
}

/* Calculate rear steering angle based on front angle (counter-steering) */
float CalculateRearSteeringAngle(float frontSteerAngle) {
    /* Rear axle uses counter-steering (opposite direction) with reduced magnitude */
    /* This creates a tighter turning radius for the 8x8 truck */
    return -frontSteerAngle * REAR_STEER_RATIO;
}

/* Calculate turning rate using Ackermann steering model */
float CalculateTurningRate(float speed, float frontSteerAngleDeg, float rearSteerAngleDeg) {
    /* For very low speeds or zero steering, no turning */
    if (fabsf(speed) < 0.01f || fabsf(frontSteerAngleDeg) < 0.1f) {
        return 0.0f;
    }
    
    /* Convert steering angles to radians */
    float frontAngleRad = frontSteerAngleDeg * DEG_TO_RAD;
    float rearAngleRad = rearSteerAngleDeg * DEG_TO_RAD;
    
    /* Combined steering effect */
    float effectiveSteerAngle = (tanf(frontAngleRad) - tanf(rearAngleRad)) / 2.0f;
    
    /* Angular velocity = v * tan(steer) / wheelbase */
    float angularVelocity = speed * effectiveSteerAngle / WHEELBASE;
    
    /* Convert to degrees per second */
    return angularVelocity * RAD_TO_DEG;
}
