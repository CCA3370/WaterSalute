/*
 * PathPlanning.h - Path planning with A* algorithm and Bezier smoothing
 * 
 * This module handles pathfinding through the road network and
 * smooth path generation for natural truck movement.
 */

#ifndef WATERSALUTE_PATHPLANNING_H
#define WATERSALUTE_PATHPLANNING_H

#include "Common.h"
#include "RoadNetwork.h"

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

/* Path planning functions */
bool FindPath(size_t startNode, size_t goalNode, std::vector<size_t>& path);
void SmoothPath(PlannedRoute& route);
PlannedRoute PlanRouteToTarget(double startX, double startZ, double targetX, double targetZ, float targetHeading);

#endif /* WATERSALUTE_PATHPLANNING_H */
