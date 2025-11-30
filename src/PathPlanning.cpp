/*
 * PathPlanning.cpp - A* pathfinding and Bezier curve smoothing
 */

#include "PathPlanning.h"

/*
 * FindPath - A* pathfinding between two nodes
 */
bool FindPath(size_t startNode, size_t goalNode, std::vector<size_t>& path) {
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
 */
void SmoothPath(PlannedRoute& route) {
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
 */
PlannedRoute PlanRouteToTarget(double startX, double startZ, double targetX, double targetZ, float targetHeading) {
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
    size_t startNode = FindNearestNode(startX, startZ, true);
    size_t goalNode = FindNearestNode(targetX, targetZ, false);
    
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
