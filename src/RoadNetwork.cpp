/*
 * RoadNetwork.cpp - Road network parsing and management
 */

#include "RoadNetwork.h"

/* Global road network instance */
RoadNetwork g_roadNetwork;

/*
 * LatLonToLocal - Convert latitude/longitude to local OpenGL coordinates
 * Uses simple equirectangular projection centered on reference point
 */
void LatLonToLocal(double lat, double lon, double refLat, double refLon, double& x, double& z) {
    double latRad = refLat * DEG_TO_RAD;
    /* X is East-West distance */
    x = (lon - refLon) * DEG_TO_RAD * EARTH_RADIUS_METERS * cos(latRad);
    /* Z in X-Plane is negative North, positive South */
    z = -(lat - refLat) * DEG_TO_RAD * EARTH_RADIUS_METERS;
}

/*
 * LocalToLatLon - Convert local OpenGL coordinates to latitude/longitude
 */
void LocalToLatLon(double x, double z, double refLat, double refLon, double& lat, double& lon) {
    double latRad = refLat * DEG_TO_RAD;
    lon = refLon + x / (EARTH_RADIUS_METERS * cos(latRad)) * RAD_TO_DEG;
    lat = refLat - z / EARTH_RADIUS_METERS * RAD_TO_DEG;
}

/*
 * ParseAptDatLine - Parse a single line from apt.dat
 */
void ParseAptDatLine(const std::string& line, std::vector<std::string>& tokens) {
    tokens.clear();
    std::istringstream iss(line);
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
}

/*
 * LoadAptDat - Load airport ground routes from apt.dat
 */
bool LoadAptDat(double acLat, double acLon) {
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
    
    /* Parse apt.dat file */
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
 */
size_t FindNearestNode(double x, double z, bool firetruckRoutesOnly) {
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
