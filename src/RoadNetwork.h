/*
 * RoadNetwork.h - Road network data structures and apt.dat parsing
 * 
 * This module handles parsing of X-Plane apt.dat files to extract
 * airport ground route networks for fire truck navigation.
 */

#ifndef WATERSALUTE_ROADNETWORK_H
#define WATERSALUTE_ROADNETWORK_H

#include "Common.h"

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

/* Global road network instance */
extern RoadNetwork g_roadNetwork;

/* Coordinate conversion functions */
void LatLonToLocal(double lat, double lon, double refLat, double refLon, double& x, double& z);
void LocalToLatLon(double x, double z, double refLat, double refLon, double& lat, double& lon);

/* apt.dat parsing functions */
void ParseAptDatLine(const std::string& line, std::vector<std::string>& tokens);
bool LoadAptDat(double acLat, double acLon);
size_t FindNearestNode(double x, double z, bool firetruckRoutesOnly);

#endif /* WATERSALUTE_ROADNETWORK_H */
