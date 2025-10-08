#ifndef LIDAR_LIB_H
#define LIDAR_LIB_H

#include <vector>

#pragma once

struct position {
    float x;
    float y;
    float angle;
    position() : x(0), y(0), angle(0) {}
    position(float x_, float y_, float angle_) : x(x_), y(y_), angle(angle_) {}
};

struct TrackResult {
    float angle;    // en radians
    float distance; // en mm
    bool found;
};


constexpr int RAYON_PILIER = 50;             // en mm
constexpr float RESOLUTION = 0.25f;
constexpr int NUM_ANGLES = static_cast<int>(360.0f / RESOLUTION);
extern const std::vector<position> pillars;



position relativeCoords(TrackResult m) ;

// Résout la pose avec 2 ou 3 piliers
position computePose(TrackResult *meas) ;



void trackPoints(const std::vector<float>& scan,
                 std::vector<TrackResult>& trackedPoints,
                 float resolution,
                position posrobot) // Traque tous les piliers
;

void inittrackedpoints(std::vector<TrackResult>& trackedPoints, position pos, int numPoints);


// cette fontion a pourbut de trouver le centre du pilier( d=100mm)
// deja trouver a angle et distance, adapter le span de recherche en fonction
// de la distance et filtrer les point trop eloignés de distance
float findcenter(const std::vector<float>& scan, float angle, float distance) ;

#endif // LIDAR_LIB_H