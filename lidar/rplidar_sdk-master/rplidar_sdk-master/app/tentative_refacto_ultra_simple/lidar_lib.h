#ifndef LIDAR_LIB_H
#define LIDAR_LIB_H
#include "ImuOTOS.h"

#include <vector>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

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


constexpr int RAYON_PILIER = 16;             
constexpr float RESOLUTION = 0.25f;
constexpr int NUM_ANGLES = static_cast<int>(360.0f / RESOLUTION);
extern std::vector<position> pillars;

// Initialise les positions des piliers selon l'équipe (0 ou 1)
void initPillars(int team);

bool initIMU(ImuOTOS& imu);
position relativeCoords(TrackResult m) ;

// Résout la pose avec 2 ou 3 piliers
position computePose(TrackResult *meas) ;



void trackPoints(const std::vector<float>& scan,
                 std::vector<TrackResult>& trackedPoints,
                 float resolution,
                position posrobot) // Traque tous les piliers
;

void inittrackedpoints(std::vector<TrackResult>& trackedPoints, position pos, int numPoints);

                        
void point_in_table2(const std::vector<float>& scan, 
                    std::vector<position>& points_in_table,
                    float resolution, 
                    const position& robot);
void point_in_table(const std::vector<float>& scan,std::vector<position>& points_in_table, float resolution,position robot);


// cette fontion a pourbut de trouver le centre du pilier( d=100mm)
// deja trouver a angle et distance, adapter le span de recherche en fonction
// de la distance et filtrer les point trop eloignés de distance
float findcenter(const std::vector<float>& scan, float angle, float distance) ;
void try_accept_client(int& client_sock, int server_sock)   ;
void init_socket(int& server_sock, int& client_sock, struct sockaddr_un& addr, const char *socket_path);
bool checkSLAMTECLIDARHealth(sl::ILidarDriver * drv);
void print_usage(int argc, const char * argv[]);
bool grabAndUpdateScan(std::vector<float>& scan, sl::ILidarDriver* drv);
ImuPose position_to_imu(const position& pos) ;
sl::ILidarDriver* initLidar(int argc, const char* argv[], sl_lidar_response_device_info_t& devinfo, int& opt_channel_type, sl::IChannel*& _channel);
#endif // LIDAR_LIB_H