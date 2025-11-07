/**
 * @file lidar_lib_refactored.cpp
 * @brief Implémentation de la bibliothèque de traitement LIDAR
 */

#include "lidar_lib_refactored.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace sl;
using namespace std::chrono;

// ============================================================================
// INITIALISATION ET CONFIGURATION
// ============================================================================

std::vector<Position> initPillars(Team team) {
    const std::vector<Position>& pillars = getPillarPositions(team);
    
    std::cout << "🎯 " << getTeamName(team) << " - Piliers chargés:\n";
    for (size_t i = 0; i < pillars.size(); ++i) {
        std::cout << "  Pilier " << (i+1) 
                  << ": x=" << pillars[i].x_mm << " mm"
                  << ", y=" << pillars[i].y_mm << " mm\n";
    }
    
    return pillars;
}

std::vector<TrackResult> initTrackedPoints(const Position& robotPos, 
                                           const std::vector<Position>& pillarPositions) {
    std::vector<TrackResult> trackedPoints;
    trackedPoints.reserve(pillarPositions.size());
    
    for (const auto& pillar : pillarPositions) {
        TrackResult tr;
        float dx = pillar.x_mm - robotPos.x_mm;
        float dy = pillar.y_mm - robotPos.y_mm;
        
        tr.distance_mm = std::sqrt(dx * dx + dy * dy);
        tr.angle_rad = std::atan2(dy, dx) - robotPos.angle_rad;
        
        // Normaliser l'angle dans [0, 2π]
        while (tr.angle_rad < 0) tr.angle_rad += 2 * PI;
        while (tr.angle_rad >= 2 * PI) tr.angle_rad -= 2 * PI;
        
        tr.found = true;
        trackedPoints.push_back(tr);
    }
    
    return trackedPoints;
}

// ============================================================================
// TRAITEMENT SCAN LIDAR
// ============================================================================

bool grabAndUpdateScan(LidarScan& scan, ILidarDriver* drv) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    auto t_start = high_resolution_clock::now();
    sl_result op_result = drv->grabScanDataHq(nodes, count);
    auto t_end = high_resolution_clock::now();
    auto duration_ms = duration_cast<milliseconds>(t_end - t_start).count();
    
    std::cout << "📡 Scan acquis en " << duration_ms << " ms (" << count << " points)\n";
    
    if (!SL_IS_OK(op_result)) {
        std::cerr << "❌ Erreur acquisition scan: " << op_result << "\n";
        return false;
    }

    // Réinitialiser le scan
    std::fill(scan.begin(), scan.end(), 0.0f);
    
    for (size_t pos = 0; pos < count; ++pos) {
        float angle_deg = (nodes[pos].angle_z_q14 * 90.0f) / 16384.0f;
        float distance_mm = nodes[pos].dist_mm_q2 / 4.0f;
        int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        
        if (quality < TRACKING_MIN_QUALITY) continue;
        
        // Calcul index (scan inversé : 1° -> 359°)
        int index = static_cast<int>(std::round(angle_deg / LIDAR_RESOLUTION_DEG));
        index = (LIDAR_NUM_ANGLES - index) % LIDAR_NUM_ANGLES;
        
        if (index >= 0 && index < LIDAR_NUM_ANGLES) {
            scan[index] = distance_mm;
        }
    }
    
    return true;
}

void filterPointsInTable(const LidarScan& scan, 
                         std::vector<Position>& pointsInTable,
                         const Position& robotPos) {
    pointsInTable.clear();
    pointsInTable.reserve(scan.size());
    
    const float res_rad = LIDAR_RESOLUTION_DEG * PI / 180.0f;
    const float cos_robot = std::cos(robotPos.angle_rad);
    const float sin_robot = std::sin(robotPos.angle_rad);
    const float cos_step = std::cos(res_rad);
    const float sin_step = std::sin(res_rad);
    
    float cos_angle = cos_robot;
    float sin_angle = sin_robot;
    
    for (size_t i = 0; i < scan.size(); ++i) {
        float distance_mm = scan[i];
        
        if (distance_mm > 0 && distance_mm < LIDAR_VALID_DISTANCE_MAX_MM) {
            float x = robotPos.x_mm + distance_mm * cos_angle;
            float y = robotPos.y_mm + distance_mm * sin_angle;
            
            // Test AABB (table)
            if (x >= 0 && x <= TABLE_WIDTH_MM && y >= 0 && y <= TABLE_HEIGHT_MM) {
                Position p;
                p.x_mm = x;
                p.y_mm = y;
                p.angle_rad = 0;
                pointsInTable.push_back(p);
            }
        }
        
        // Rotation incrémentale
        float tmp_cos = cos_angle * cos_step - sin_angle * sin_step;
        sin_angle = sin_angle * cos_step + cos_angle * sin_step;
        cos_angle = tmp_cos;
    }
}

// ============================================================================
// TRACKING DES PILIERS
// ============================================================================

float findPillarCenter(const LidarScan& scan, 
                       float estimatedAngle_rad, 
                       float estimatedDistance_mm) {
    if (estimatedDistance_mm <= 0.0f) {
        std::cerr << "⚠️ Distance invalide pour findPillarCenter\n";
        return estimatedAngle_rad;
    }
    
    // Calcul du span de recherche
    float angle_span_rad = std::atan2(PILLAR_SEARCH_RADIUS_MM, estimatedDistance_mm);
    float angle_span_deg = angle_span_rad * 180.0f / PI;
    float estimated_angle_deg = estimatedAngle_rad * 180.0f / PI;
    
    int start_idx = static_cast<int>(std::round((estimated_angle_deg - angle_span_deg) / LIDAR_RESOLUTION_DEG));
    int end_idx = static_cast<int>(std::round((estimated_angle_deg + angle_span_deg) / LIDAR_RESOLUTION_DEG));
    
    start_idx = std::max(0, start_idx);
    end_idx = std::min(LIDAR_NUM_ANGLES - 1, end_idx);
    
    int best_idx = -1;
    float best_distance = 100000.0f;
    
    for (int i = start_idx; i <= end_idx; ++i) {
        int idx = (i % LIDAR_NUM_ANGLES + LIDAR_NUM_ANGLES) % LIDAR_NUM_ANGLES;
        float d = scan[idx];
        
        // Filtre distance
        float dist_diff = std::fabs(d - estimatedDistance_mm);
        if (dist_diff > PILLAR_DISTANCE_FILTER_MM) continue;
        
        if (d > 0 && d < best_distance) {
            best_distance = d;
            best_idx = idx;
        }
    }
    
    if (best_idx >= 0) {
        return (best_idx * LIDAR_RESOLUTION_DEG) * PI / 180.0f;
    }
    
    return estimatedAngle_rad;
}

void trackPillars(const LidarScan& scan,
                  std::vector<TrackResult>& trackedPoints,
                  const Position& robotPos,
                  const std::vector<Position>& pillarPositions) {
    const float max_dist_norm = TRACKING_MAX_DISTANCE_MM;
    
    for (size_t p = 0; p < trackedPoints.size(); ++p) {
        float angle_ref = trackedPoints[p].angle_rad;
        float dist_ref = trackedPoints[p].distance_mm;
        
        float bestScore = 1000000.0f;
        TrackResult best = {0, 0, false};
        
        // Recherche du meilleur match dans le scan
        for (int i = 0; i < LIDAR_NUM_ANGLES; ++i) {
            float d = scan[i];
            if (d <= 0) continue;
            
            float angle_rad = (i * LIDAR_RESOLUTION_DEG) * PI / 180.0f;
            
            // Différence angulaire
            float da = std::fabs(angle_rad - angle_ref);
            da = std::min(da, 2 * PI - da); // wraparound
            
            // Différence distance
            float dd = std::fabs(d - dist_ref);
            
            // Score normalisé
            float norm_da = da / (2 * PI);
            float norm_dd = dd / max_dist_norm;
            float score = norm_da + norm_dd;
            
            if (score < bestScore) {
                bestScore = score;
                best = {angle_rad, d, true};
            }
        }
        
        if (!best.found) {
            trackedPoints[p].found = false;
            continue;
        }
        
        // Raffiner : trouver le centre exact du pilier
        best.angle_rad = findPillarCenter(scan, best.angle_rad, best.distance_mm);
        
        // Recalculer la distance au centre
        int idx = static_cast<int>(std::round((best.angle_rad * 180.0f / PI) / LIDAR_RESOLUTION_DEG));
        idx = (idx % LIDAR_NUM_ANGLES + LIDAR_NUM_ANGLES) % LIDAR_NUM_ANGLES;
        best.distance_mm = scan[idx];
        
        // Validation : vérifier que le pilier est proche de sa position attendue
        float adjusted_angle = best.angle_rad + robotPos.angle_rad;
        float x = robotPos.x_mm + (best.distance_mm + PILLAR_RADIUS_MM) * std::cos(adjusted_angle);
        float y = robotPos.y_mm + (best.distance_mm + PILLAR_RADIUS_MM) * std::sin(adjusted_angle);
        
        float dx = x - pillarPositions[p].x_mm;
        float dy = y - pillarPositions[p].y_mm;
        float error = std::sqrt(dx*dx + dy*dy);
        
        if (error > PILLAR_POSITION_TOLERANCE_MM) {
            std::cout << "⚠️ Pilier " << (p+1) << " hors tolérance (erreur=" << error << " mm)\n";
            best.found = false;
        }
        
        trackedPoints[p] = best;
    }
}

// ============================================================================
// CALCUL DE POSE (TRILATÉRATION)
// ============================================================================

Position polarToCartesian(const TrackResult& measurement) {
    Position p;
    p.x_mm = measurement.distance_mm * std::cos(measurement.angle_rad);
    p.y_mm = measurement.distance_mm * std::sin(measurement.angle_rad);
    p.angle_rad = 0;
    return p;
}

Position computeRobotPose(const std::vector<TrackResult>& measurements,
                          const std::vector<Position>& pillarPositions) {
    Position pose = {0, 0, 0};
    
    // Compter les piliers détectés
    int n = 0;
    std::vector<int> validIndices;
    for (size_t i = 0; i < measurements.size(); ++i) {
        if (measurements[i].found) {
            validIndices.push_back(i);
            n++;
        }
    }
    
    if (n < 2) {
        std::cerr << "❌ Pas assez de piliers détectés (" << n << "/3)\n";
        return pose;
    }
    
    // Trier pour avoir les 2 piliers les plus proches
    std::sort(validIndices.begin(), validIndices.end(), 
        [&measurements](int a, int b) {
            return measurements[a].distance_mm < measurements[b].distance_mm;
        });
    
    int idx1 = validIndices[0];
    int idx2 = validIndices[1];
    
    // === CAS 2 PILIERS ===
    Position r1 = polarToCartesian(measurements[idx1]);
    Position g1 = pillarPositions[idx1];
    Position r2 = polarToCartesian(measurements[idx2]);
    Position g2 = pillarPositions[idx2];
    
    double alpha_global = std::atan2(g2.y_mm - g1.y_mm, g2.x_mm - g1.x_mm);
    double alpha_local = std::atan2(r2.y_mm - r1.y_mm, r2.x_mm - r1.x_mm);
    
    pose.angle_rad = alpha_global - alpha_local;
    
    // Normaliser [-π, π]
    while (pose.angle_rad > PI) pose.angle_rad -= 2 * PI;
    while (pose.angle_rad < -PI) pose.angle_rad += 2 * PI;
    
    // Translation
    float cos_theta = std::cos(pose.angle_rad);
    float sin_theta = std::sin(pose.angle_rad);
    pose.x_mm = g1.x_mm - (cos_theta * r1.x_mm - sin_theta * r1.y_mm);
    pose.y_mm = g1.y_mm - (sin_theta * r1.x_mm + cos_theta * r1.y_mm);
    
    // === CAS 3 PILIERS : RAFFINEMENT ===
    if (n == 3) {
        double theta_sum = 0;
        int count = 0;
        
        for (size_t i = 0; i < validIndices.size(); ++i) {
            for (size_t j = i + 1; j < validIndices.size(); ++j) {
                int ia = validIndices[i];
                int ib = validIndices[j];
                
                Position gi = pillarPositions[ia];
                Position gj = pillarPositions[ib];
                Position ri = polarToCartesian(measurements[ia]);
                Position rj = polarToCartesian(measurements[ib]);
                
                double ag = std::atan2(gj.y_mm - gi.y_mm, gj.x_mm - gi.x_mm);
                double al = std::atan2(rj.y_mm - ri.y_mm, rj.x_mm - ri.x_mm);
                double dtheta = ag - al;
                
                // Normaliser
                while (dtheta > PI) dtheta -= 2 * PI;
                while (dtheta < -PI) dtheta += 2 * PI;
                
                theta_sum += dtheta;
                count++;
            }
        }
        
        pose.angle_rad = theta_sum / count;
        
        // Normaliser
        while (pose.angle_rad > PI) pose.angle_rad -= 2 * PI;
        while (pose.angle_rad < -PI) pose.angle_rad += 2 * PI;
        
        // Recalcul translation (moyenne)
        double tx = 0, ty = 0;
        cos_theta = std::cos(pose.angle_rad);
        sin_theta = std::sin(pose.angle_rad);
        
        for (int idx : validIndices) {
            Position gi = pillarPositions[idx];
            Position ri = polarToCartesian(measurements[idx]);
            
            double xi = gi.x_mm - (cos_theta * ri.x_mm - sin_theta * ri.y_mm);
            double yi = gi.y_mm - (sin_theta * ri.x_mm + cos_theta * ri.y_mm);
            
            tx += xi;
            ty += yi;
        }
        
        pose.x_mm = tx / n;
        pose.y_mm = ty / n;
    }
    
    std::cout << "📍 Pose calculée: x=" << pose.x_mm << " mm, y=" << pose.y_mm 
              << " mm, θ=" << (pose.angle_rad * 180.0f / PI) << "°\n";
    
    return pose;
}

// ============================================================================
// CONVERSIONS
// ============================================================================

ImuPose positionToImuPose(const Position& pos) {
    ImuPose p;
    p.x = pos.x_mm / 1000.0f; // mm -> m
    p.y = pos.y_mm / 1000.0f;
    p.h = pos.angle_rad;
    return p;
}

// ============================================================================
// INITIALISATION MATÉRIEL
// ============================================================================

bool initIMU(ImuOTOS& imu) {
    std::cout << "🔧 Initialisation IMU OTOS...\n";
    
    if (!imu.openBus()) {
        std::cerr << "❌ Erreur ouverture I2C\n";
        return false;
    }
    
    if (!imu.reset()) {
        std::cerr << "❌ Erreur reset IMU\n";
        imu.closeBus();
        return false;
    }
    
    std::cout << "⏳ Calibration IMU (" << IMU_CALIBRATION_TIME_MS << " ms)...\n";
    if (!imu.calibrate(255)) {
        std::cerr << "❌ Erreur calibration IMU\n";
        imu.closeBus();
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(IMU_CALIBRATION_TIME_MS));
    
    if (!imu.enableSignalProcessing(true, true, true, true)) {
        std::cerr << "❌ Erreur config signal IMU\n";
        imu.closeBus();
        return false;
    }
    
    std::cout << "✅ IMU initialisé avec succès\n";
    return true;
}

bool checkLidarHealth(ILidarDriver* drv) {
    sl_lidar_response_device_health_t healthinfo;
    sl_result op_result = drv->getHealth(healthinfo);
    
    if (!SL_IS_OK(op_result)) {
        std::cerr << "❌ Impossible de lire l'état du LIDAR: " << op_result << "\n";
        return false;
    }
    
    std::cout << "🩺 LIDAR status: " << healthinfo.status << "\n";
    
    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
        std::cerr << "❌ LIDAR erreur interne - Redémarrage requis\n";
        return false;
    }
    
    return true;
}

void printUsage(int argc, const char* argv[]) {
    std::cout << "Usage:\n"
              << "  Serial: " << argv[0] << " --channel --serial <port> [baudrate]\n"
              << "  UDP:    " << argv[0] << " --channel --udp <ip> [port]\n"
              << "\nBaudrates: A1(115200), A2M8(115200), A2M7/A2M12/A3/S1(256000), S2/S3(1000000)\n";
}

ILidarDriver* initLidar(int argc, const char* argv[], 
                        sl_lidar_response_device_info_t& devinfo,
                        int& opt_channel_type, 
                        IChannel*& _channel) {
    const char* opt_is_channel = NULL;
    const char* opt_channel = NULL;
    const char* opt_channel_param_first = NULL;
    sl_u32 opt_channel_param_second = 0;
    sl_u32 baudrateArray[2] = {115200, 256000};
    bool useArgcBaudrate = false;
    
    std::cout << "🚀 Ultra simple LIDAR - SDK v" << SL_LIDAR_SDK_VERSION << "\n";
    
    if (argc < 3) {
        printUsage(argc, argv);
        return nullptr;
    }
    
    opt_is_channel = argv[1];
    if (strcmp(opt_is_channel, "--channel") != 0) {
        printUsage(argc, argv);
        return nullptr;
    }
    
    opt_channel = argv[2];
    if (strcmp(opt_channel, "-s") == 0 || strcmp(opt_channel, "--serial") == 0) {
        if (argc < 4) {
            printUsage(argc, argv);
            return nullptr;
        }
        opt_channel_param_first = argv[3];
        std::cout << "📡 Port série: " << opt_channel_param_first << "\n";
        
        if (argc > 4) {
            opt_channel_param_second = strtoul(argv[4], NULL, 10);
            useArgcBaudrate = true;
            std::cout << "⚡ Baudrate: " << opt_channel_param_second << "\n";
        }
    } else if (strcmp(opt_channel, "-u") == 0 || strcmp(opt_channel, "--udp") == 0) {
        opt_channel_param_first = argv[3];
        if (argc > 4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
        opt_channel_type = CHANNEL_TYPE_UDP;
    } else {
        printUsage(argc, argv);
        return nullptr;
    }
    
    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT && !opt_channel_param_first) {
#ifdef _WIN32
        opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
        opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
        opt_channel_param_first = "/dev/ttyUSB0";
#endif
    }
    
    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "❌ Mémoire insuffisante\n";
        return nullptr;
    }
    
    bool connectSuccess = false;
    
    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (useArgcBaudrate) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK(drv->connect(_channel))) {
                if (SL_IS_OK(drv->getDeviceInfo(devinfo))) {
                    connectSuccess = true;
                } else {
                    delete drv;
                    drv = NULL;
                }
            }
        } else {
            for (size_t i = 0; i < 2; ++i) {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK(drv->connect(_channel))) {
                    if (SL_IS_OK(drv->getDeviceInfo(devinfo))) {
                        connectSuccess = true;
                        break;
                    } else {
                        delete drv;
                        drv = NULL;
                    }
                }
            }
        }
    } else if (opt_channel_type == CHANNEL_TYPE_UDP) {
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK(drv->connect(_channel))) {
            if (SL_IS_OK(drv->getDeviceInfo(devinfo))) {
                connectSuccess = true;
            } else {
                delete drv;
                drv = NULL;
            }
        }
    }
    
    if (!connectSuccess) {
        std::cerr << "❌ Connexion LIDAR échouée\n";
        if (drv) delete drv;
        return nullptr;
    }
    
    std::cout << "📟 LIDAR S/N: ";
    for (int i = 0; i < 16; ++i) {
        printf("%02X", devinfo.serialnum[i]);
    }
    std::cout << "\n🔧 Firmware: " << (devinfo.firmware_version >> 8) << "." 
              << (devinfo.firmware_version & 0xFF) << "\n";
    std::cout << "⚙️ Hardware Rev: " << (int)devinfo.hardware_version << "\n";
    
    if (!checkLidarHealth(drv)) {
        delete drv;
        return nullptr;
    }
    
    return drv;
}

// ============================================================================
// COMMUNICATION SOCKET
// ============================================================================

void initSocket(int& server_sock, int& client_sock, 
                struct sockaddr_un& addr, const char* socketPath) {
    unlink(socketPath);
    
    if (server_sock < 0) {
        std::cerr << "❌ Erreur création socket\n";
        exit(EXIT_FAILURE);
    }
    
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socketPath, sizeof(addr.sun_path) - 1);
    
    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "❌ Erreur bind socket\n";
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    
    if (listen(server_sock, 1) < 0) {
        std::cerr << "❌ Erreur listen socket\n";
        close(server_sock);
        exit(EXIT_FAILURE);
    }
    
    std::cout << "🔌 Serveur socket en attente sur " << socketPath << "\n";
}

void tryAcceptClient(int& client_sock, int server_sock) {
    if (client_sock < 0) {
        client_sock = accept(server_sock, NULL, NULL);
        if (client_sock < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "⚠️ Erreur accept: " << strerror(errno) << "\n";
            }
        } else {
            std::cout << "✅ Client Python connecté\n";
        }
    }
}
