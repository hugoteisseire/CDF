/**
 * @file main_refactored.cpp
 * @brief Programme principal refactorisé pour robot LIDAR + IMU
 * 
 * Architecture modulaire avec:
 * - Structures pour encapsuler les états
 * - Fonctions décomposées du mainLoop
 * - Utilisation de robot_config.h pour toutes les constantes
 * - Gestion d'erreurs améliorée
 * - Code testable et maintenable
 */

#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <thread>
#include <chrono>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>

#include "robot_config.h"
#include "lidar_lib_refactored.h"
#include "ImuOTOS.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;
using namespace std::chrono;

// ============================================================================
// STRUCTURES D'ÉTAT
// ============================================================================

/**
 * @brief État de la communication socket
 */
struct SocketState {
    int serverSock;
    int clientSock;
    struct sockaddr_un addr;
    const char* socketPath;
    
    SocketState(const char* path) 
        : serverSock(socket(AF_UNIX, SOCK_STREAM, 0))
        , clientSock(-1)
        , socketPath(path) {
        memset(&addr, 0, sizeof(addr));
    }
    
    ~SocketState() {
        if (clientSock >= 0) close(clientSock);
        if (serverSock >= 0) close(serverSock);
    }
};

/**
 * @brief État du robot (positions, vitesses, historique)
 */
struct RobotState {
    Position posRobot;      ///< Position actuelle du robot (fusionnée IMU+LIDAR)
    Position posIMU;        ///< Position IMU brute
    Position posLIDAR;      ///< Position LIDAR calculée
    ImuPose velocity;       ///< Vitesse du robot (m/s)
    
    std::deque<Position> posHistory;  ///< Historique pour lissage
    static const size_t HISTORY_SIZE = IMU_SMOOTHING_WINDOW_SIZE;
    
    void addToHistory(const Position& pos) {
        posHistory.push_back(pos);
        if (posHistory.size() > HISTORY_SIZE) {
            posHistory.pop_front();
        }
    }
    
    Position getSmoothedPosition() const {
        if (posHistory.empty()) return posLIDAR;
        
        float sum_x = 0, sum_y = 0, sum_angle = 0;
        for (const auto& p : posHistory) {
            sum_x += p.x_mm;
            sum_y += p.y_mm;
            sum_angle += p.angle_rad;
        }
        
        float n = static_cast<float>(posHistory.size());
        return {sum_x / n, sum_y / n, sum_angle / n};
    }
};

/**
 * @brief État de la localisation par piliers
 */
struct LocalizationState {
    std::vector<Position> pillarPositions;     ///< Positions de référence des piliers
    std::vector<TrackResult> trackedPoints;    ///< Points suivis dans le scan
    int consecutiveDetections;                  ///< Nombre de détections consécutives
    
    static const int DETECTION_THRESHOLD = 20;  ///< Seuil pour mise à jour IMU
    static const float SPEED_THRESHOLD;         ///< Vitesse max pour mise à jour (m/s)
    
    LocalizationState() : consecutiveDetections(0) {}
    
    void reset() { consecutiveDetections = 0; }
    void increment() { consecutiveDetections++; }
    bool shouldUpdateIMU() const { return consecutiveDetections >= DETECTION_THRESHOLD; }
    int countDetectedPillars() const {
        int count = 0;
        for (const auto& tp : trackedPoints) {
            if (tp.found) count++;
        }
        return count;
    }
};

const float LocalizationState::SPEED_THRESHOLD = 0.02f; // 2 cm/s

/**
 * @brief Configuration globale du système
 */
struct SystemConfig {
    Team team;
    bool ctrlCPressed;
    
    SystemConfig(Team t) : team(t), ctrlCPressed(false) {}
};

// ============================================================================
// GESTION SIGNAUX
// ============================================================================

SystemConfig* g_systemConfig = nullptr;

void signalHandler(int) {
    if (g_systemConfig) {
        g_systemConfig->ctrlCPressed = true;
        std::cout << "\n🛑 Arrêt demandé (Ctrl+C)...\n";
    }
}

// ============================================================================
// FONCTIONS DE COMMUNICATION
// ============================================================================

/**
 * @brief Envoie les données de localisation via socket
 * @param socketState État du socket
 * @param robotState État du robot
 * @param locState État de localisation
 */
void sendDataToClient(SocketState& socketState, 
                      const RobotState& robotState,
                      const LocalizationState& locState) {
    if (socketState.clientSock < 0) {
        tryAcceptClient(socketState.clientSock, socketState.serverSock);
        return;
    }
    
    // Format: [posIMU(3), posLIDAR(3), 3x piliers(angle, dist, found)]
    float data[18] = {
        robotState.posIMU.x_mm,
        robotState.posIMU.y_mm,
        robotState.posIMU.angle_rad,
        robotState.posLIDAR.x_mm,
        robotState.posLIDAR.y_mm,
        robotState.posLIDAR.angle_rad
    };
    
    for (size_t i = 0; i < 3 && i < locState.trackedPoints.size(); ++i) {
        data[6 + i*3 + 0] = locState.trackedPoints[i].angle_rad;
        data[6 + i*3 + 1] = locState.trackedPoints[i].distance_mm;
        data[6 + i*3 + 2] = locState.trackedPoints[i].found ? 1.0f : 0.0f;
    }
    
    ssize_t sent = write(socketState.clientSock, data, sizeof(data));
    if (sent < 0) {
        std::cerr << "⚠️ Erreur envoi socket, reconnexion...\n";
        close(socketState.clientSock);
        socketState.clientSock = -1;
    }
}

// ============================================================================
// FONCTIONS DE TRAITEMENT
// ============================================================================

/**
 * @brief Met à jour la position IMU du robot
 */
bool updateIMUPosition(ImuOTOS& imu, RobotState& robotState) {
    ImuPose pose;
    if (!imu.readPose(pose)) {
        std::cerr << "❌ Erreur lecture pose IMU\n";
        return false;
    }
    
    robotState.posIMU.x_mm = pose.x * 1000.0f;
    robotState.posIMU.y_mm = pose.y * 1000.0f;
    robotState.posIMU.angle_rad = pose.h;
    
    return true;
}

/**
 * @brief Met à jour la vitesse du robot
 */
bool updateVelocity(ImuOTOS& imu, RobotState& robotState) {
    if (!imu.readVelocity(robotState.velocity)) {
        std::cerr << "⚠️ Erreur lecture vitesse IMU\n";
        return false;
    }
    return true;
}

/**
 * @brief Traite un scan LIDAR et met à jour le tracking
 */
bool processScan(LidarScan& scan, 
                 ILidarDriver* drv,
                 RobotState& robotState,
                 LocalizationState& locState) {
    auto t_start = high_resolution_clock::now();
    
    if (!grabAndUpdateScan(scan, drv)) {
        std::cerr << "❌ Échec acquisition scan\n";
        return false;
    }
    
    // Tracking des piliers
    trackPillars(scan, locState.trackedPoints, robotState.posRobot, locState.pillarPositions);
    
    // Recalcul des piliers non détectés (position estimée)
    for (size_t i = 0; i < locState.trackedPoints.size(); ++i) {
        if (!locState.trackedPoints[i].found) {
            float dx = locState.pillarPositions[i].x_mm - robotState.posRobot.x_mm;
            float dy = locState.pillarPositions[i].y_mm - robotState.posRobot.y_mm;
            
            locState.trackedPoints[i].distance_mm = std::sqrt(dx*dx + dy*dy) - PILLAR_RADIUS_MM;
            locState.trackedPoints[i].angle_rad = std::atan2(dy, dx) - robotState.posRobot.angle_rad;
            
            // Normaliser [0, 2π]
            while (locState.trackedPoints[i].angle_rad < 0) 
                locState.trackedPoints[i].angle_rad += 2 * PI;
            while (locState.trackedPoints[i].angle_rad >= 2 * PI) 
                locState.trackedPoints[i].angle_rad -= 2 * PI;
        }
    }
    
    auto t_end = high_resolution_clock::now();
    auto duration_ms = duration_cast<milliseconds>(t_end - t_start).count();
    std::cout << "⏱️ Traitement scan: " << duration_ms << " ms\n";
    
    return true;
}

/**
 * @brief Calcule la pose LIDAR et met à jour l'état
 */
void computeAndUpdatePose(RobotState& robotState, LocalizationState& locState) {
    Position rawPose = computeRobotPose(locState.trackedPoints, locState.pillarPositions);
    
    robotState.addToHistory(rawPose);
    robotState.posLIDAR = robotState.getSmoothedPosition();
}

/**
 * @brief Décide s'il faut mettre à jour l'IMU avec la pose LIDAR
 */
bool shouldFuseWithIMU(const RobotState& robotState, const LocalizationState& locState) {
    int detected = locState.countDetectedPillars();
    
    if (detected < 2) return false;
    if (!locState.shouldUpdateIMU()) return false;
    
    float speedNorm = std::hypot(robotState.velocity.x, robotState.velocity.y);
    std::cout << "🚀 Vitesse: " << speedNorm << " m/s\n";
    
    return speedNorm < LocalizationState::SPEED_THRESHOLD;
}

/**
 * @brief Fusionne LIDAR et IMU
 */
void fusePoses(ImuOTOS& imu, RobotState& robotState, LocalizationState& locState) {
    int detected = locState.countDetectedPillars();
    
    if (detected >= 2) {
        locState.increment();
        
        if (shouldFuseWithIMU(robotState, locState)) {
            std::cout << "✅ Fusion LIDAR → IMU\n";
            robotState.posRobot = robotState.posLIDAR;
            imu.writePose(positionToImuPose(robotState.posRobot));
            locState.reset();
        }
    } else {
        locState.reset();
    }
}

/**
 * @brief Affiche l'état actuel du système
 */
void displayStatus(const RobotState& robotState, const LocalizationState& locState) {
    std::cout << "\n📊 === STATUS ===\n";
    
    // Position IMU
    std::cout << "📍 IMU: x=" << robotState.posIMU.x_mm << " mm, y=" << robotState.posIMU.y_mm 
              << " mm, θ=" << (robotState.posIMU.angle_rad * 180.0f / PI) << "°\n";
    
    // Position LIDAR
    std::cout << "📍 LIDAR: x=" << robotState.posLIDAR.x_mm << " mm, y=" << robotState.posLIDAR.y_mm 
              << " mm, θ=" << (robotState.posLIDAR.angle_rad * 180.0f / PI) << "°\n";
    
    // Delta
    std::cout << "Δ: dx=" << (robotState.posLIDAR.x_mm - robotState.posIMU.x_mm) << " mm"
              << ", dy=" << (robotState.posLIDAR.y_mm - robotState.posIMU.y_mm) << " mm"
              << ", dθ=" << ((robotState.posLIDAR.angle_rad - robotState.posIMU.angle_rad) * 180.0f / PI) << "°\n";
    
    // Piliers
    std::cout << "🎯 Piliers détectés: " << locState.countDetectedPillars() << "/3\n";
    for (size_t i = 0; i < locState.trackedPoints.size(); ++i) {
        const auto& tp = locState.trackedPoints[i];
        std::cout << "  P" << (i+1) << ": " << (tp.found ? "✓" : "✗")
                  << " angle=" << (tp.angle_rad * 180.0f / PI) << "°"
                  << ", dist=" << tp.distance_mm << " mm\n";
    }
    
    std::cout << "================\n\n";
}

// ============================================================================
// BOUCLE PRINCIPALE DÉCOMPOSÉE
// ============================================================================

/**
 * @brief Boucle principale du système
 */
void mainLoop(ImuOTOS& imu, 
              ILidarDriver* drv,
              SystemConfig& sysConfig,
              SocketState& socketState,
              RobotState& robotState,
              LocalizationState& locState) {
    
    LidarScan scan(LIDAR_NUM_ANGLES, 0.0f);
    
    std::cout << "🚀 Démarrage boucle principale...\n";
    std::cout << std::fixed << std::setprecision(2);
    
    // Démarrage LIDAR
    drv->setMotorSpeed();
    drv->startScan(0, 1);
    
    while (!sysConfig.ctrlCPressed) {
        // 1. Communication
        sendDataToClient(socketState, robotState, locState);
        
        // 2. Lecture IMU
        if (!updateIMUPosition(imu, robotState)) continue;
        updateVelocity(imu, robotState);
        robotState.posRobot = robotState.posIMU;
        
        // 3. Acquisition et traitement LIDAR
        if (!processScan(scan, drv, robotState, locState)) continue;
        
        // 4. Calcul pose LIDAR
        computeAndUpdatePose(robotState, locState);
        
        // 5. Fusion IMU/LIDAR
        fusePoses(imu, robotState, locState);
        
        // 6. Affichage
        displayStatus(robotState, locState);
        
        // Petit délai pour ne pas saturer le CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "🛑 Fin de la boucle principale\n";
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "🤖 === ROBOT LIDAR REFACTORISÉ ===\n\n";
    
    // 1. Configuration
    Team team = Team::TEAM0;
    if (argc > 1) {
        team = intToTeam(atoi(argv[1]));
    }
    
    std::cout << "🏁 " << getTeamName(team) << "\n";
    
    SystemConfig sysConfig(team);
    g_systemConfig = &sysConfig;
    signal(SIGINT, signalHandler);
    
    // 2. Initialisation Socket
    SocketState socketState(SOCKET_PATH);
    fcntl(socketState.serverSock, F_SETFL, O_NONBLOCK);
    initSocket(socketState.serverSock, socketState.clientSock, 
               socketState.addr, socketState.socketPath);
    
    // 3. Initialisation localisation
    LocalizationState locState;
    locState.pillarPositions = initPillars(team);
    
    // 4. État du robot
    RobotState robotState;
    robotState.posRobot = getRobotStartPosition(team);
    robotState.posIMU = robotState.posRobot;
    robotState.posLIDAR = robotState.posRobot;
    
    std::cout << "📍 Position initiale: x=" << robotState.posRobot.x_mm << " mm"
              << ", y=" << robotState.posRobot.y_mm << " mm"
              << ", θ=" << (robotState.posRobot.angle_rad * 180.0f / PI) << "°\n\n";
    
    // 5. Initialisation IMU
    ImuOTOS imu("/dev/i2c-1", 0x17);
    if (!initIMU(imu)) {
        std::cerr << "❌ Échec initialisation IMU\n";
        return 1;
    }
    
    imu.writePose(positionToImuPose(robotState.posRobot));
    locState.trackedPoints = initTrackedPoints(robotState.posRobot, locState.pillarPositions);
    
    // 6. Initialisation LIDAR
    const char* lidarArgv[] = {
        "./ultra_simple", "--channel", "--serial", "/dev/serial0", "256000"
    };
    int lidarArgc = 5;
    
    sl_lidar_response_device_info_t devinfo;
    int channelType = CHANNEL_TYPE_SERIALPORT;
    IChannel* channel = nullptr;
    
    ILidarDriver* drv = initLidar(lidarArgc, lidarArgv, devinfo, channelType, channel);
    if (!drv) {
        std::cerr << "❌ Échec initialisation LIDAR\n";
        return 1;
    }
    
    // 7. Boucle principale
    mainLoop(imu, drv, sysConfig, socketState, robotState, locState);
    
    // 8. Nettoyage
    std::cout << "🧹 Nettoyage...\n";
    drv->stop();
    delay(200);
    if (channelType == CHANNEL_TYPE_SERIALPORT) {
        drv->setMotorSpeed(0);
    }
    delete drv;
    
    std::cout << "👋 Programme terminé\n";
    return 0;
}
