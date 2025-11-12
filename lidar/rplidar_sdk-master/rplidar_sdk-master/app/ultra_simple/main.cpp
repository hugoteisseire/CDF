/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *  Copyright (c) 2009 - 2020 RoboPeak/Slamtec
 *  GPL v3
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <fcntl.h>
#include "ImuOTOS.h"
#include "lidar_lib.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <deque>   // ajouté pour historiser les positions
#include <chrono>  
#include "rpi_pwm.h"
#include <stdint.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

// === SECTION : Définition des variables globales et structures ===

// === Multi-threading - Variables partagées ===
std::mutex scan_mutex;                          // Protection du scan partagé
std::vector<float> shared_scan(NUM_ANGLES, -1.0f); // Dernier scan LIDAR acquis
std::atomic<bool> new_scan_available{false};    // Indicateur de nouveau scan

std::mutex position_mutex;                      // Protection des positions
position shared_posImu(0, 0, 0);               // Position IMU partagée
position shared_poslidar(0, 0, 0);             // Position LIDAR partagée
position shared_posennemy(0, 0, 0);            // Position adversaire partagée
std::vector<TrackResult> shared_trackedPoints; // Piliers trackés
std::atomic<int> shared_pillardetected{0};     // Nombre de piliers détectés

std::atomic<bool> running{true};                // Flag pour arrêter les threads
const char* shared_current_state = "init\n";   // État courant (atomic pointer)

// Chemin du socket UNIX pour communication inter-processus
const char *socket_path = "/tmp/robot.sock";
int client_sock = -1;  // Maintenant on est CLIENT (pas serveur)
const int MAX_SOCKET_RETRIES = 10;

int channel_pwm = 2;
int frequency_pwm = 20000; // Hz
RPI_PWM pwm_lidar;


// Flag d'arrêt via Ctrl+C
bool ctrl_c_pressed = false;
void ctrlc(int) { 
    ctrl_c_pressed = true; 
    running = false;  // Arrête aussi les threads
}
using namespace std::chrono;

using json = nlohmann::json;

// Flag pour activer/désactiver les prints de debug
bool DEBUG_PRINT = false;

// helper : moyenne d'un deque<position>
static position average_position(const std::deque<position>& dq) {
    position sum;
    sum.x = 0.0f; sum.y = 0.0f; sum.angle = 0.0f;
    for (const auto &p : dq) {
        sum.x += p.x;
        sum.y += p.y;
        sum.angle += p.angle;
    }
    if (dq.empty()) return sum;
    float n = static_cast<float>(dq.size());
    sum.x /= n;
    sum.y /= n;
    sum.angle /= n;
    return sum;
}

// ============================================================================
// THREAD 1 : Acquisition LIDAR (bloquante) - Tourne en continu
// ============================================================================
void lidar_acquisition_thread(ILidarDriver* drv) {
    if (DEBUG_PRINT) {
        std::cout << "🔄 Thread LIDAR démarré\n";
    }
    
    drv->setMotorSpeed();
    drv->startScan(0, 1);
    
    // Warmup: attendre 5s pour remplir le buffer scan
    auto warmup_start = high_resolution_clock::now();
    const int WARMUP_SECONDS = 5;
    
    while (running) {
        auto now = high_resolution_clock::now();
        auto warm_elapsed = duration_cast<seconds>(now - warmup_start).count();
        bool in_warmup = warm_elapsed < WARMUP_SECONDS;
        
        // Acquisition LIDAR (BLOQUANT)
        std::vector<float> local_scan(NUM_ANGLES, -1.0f);
        bool grab_ok = grabAndUpdateScan(local_scan, drv);
        
        // Mise à jour du scan partagé
        {
            std::lock_guard<std::mutex> lock(scan_mutex);
            shared_scan = local_scan;
            new_scan_available = true;
        }
        
        // Mise à jour de l'état
        if (in_warmup) {
            shared_current_state = "init\n";
        } else if (!grab_ok) {
            shared_current_state = "error\n";
        }
        // Note: "ready" ou "lost" sera déterminé par le thread IMU
    }
    
    if (DEBUG_PRINT) {
        std::cout << "🛑 Thread LIDAR arrêté\n";
    }
}

// ============================================================================
// THREAD 2 : IMU + Calculs + Publication (50Hz)
// ============================================================================
void imu_compute_publish_thread(ImuOTOS& imu, position p_init) {
    if (DEBUG_PRINT) {
        std::cout << "🔄 Thread IMU+Publication démarré\n";
    }
    
    // Variables locales du thread
    position posrobot = p_init;
    position posImu = p_init;
    position poslidar = p_init;
    position posennemy(0, 0, 0);
    
    std::vector<TrackResult> trackedPoints;
    std::deque<position> pos_history;
    const size_t SMOOTH_N = 5;
    
    inittrackedpoints(trackedPoints, posrobot, 3);
    ImuPose p_init_imu = position_to_imu(posrobot);
    imu.writePose(p_init_imu);
    
    ImuPose pose;
    ImuPose speed;
    bool opp_has_prev = false;
    
    // Compteur pour la condition de mise à jour IMU
    int consec_detect_count = 0;
    const int COUNT_THRESHOLD = 20;
    const float SPEED_THRESHOLD = 0.02f;
    
    // États LIDAR
    const char *STATE_INIT   = "init\n";
    const char *STATE_READY  = "ready\n";
    const char *STATE_LOST   = "lost\n";
    const char *STATE_ERROR  = "error\n";
    const char *current_state = STATE_INIT;
    
    const auto loop_period = std::chrono::milliseconds(20); // 50Hz
    auto next_loop = std::chrono::steady_clock::now();
    
    while (running) {
        auto loop_start = std::chrono::steady_clock::now();
        
        // 1) Lecture de la pose IMU (rapide, non-bloquant)
        if (imu.readPose(pose)) {
            posImu.x = pose.x * 1000.0f;
            posImu.y = pose.y * 1000.0f;
            posImu.angle = pose.h;
        } else {
            std::cerr << "Erreur lecture pose IMU\n";
        }
        posrobot = posImu;
        
        // 2) Copie du dernier scan LIDAR disponible
        std::vector<float> local_scan;
        bool scan_is_available = false;
        {
            std::lock_guard<std::mutex> lock(scan_mutex);
            if (new_scan_available) {
                local_scan = shared_scan;
                scan_is_available = true;
            }
        }
        
        // 3) Traitement LIDAR si disponible
        if (scan_is_available) {
            auto t3 = high_resolution_clock::now();
            
            // 3a) Points dans la table
            std::vector<position> points_in_table;
            point_in_table2(local_scan, points_in_table, RESOLUTION, posrobot);
            if (DEBUG_PRINT) {
                std::cout << "Nombre de points dans la table: " << points_in_table.size() << std::endl;
            }
            
            // 3b) Suivi des piliers
            trackPoints(local_scan, trackedPoints, RESOLUTION, posrobot);
            
            int pillardetected = 0;
            for (int i = 0; i < 3; i++) {
                if (!trackedPoints[i].found) {
                    float dx = pillars[i].x - posrobot.x;
                    float dy = pillars[i].y - posrobot.y;
                    trackedPoints[i].distance = sqrt(dx * dx + dy * dy) - RAYON_PILIER;
                    trackedPoints[i].angle = atan2(dy, dx) - posrobot.angle;
                    if (trackedPoints[i].angle < 0) trackedPoints[i].angle += 2 * M_PI;
                    if (trackedPoints[i].angle >= 2 * M_PI) trackedPoints[i].angle -= 2 * M_PI;
                } else {
                    pillardetected++;
                }
                if (DEBUG_PRINT) {
                    std::cout << "\n=> Pilier " << (i+1)
                        << ": angle=" << trackedPoints[i].angle * 180.0f / M_PI
                        << "°, distance=" << trackedPoints[i].distance
                        << " mm, found=" << trackedPoints[i].found
                        << " scan=" << local_scan[(int)(trackedPoints[i].angle * 180.0f / M_PI / RESOLUTION)]
                        << " mm";
                }
            }
            
            // 3c) Pose robot via piliers + lissage
            poslidar = computePose(trackedPoints.data());
            pos_history.push_back(poslidar);
            if (pos_history.size() > SMOOTH_N) pos_history.pop_front();
            position smoothed = average_position(pos_history);
            poslidar = smoothed;
            
            // 3d) Mise à jour IMU si assez de détections et robot lent
            if (pillardetected >= 2) {
                consec_detect_count++;
                if (consec_detect_count >= COUNT_THRESHOLD) {
                    if (imu.readVelocity(speed)) {
                        float speed_norm = std::hypot(speed.x, speed.y);
                        if (DEBUG_PRINT) {
                            std::cout << " => Vitesse IMU: vx=" << speed.x << " m/s, vy=" << speed.y
                                << " m/s, norme=" << speed_norm << " m/s\n";
                        }
                        if (speed_norm < SPEED_THRESHOLD) {
                            posrobot = poslidar;
                            ImuPose p = position_to_imu(posrobot);
                            imu.writePose(p);
                        }
                    } else {
                        std::cerr << "Warning: impossible de lire la vitesse IMU\n";
                    }
                    consec_detect_count = 0;
                }
            } else {
                consec_detect_count = 0;
            }
            
            // 3e) Détection robot adverse (cluster)
            position opp_out;
            bool opp_ok = false;
            if (opp_has_prev) {
                opp_ok = detectOpponentRobotWithPrior(points_in_table, posennemy, opp_out);
            } else {
                opp_ok = detectOpponentRobot(points_in_table, opp_out);
            }
            if (opp_ok) { posennemy = opp_out; opp_has_prev = true; }
            
            // 3f) État
            current_state = (pillardetected >= 2 ? STATE_READY : STATE_LOST);
            shared_current_state = current_state;
            
            auto t4 = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(t4 - t3).count();
            if (DEBUG_PRINT) {
                std::cout << "Durée traitement: " << duration << " ms\n";
            }
        }
        
        // 4) Logs vitesse (facultatif)
        if (DEBUG_PRINT) {
            imu.readVelocity(speed);
            float speed_norm = std::hypot(speed.x, speed.y);
            std::cout << " => Vitesse IMU: vx=" << speed.x << " m/s, vy=" << speed.y
                        << " m/s, norme=" << speed_norm << " m/s\n";
            std::cout << "\n=> Pose calculée: x=" << poslidar.x << " mm, y=" << poslidar.y
                << " mm, angle=" << poslidar.angle * 180.0f / M_PI << "°\n";
            std::cout << "=> Pose IMU: x=" << posImu.x << " mm, y=" << posImu.y
                << " mm, angle=" << posImu.angle * 180.0f / M_PI << "°\n";
            std::cout << "=> delta position: dx=" << (poslidar.x - posImu.x)
                << " mm, dy=" << (poslidar.y - posImu.y)
                << " mm, dangle=" << (poslidar.angle - posImu.angle) * 180.0f / M_PI << "°\n";
        }
        
        // 5) Mise à jour des positions partagées (POINT CRITIQUE DE SYNCHRONISATION)
        {
            std::lock_guard<std::mutex> lock(position_mutex);
            shared_posImu = posImu;
            shared_poslidar = poslidar;
            shared_posennemy = posennemy;
            shared_trackedPoints = trackedPoints;
            shared_pillardetected = static_cast<int>(std::count_if(
                trackedPoints.begin(), trackedPoints.end(),
                [](const TrackResult& t) { return t.found; }
            ));
        }
        
        // 6) Publication socket
        if (client_sock >= 0) {
            // Copie locale pour minimiser le temps de lock
            position pos_imu_copy, pos_lidar_copy, pos_enemy_copy;
            std::vector<TrackResult> tracked_copy;
            {
                std::lock_guard<std::mutex> lock(position_mutex);
                pos_imu_copy = shared_posImu;
                pos_lidar_copy = shared_poslidar;
                pos_enemy_copy = shared_posennemy;
                tracked_copy = shared_trackedPoints;
            }
            
            // Envoi état
            ssize_t sent = write(client_sock, shared_current_state, strlen(shared_current_state));
            if (sent < 0) { 
                perror("Erreur envoi état"); 
                running = false;
                break;
            }
            
            // Envoi données format 20 floats
            float data[20] = {
                pos_imu_copy.x, pos_imu_copy.y, pos_imu_copy.angle,
                pos_lidar_copy.x, pos_lidar_copy.y, pos_lidar_copy.angle,
                pos_enemy_copy.x, pos_enemy_copy.y,
                tracked_copy.size() > 0 ? tracked_copy[0].angle : 0.0f,
                tracked_copy.size() > 0 ? tracked_copy[0].distance : 0.0f,
                tracked_copy.size() > 0 ? (tracked_copy[0].found ? 1.0f : 0.0f) : 0.0f,
                tracked_copy.size() > 1 ? tracked_copy[1].angle : 0.0f,
                tracked_copy.size() > 1 ? tracked_copy[1].distance : 0.0f,
                tracked_copy.size() > 1 ? (tracked_copy[1].found ? 1.0f : 0.0f) : 0.0f,
                tracked_copy.size() > 2 ? tracked_copy[2].angle : 0.0f,
                tracked_copy.size() > 2 ? tracked_copy[2].distance : 0.0f,
                tracked_copy.size() > 2 ? (tracked_copy[2].found ? 1.0f : 0.0f) : 0.0f,
                0.0f, 0.0f, 0.0f
            };
            sent = write(client_sock, data, sizeof(data));
            if (sent < 0) {
                perror("Erreur envoi données");
                running = false;
                break;
            }
        }
        
        // 7) Maintenir 50Hz (20ms par itération)
        next_loop += loop_period;
        std::this_thread::sleep_until(next_loop);
    }
    
    if (DEBUG_PRINT) {
        std::cout << "🛑 Thread IMU+Publication arrêté\n";
    }
}

// --- Nettoyage des ressources ---
void cleanup(ILidarDriver* drv, int opt_channel_type, int client_sock) {
    // Fermer le socket
    if (client_sock >= 0) {
        close(client_sock);
        if (DEBUG_PRINT) {
            printf("Socket fermé\n");
        }
    }
    
    drv->stop();
    delay(200);
    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    if(drv) {
        delete drv;
        drv = NULL;
    }
}

// --- Fonction principale refactorisée ---
int main(int argc1, char *argv1[]) {
    int team=0;
    if (argc1 > 1) {
        team = atoi(argv1[1]);  // Récupère 0, 1 ou 3
        if (DEBUG_PRINT) {
            printf("Équipe: %d\n", team);
        }
    }

    
    // Argument optionnel pour forcer le debug: ./ultra_simple <team> debug
    if (argc1 > 2 && strcmp(argv1[2], "debug") == 0) {
        DEBUG_PRINT = true;
    }
    
    // Initialisation des piliers selon l'équipe
    initPillars(team);
    
    // Position initiale selon l'équipe
    // Équipe 0: côté droit (x=2200), Équipe 1: côté gauche (x=800), Debug: centre (x=1500)
    position p_init;
    if (team == 3) {
        p_init = position(1500, 1000, M_PI/2);  // Équipe 1 (gauche)
        if (DEBUG_PRINT) {
            std::cout << "🐛 Mode DEBUG: position centrale\n";
        }
    } else if (team == 1) {
        p_init = position(1500, 1000, M_PI/2);  // Équipe 1 (gauche)
    } else {
        p_init = position(2200, 1000, 0);  // Équipe 0 (droite)
    }
    
    // === CONNEXION SOCKET CLIENT (avec retry) ===
    if (DEBUG_PRINT) {
        printf("🔌 Connexion au serveur Python sur %s...\n", socket_path);
    }
    
    client_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (client_sock < 0) {
        perror("Erreur création socket");
        return 1;
    }
    
    struct sockaddr_un server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, socket_path, sizeof(server_addr.sun_path) - 1);
    
    // Tentatives de connexion avec retry
    int retry = 0;
    while (retry < MAX_SOCKET_RETRIES) {
        if (connect(client_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) == 0) {
            if (DEBUG_PRINT) {
                printf("✅ Connecté au serveur Python!\n");
            }
            break;
        }
        
        retry++;
        if (DEBUG_PRINT) {
            printf("⏳ Tentative %d/%d...\n", retry, MAX_SOCKET_RETRIES);
        }
        sleep(1);
    }
    
    if (retry >= MAX_SOCKET_RETRIES) {
        fprintf(stderr, "❌ Impossible de se connecter au serveur après %d tentatives\n", MAX_SOCKET_RETRIES);
        close(client_sock);
        client_sock = -1;
        // On continue quand même (mode dégradé sans socket)
    }
    
    // === FIN CONNEXION SOCKET ===

    pwm_lidar.start(channel_pwm, frequency_pwm);
    pwm_lidar.setDutyCycle(60);


    // Initialisation des positions
    position poslidar, posImu, posrobot,posennemy;
    posennemy = position(0, 0, 0);
    posrobot = p_init;
    
 
    if (DEBUG_PRINT) {
        std::cout << "Position initiale robot: x=" << p_init.x << " mm, y=" << p_init.y 
                  << " mm, angle=" << (p_init.angle * 180.0f / PI) << "°\n";
    }

    // Initialisation IMU
    ImuOTOS imu("/dev/i2c-4", 0x17);
    if (!initIMU(imu)) return 1;

    // Initialisation des arguments LIDAR
    int argc = 5;
    const char *argv[] = {
        "./ultra_simple", "--channel", "--serial", "/dev/serial0", "256000"
    };

    // Initialisation du driver LIDAR
    sl_lidar_response_device_info_t devinfo;
    int opt_channel_type = CHANNEL_TYPE_SERIALPORT;
    IChannel* _channel = nullptr;
    ILidarDriver* drv = initLidar(argc, argv, devinfo, opt_channel_type, _channel);
    if (!drv) return -1;

    // === LANCEMENT DES THREADS ===
    signal(SIGINT, ctrlc);
    
    if (DEBUG_PRINT) {
        std::cout << "🚀 Démarrage du système multi-thread...\n";
    }
    
    // Thread 1 : Acquisition LIDAR (bloquante)
    std::thread lidar_thread(lidar_acquisition_thread, drv);
    
    // Thread 2 : IMU + Calculs + Publication (50Hz)
    std::thread imu_pub_thread(imu_compute_publish_thread, std::ref(imu), p_init);
    
    // Attendre l'arrêt (Ctrl+C)
    while (running && !ctrl_c_pressed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (DEBUG_PRINT) {
        std::cout << "\n⏳ Arrêt des threads...\n";
    }
    
    // Arrêt propre
    running = false;
    
    // Attendre la fin des threads
    if (lidar_thread.joinable()) {
        lidar_thread.join();
    }
    if (imu_pub_thread.joinable()) {
        imu_pub_thread.join();
    }
    
    if (DEBUG_PRINT) {
        std::cout << "✅ Threads arrêtés\n";
    }

    // Nettoyage
    cleanup(drv, opt_channel_type, client_sock);

    return 0;
}


