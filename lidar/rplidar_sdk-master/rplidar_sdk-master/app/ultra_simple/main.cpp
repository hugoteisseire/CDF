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

// Chemin du socket UNIX pour communication inter-processus
const char *socket_path = "/tmp/robot.sock";
int server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
struct sockaddr_un addr;
int client_sock = -1;

int channel_pwm = 2;
int frequency_pwm = 20000; // Hz
RPI_PWM pwm_lidar;


// Flag d'arrêt via Ctrl+C
bool ctrl_c_pressed;
void ctrlc(int) { ctrl_c_pressed = true; }
using namespace std::chrono;

using json = nlohmann::json;

void afficherTousLesChamps(const json& json_data) {

    // Variables locales
    float x, y, angle;
    bool equipe;

    // Copie des valeurs depuis le JSON
    x = json_data["robot"]["x"];
    y = json_data["robot"]["y"];
    angle = json_data["robot"]["angle"];

    equipe = json_data["equipe"];;  // "1" -> true, "0" -> false

    // Affichage des valeurs
    std::cout << "Robot - x: " << x << ", y: " << y << ", angle: " << angle << std::endl;
    std::cout << "Équipe: " << (equipe ? "true" : "false") << std::endl;
}

void lireFichierJSON() {

    std::string jsonpath = "/home/raspi/Desktop/CDF/init_prog/init_data.json";
    std::ifstream json_file(jsonpath);

    if (!json_file.is_open()) {
        std::cerr << "Erreur : impossible d'ouvrir le fichier JSON : " << jsonpath << std::endl;
        return;
    }

    json json_data;
    try {
        json_file >> json_data;
    } catch (const json::parse_error& e) {
        std::cerr << "Erreur de parsing JSON : " << e.what() << std::endl;
        json_file.close();
        return;
    }

    json_file.close();
    afficherTousLesChamps(json_data);
}

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

// --- Boucle principale du robot ---
void mainLoop(ImuOTOS& imu, ILidarDriver* drv, position& posrobot, position& posImu, position& poslidar, int& client_sock, int& server_sock) {
    
    std::vector<float> scan(NUM_ANGLES, -1.0f);
    std::vector<TrackResult> trackedPoints;

    // historique des dernières positions lidar (pour lissage)
    std::deque<position> pos_history;
    const size_t SMOOTH_N = 5; // nombre d'échantillons pour le lissage

    inittrackedpoints(trackedPoints, posrobot, 3);
    ImuPose p_init_imu = position_to_imu(posrobot);
    imu.writePose(p_init_imu);

    signal(SIGINT, ctrlc);
    drv->setMotorSpeed();
    drv->startScan(0,1);

    std::cout << "Début boucle principale\n";
    std::cout << std::fixed << std::setprecision(5);
    ImuPose pose;
    ImuPose speed;
    try_accept_client(client_sock, server_sock);
    int count=0; // compteur de mesures consécutives avec au moins 3 piliers détectés
    const int COUNT_THRESHOLD = 20;        // nombre d'itérations consécutives requises
    const float SPEED_THRESHOLD = 0.02f;   // m/s, seuil pour considérer le robot "lent"
    while (!ctrl_c_pressed) {
        // Communication socket : envoi de la position IMU
        if (client_sock >= 0) {
            // je veut rajoute la pose lidar ainsi que trackedPoints
            float data[18] = {posImu.x, posImu.y, posImu.angle, poslidar.x, poslidar.y, poslidar.angle,trackedPoints[0].angle, trackedPoints[0].distance, trackedPoints[0].found,
                            trackedPoints[1].angle, trackedPoints[1].distance, trackedPoints[1].found,
                            trackedPoints[2].angle, trackedPoints[2].distance, trackedPoints[2].found};
            write(client_sock, data, sizeof(data));
        } else {
            try_accept_client(client_sock, server_sock);
        }

        // Lecture de la pose IMU
        if (imu.readPose(pose)) {
            posImu.x = pose.x * 1000.0f;
            posImu.y = pose.y * 1000.0f;
            posImu.angle = pose.h;
        } else {
            std::cerr << "Erreur lecture pose\n";
        }
        posrobot = posImu;
        auto t3 = high_resolution_clock::now();
        // Acquisition et traitement du scan LIDAR
        grabAndUpdateScan(scan, drv);
        auto t4 = high_resolution_clock::now(); 
        auto duration = duration_cast<milliseconds>(t4 - t3).count();
        std::cout << "Durée acquisition et traitement du scan LIDAR: " << duration << " ms\n";
        std::vector<position> points_in_table;
        point_in_table2(scan, points_in_table, RESOLUTION, posrobot);
        std::cout << "Nombre de points dans la table: " << points_in_table.size() << std::endl;
        trackPoints(scan, trackedPoints, RESOLUTION, posrobot);

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
            std::cout << "\n=> Pilier " << (i+1)
                << ": angle=" << trackedPoints[i].angle * 180.0f / M_PI
                << "°, distance=" << trackedPoints[i].distance
                << " mm, found=" << trackedPoints[i].found
                << " scan=" << scan[(int)(trackedPoints[i].angle * 180.0f / M_PI / RESOLUTION)]
                << " mm";
        }

        poslidar = computePose(trackedPoints.data());
        // je veut lisser la pos sur les 5 dernieres positions
        pos_history.push_back(poslidar);
        if (pos_history.size() > SMOOTH_N) pos_history.pop_front();

        // calculer la moyenne des dernières positions et l'utiliser comme poslidar lissée
        position smoothed = average_position(pos_history);
        poslidar = smoothed;

        if (pillardetected >= 2) {
            count++;
            // si on a COUNT_THRESHOLD mesures consécutives avec >= 2 piliers détectés,
            // on met à jour l'IMU (mais seulement si le robot est suffisamment lent)
            if (count >= COUNT_THRESHOLD) {
                // lire la vitesse (vérifier que la lecture réussit si la méthode retourne bool)
                if (imu.readVelocity(speed)) { // adapter si readVelocity a une autre signature
                    float speed_norm = std::hypot(speed.x, speed.y); // norme (m/s)
                    std::cout << " => Vitesse IMU: vx=" << speed.x << " m/s, vy=" << speed.y
                        << " m/s, norme=" << speed_norm << " m/s\n";
                    if (speed_norm < SPEED_THRESHOLD) {
                        // on met à jour la pose IMU avec la pose LIDAR lissée
                        posrobot = poslidar;
                        ImuPose p = position_to_imu(posrobot);
                        imu.writePose(p);

                    }
                } else {
                    std::cerr << "Warning: impossible de lire la vitesse IMU\n";
                }
                count = 0;
            }
        } else {
            count = 0;
        }
        imu.readVelocity(speed);
        float speed_norm = std::hypot(speed.x, speed.y); // norme (m/s)
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
}

// --- Nettoyage des ressources ---
void cleanup(ILidarDriver* drv, int opt_channel_type) {
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
int main() {
    lireFichierJSON() ;
    // Initialisation du socket serveur
    fcntl(server_sock, F_SETFL, O_NONBLOCK);
    init_socket(server_sock, client_sock, addr ,socket_path);

    pwm_lidar.start(channel_pwm, frequency_pwm);
    pwm_lidar.setDutyCycle(50);


    // Initialisation des positions
    position poslidar, posImu, posrobot;
    position p_init(2200, 1000, 0);
    posrobot = p_init;

    // Initialisation IMU
    ImuOTOS imu("/dev/i2c-1", 0x17);
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

    // Boucle principale
    mainLoop(imu, drv, posrobot, posImu, poslidar, client_sock, server_sock);

    // Nettoyage
    cleanup(drv, opt_channel_type);

    return 0;
}


