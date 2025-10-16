#include "lidar_lib.h"
#include "ImuOTOS.h"
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <iostream>
#include <thread>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace sl;
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
/*
const std::vector<position> pillars = {
    {3094, 50, 0},
    {3094, 1950, 0},
    {-94, 1000, 0}
};*/
const std::vector<position> pillars = {
    {2950, 50, 0},
    {2950, 1950, 0},
    {50, 1000, 0}
};



ImuPose position_to_imu(const position& pos) {
    ImuPose p;
    p.x = pos.x / 1000.0f; // en m
    p.y = pos.y / 1000.0f; // en m
    p.h = pos.angle; // en rad
    return p;
}
// Convertit une mesure (distance, angle) en coordonnées relatives
// Conversion polaire -> cartésien dans le repère mathématique direct
position relativeCoords(TrackResult m) {
    position p;
    p.x = m.distance * cos(m.angle); // X+ = 0 rad
    p.y = m.distance * sin(m.angle); // Y+ = +pi/2
    return p;
}

// Résout la pose avec 2 ou 3 piliers
position computePose(TrackResult *meas) {
    position pose = {0, 0, 0};
  
    int n = 0;
    float d[3] = {10000, 10000, 10000};
    int idx[3] = {0, 1, 2};
    // Récupère les distances des piliers détectés
    for (int i = 0; i < 3; i++) {
        if (meas[i].found) {
            d[i] = meas[i].distance+ RAYON_PILIER;
            n++;
        }
    }
    if (n < 2) return pose; // Pas assez de piliers

    // Trie les indices des distances croissantes (sélectionne les 2 plus proches)
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (d[idx[j]] < d[idx[i]]) {
                int tmp = idx[i];
                idx[i] = idx[j];
                idx[j] = tmp;
            }
        }
    }
    int min1 = idx[0], min2 = idx[1];

    // --- 1) Cas 2 piliers ---
    position r1 = relativeCoords(meas[min1]);
    position g1 = pillars[min1];
    position r2 = relativeCoords(meas[min2]);
    position g2 = pillars[min2];


    // Calcul des angles dans le repère mathématique direct
    double alpha_global = atan2(g2.y - g1.y, g2.x - g1.x); // Y, X
    double alpha_local  = atan2(r2.y - r1.y, r2.x - r1.x); // Y, X

    pose.angle = alpha_global - alpha_local;
    // Normalise l'angle dans [-pi, pi]
    while (pose.angle > PI) pose.angle -= 2 * PI;
    while (pose.angle < -PI) pose.angle += 2 * PI;

    // Translation en utilisant le 1er point
    // Application de la rotation et translation dans le repère direct
    pose.x = g1.x - (cos(pose.angle) * r1.x - sin(pose.angle) * r1.y);
    pose.y = g1.y - (sin(pose.angle) * r1.x + cos(pose.angle) * r1.y);

    // --- 2) Si 3 piliers, on peut raffiner ---
    if (n == 3) {
        double theta_sum = 0;
        int count = 0;
        // Moyenne des différences d'angles entre chaque paire de piliers
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                position gi = pillars[i];
                position gj = pillars[j];
                position ri = relativeCoords(meas[i]);
                position rj = relativeCoords(meas[j]);

                double ag = atan2(gj.y - gi.y, gj.x - gi.x); // Y, X
                double al = atan2(rj.y - ri.y, rj.x - ri.x); // Y, X
                double dtheta = ag - al;
                // Normalise l'angle
                while (dtheta > PI) dtheta -= 2 * PI;
                while (dtheta < -PI) dtheta += 2 * PI;
                theta_sum += dtheta;
                count++;
            }
        }
        pose.angle = theta_sum / count;
        // Normalise l'angle
        while (pose.angle > PI) pose.angle -= 2 * PI;
        while (pose.angle < -PI) pose.angle += 2 * PI;

        // Recalcule translation en moindres carrés (simple moyenne)
        double tx = 0, ty = 0;
        for (int i = 0; i < 3; i++) {
            position gi = pillars[i];
            position ri = relativeCoords(meas[i]);
            double xi = gi.x - (cos(pose.angle) * ri.x - sin(pose.angle) * ri.y);
            double yi = gi.y - (sin(pose.angle) * ri.x + cos(pose.angle) * ri.y);
            tx += xi; ty += yi;
        }
        pose.x = tx / 3.0;
        pose.y = ty / 3.0;
    }

    return pose;
}


void trackPoints(const std::vector<float>& scan,
                 std::vector<TrackResult>& trackedPoints,
                 float resolution ,
                position posrobot) // Traque tous les piliers
{
    float max_dist = 2000.0f;
    int numAngles = scan.size();
    for (size_t p = 0; p < trackedPoints.size(); ++p) {
        float angle_ref = trackedPoints[p].angle; // radians
        float dist_ref = trackedPoints[p].distance;
        float bestScore = 1000000;
        TrackResult best = {0, 0, false};
        for (int i = 0; i < numAngles; i++) {
            float d = scan[i];
            if (d <= 0) continue;
            float angle_deg = i * resolution; // indexation en degrés
            float angle = angle_deg * PI / 180.0f; // conversion en radians
            float da = fabs(angle - angle_ref);
            da = fmin(da, 2 * PI - da);
            float dd = fabs(d - dist_ref);
            float normalized_da = da / (2 * PI);
            float normalized_dd = dd / max_dist;
            float score = normalized_da + normalized_dd;
            if (score < bestScore) {
                bestScore = score;
                best = {angle, d, true};
            }
        }
        //printf(best.found ? "\nPilier %zu trouvé à (angle: %.2f rad, distance: %.2f mm)\n" : "Pilier %zu non trouvé\n", p, best.angle, best.distance);
        best.angle = findcenter(scan, best.angle, best.distance);

        best.distance= scan[(int)(best.angle * 180.0f / PI / resolution)];
        //printf(best.found ? "2/Pilier %zu trouvé à (angle: %.2f rad, distance: %.2f mm)\n" : "Pilier %zu non trouvé\n\n\n", p, best.angle, best.distance);

        // Calculer la position du point suivi en coordonnées cartésiennes
        float adjusted_angle_rad = best.angle + (posrobot.angle); // tout en radians
        float x = posrobot.x + (best.distance + RAYON_PILIER) * cos(adjusted_angle_rad);
        float y = posrobot.y + (best.distance + RAYON_PILIER) * sin(adjusted_angle_rad);
        float dx = x - pillars[p].x;
        float dy = y - pillars[p].y;
        float error = sqrt(dx*dx + dy*dy);
        if (error > 350) {
            best.found = false;
        }
        trackedPoints[p] = best;

    }
}

void inittrackedpoints(std::vector<TrackResult>& trackedPoints, position pos, int numPoints) {
    // a partir de pilars et posimu, initialiser les positions des points à suivre
    for (int i = 0; i < numPoints && i < pillars.size(); ++i) {
        TrackResult tr;
        float dx = pillars[i].x - pos.x;
        float dy = pillars[i].y - pos.y;
        tr.distance = sqrt(dx * dx + dy * dy);
        tr.angle = atan2(dy, dx); // radians
        if (tr.angle < 0) tr.angle += 2 * PI; // angle positif
        tr.found = true;    
        trackedPoints.push_back(tr);
    }   
}


// cette fontion a pourbut de trouver le centre du pilier( d=100mm)
// deja trouver a angle et distance, adapter le span de recherche en fonction
// de la distance et filtrer les point trop eloignés de distance
float findcenter(const std::vector<float>& scan, float angle, float distance) {
    
    if (distance <= 0.0f) return -1; // Évite les erreurs si distance est nulle ou négative

    float angle_rad = angle; // conversion en radians
    float angle_deg = angle * 180.0f / PI; // conversion en degrés
    float angle_span = atan2(150.0f, distance) * 180.0f / PI; // angle de recherche en degrés (rayon 100 mm)
    int start_index = static_cast<int>(std::round((angle_deg - angle_span) / RESOLUTION));
    int end_index = static_cast<int>(std::round((angle_deg + angle_span) / RESOLUTION));

    // Gestion des limites
    start_index = std::max(0, start_index);
    end_index = std::min(NUM_ANGLES - 1, end_index);

    int best_index = -1;
    float best_distance = 100000.0f; // distance max au centre du pilier

    for (int i = start_index; i <= end_index; ++i) {
        int index_mod = (i % NUM_ANGLES + NUM_ANGLES) % NUM_ANGLES; // Gestion du "wrap-around"
        float d = scan[index_mod];
        // Filtrer les points trop éloignés
        float dd= fabs(d - distance);
        if (dd > 190.0f) continue; // Ignore les points à plus
        if (d > 0 && d < best_distance) {
            best_distance = d;
            best_index = index_mod; // Retourne l'indice modifié, pas i
        }
    }
    angle = best_index * RESOLUTION; // conversion en degrés
    angle_rad= angle * PI / 180.0f; // conversion en radians

    return angle_rad; // retourne l'angle en radians
}



void try_accept_client(int& client_sock, int server_sock) {
    if (client_sock < 0) {
        client_sock = accept(server_sock, NULL, NULL);
        if (client_sock < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                perror("accept");
        } else {
            printf("Client Python connecté.\n");
        }
    }
}



void init_socket(int& server_sock, int& client_sock, struct sockaddr_un& addr, const char *socket_path)
{
    // Supprimer l'ancien fichier s'il existe
    unlink(socket_path);

    if (server_sock < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Configurer l'adresse
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

    // Associer le socket au fichier
    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    // Mettre en écoute
    if (listen(server_sock, 1) < 0) {
        perror("listen");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    printf("Serveur en attente de connexion sur %s...\n", socket_path);
}


void print_usage(int argc, const char * argv[])
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
		   " For udp channel\n %s --channel --udp <ipaddr> [port NO.]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool grabAndUpdateScan(std::vector<float>& scan, sl::ILidarDriver* drv) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    // Récupération des données
    sl_result op_result = drv->grabScanDataHq(nodes, count);

    if (SL_IS_OK(op_result)) {
        for (size_t pos = 0; pos < count; ++pos) {
            // Conversion angle Q14 -> degrés
            float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;

            // Conversion distance Q2 -> mm
            float distance = nodes[pos].dist_mm_q2 / 4.0f;

            // Qualité
            int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

            int index = static_cast<int>(std::round(angle / RESOLUTION));
            // Seulement si qualité > 0
            if (quality > 0) {
                if (index >= NUM_ANGLES) index = 0; // sécurité pour 360°
                // je retourne le scan , 1° devient 359°
                index = (NUM_ANGLES - index) % NUM_ANGLES;

                // Écrase la valeur précédente
                scan[index] = distance;
            }else{
                scan[index] = 0; 
            }
        }
        return true; // succès
    }

    return false; // échec acquisition
}



bool initIMU(ImuOTOS& imu) {
    if (!imu.openBus()) {
        std::cerr << "Erreur ouverture I2C\n";
        return false;
    }
    if (!imu.reset()) {
        std::cerr << "Erreur reset\n";
        imu.closeBus();
        return false;
    }
    if (!imu.calibrate(255)) {
        std::cerr << "Erreur calibrage\n";
        imu.closeBus();
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3 * 255));
    if (!imu.enableSignalProcessing(true, true, true, true)) {
        std::cerr << "Erreur config signal\n";
        imu.closeBus();
        return false;
    }
    return true;
}


// --- Initialisation du driver LIDAR ---
sl::ILidarDriver* initLidar(int argc, const char* argv[], sl_lidar_response_device_info_t& devinfo, int& opt_channel_type, sl::IChannel*& _channel) {
    const char * opt_is_channel = NULL; 
    const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
    sl_u32 opt_channel_param_second = 0;
    sl_u32 baudrateArray[2] = {115200, 256000};
    sl_result op_result;
    bool useArgcBaudrate = false;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\nVersion: %s\n", SL_LIDAR_SDK_VERSION);

    if (argc > 1) { 
        opt_is_channel = argv[1];
    } else {
        print_usage(argc, argv);
        return nullptr;
    }

    if(strcmp(opt_is_channel, "--channel") == 0){
        opt_channel = argv[2];
        if(strcmp(opt_channel, "-s") == 0 || strcmp(opt_channel, "--serial") == 0) {
            opt_channel_param_first = argv[3];
            printf("serial port=%s\n", opt_channel_param_first);
            if (argc > 4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
            printf("baudrate=%d\n", opt_channel_param_second);
            useArgcBaudrate = true;
        } else if(strcmp(opt_channel, "-u") == 0 || strcmp(opt_channel, "--udp") == 0) {
            opt_channel_param_first = argv[3];
            if (argc > 4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
            opt_channel_type = CHANNEL_TYPE_UDP;
        } else {
            print_usage(argc, argv);
            return nullptr;
        }
    } else {
        print_usage(argc, argv);
        return nullptr;
    }

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (!opt_channel_param_first) {
#ifdef _WIN32
            opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
            opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
            opt_channel_param_first = "/dev/ttyUSB0";
#endif
        }
    }

    ILidarDriver * drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        return nullptr;
    }

    bool connectSuccess = false;
    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);
                if (SL_IS_OK(op_result)) connectSuccess = true;
                else { delete drv; drv = NULL; }
            }
        } else {
            size_t baudRateArraySize = sizeof(baudrateArray) / sizeof(baudrateArray[0]);
            for(size_t i = 0; i < baudRateArraySize; ++i) {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);
                    if (SL_IS_OK(op_result)) { connectSuccess = true; break; }
                    else { delete drv; drv = NULL; }
                }
            }
        }
    } else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);
            if (SL_IS_OK(op_result)) connectSuccess = true;
            else { delete drv; drv = NULL; }
        }
    }

    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
            (fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_channel_param_first)):
            (fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n", opt_channel_param_first));
        if(drv) { delete drv; }
        return nullptr;
    }

    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\nFirmware Ver: %d.%02d\nHardware Rev: %d\n",
        devinfo.firmware_version>>8,
        devinfo.firmware_version & 0xFF,
        (int)devinfo.hardware_version);

    if (!checkSLAMTECLIDARHealth(drv)) {
        delete drv;
        return nullptr;
    }

    return drv;
}

