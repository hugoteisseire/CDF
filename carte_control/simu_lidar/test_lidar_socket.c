/*
 * Simulateur simple d'états LIDAR via socket UNIX
 * Se connecte au serveur Python et envoie périodiquement des changements d'état
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <signal.h>

#define SOCKET_PATH "/tmp/robot.sock"
#define MAX_RETRIES 10

volatile int running = 1;

void sigint_handler(int sig) {
    running = 0;
    printf("\nArrêt du simulateur...\n");
}

int main(int argc, char *argv[]) {
    int team=0;
    if (argc > 1) {
        team = atoi(argv[1]);  // Récupère 0 ou 1
        printf("Équipe: %d\n", team);
    }
    int client_fd;
    struct sockaddr_un server_addr;
    int count = 0;
    int retry = 0;
    int delay_between_states = 2; // secondes
    if (team == 1) {
        printf("⚠️  Mode Équipe 1 activé (délai entre états augmenté)\n");
        delay_between_states = 5;
    }
    // États possibles
    const char *states[] = {"init\n", "ready\n", "lost\n", "error\n"};
    int num_states = sizeof(states) / sizeof(states[0]);
    
    signal(SIGINT, sigint_handler);
    
    printf("🔌 Simulateur LIDAR (CLIENT)\n");
    printf("Connexion au serveur Python sur %s...\n", SOCKET_PATH);
    
    // Créer le socket client
    client_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (client_fd < 0) {
        perror("Erreur création socket");
        return 1;
    }
    
    // Configuration de l'adresse du serveur
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, SOCKET_PATH, sizeof(server_addr.sun_path) - 1);
    
    // Tentatives de connexion avec retry
    while (retry < MAX_RETRIES && running) {
        if (connect(client_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == 0) {
            printf("✅ Connecté au serveur Python!\n");
            break;
        }
        
        retry++;
        printf("⏳ Tentative %d/%d...\n", retry, MAX_RETRIES);
        sleep(1);
    }
    
    if (retry >= MAX_RETRIES) {
        fprintf(stderr, "❌ Impossible de se connecter au serveur après %d tentatives\n", MAX_RETRIES);
        close(client_fd);
        return 1;
    }
    
    printf("Envoi des états + données simulées...\n\n");
    
    while (running) {
        // Sélection de l'état actuel (cycle à travers les états)
        const char *current_state = states[count % num_states];
        
        // Envoi de l'état (string terminée par \n)
        ssize_t sent = write(client_fd, current_state, strlen(current_state));
        if (sent < 0) {
            perror("Erreur envoi état");
            break;
        }
        
        // Simulation des données LIDAR
        // float data[20] = {posImu.x, posImu.y, posImu.angle, 
        //                   poslidar.x, poslidar.y, poslidar.angle, 
        //                   pos_adv.x, pos_adv.y,
        //                   trackedPoints[0].angle, trackedPoints[0].distance, trackedPoints[0].found,
        //                   trackedPoints[1].angle, trackedPoints[1].distance, trackedPoints[1].found,
        //                   trackedPoints[2].angle, trackedPoints[2].distance, trackedPoints[2].found,
        //                   reserved[0], reserved[1]}
        
        float data[20];
        
        // posImu (x, y, angle)
        data[0] = 1000.0f + count * 10.0f;  // posImu.x (mm)
        data[1] = 500.0f + count * 5.0f;     // posImu.y (mm)
        data[2] = (count % 360) * 3.14159f / 180.0f;  // posImu.angle (rad)
        
        // poslidar (x, y, angle)
        data[3] = 1005.0f + count * 10.0f;  // poslidar.x (mm)
        data[4] = 505.0f + count * 5.0f;     // poslidar.y (mm)
        data[5] = (count % 360) * 3.14159f / 180.0f;  // poslidar.angle (rad)
        
        // pos_adv (x, y) - position adversaire
        data[6] = 2000.0f;  // pos_adv.x (mm)
        data[7] = 1500.0f;  // pos_adv.y (mm)
        
        // trackedPoints[0] (angle, distance, found)
        data[8] = 45.0f * 3.14159f / 180.0f;  // angle (rad)
        data[9] = 800.0f;                      // distance (mm)
        data[10] = 1.0f;                       // found (1=trouvé, 0=perdu)
        
        // trackedPoints[1]
        data[11] = 135.0f * 3.14159f / 180.0f;
        data[12] = 750.0f;
        data[13] = 1.0f;
        
        // trackedPoints[2]
        data[14] = 225.0f * 3.14159f / 180.0f;
        data[15] = 820.0f;
        data[16] = 0.0f;  // perdu
        
        // Réservé pour extensions futures
        data[17] = 0.0f;
        data[18] = 0.0f;
        data[19] = 0.0f;
        
        // Envoi de la trame de données (80 octets = 20 floats)
        sent = write(client_fd, data, sizeof(data));
        if (sent < 0) {
            perror("Erreur envoi données");
            break;
        }
        
        // Affichage
        printf("[%04d] État: %s", count, current_state);
        printf("       Données: posImu=(%.1f, %.1f, %.2f°) posLidar=(%.1f, %.1f, %.2f°) piliers=%d/%d/%d\n",
               data[0], data[1], data[2] * 180.0f / 3.14159f,
               data[3], data[4], data[5] * 180.0f / 3.14159f,
               (int)data[10], (int)data[13], (int)data[16]);
        
        count++;
        sleep(delay_between_states);  // Changement d'état toutes les N secondes
    }
    
    close(client_fd);
    
    printf("\n✅ Simulateur arrêté proprement\n");
    return 0;
}
