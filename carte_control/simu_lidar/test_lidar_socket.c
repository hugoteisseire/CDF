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

int main() {
    int client_fd;
    struct sockaddr_un server_addr;
    int count = 0;
    int retry = 0;
    
    // États possibles
    const char *states[] = {"init\n", "ready\n", "running\n", "lost\n", "ready\n", "running\n"};
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
    
    printf("Envoi des états simulés...\n\n");
    
    while (running) {
        // Sélection de l'état actuel (cycle à travers les états)
        const char *current_state = states[count % num_states];
        
        // Envoi de l'état
        ssize_t sent = write(client_fd, current_state, strlen(current_state));
        if (sent < 0) {
            perror("Erreur envoi");
            break;
        }
        
        // Affichage
        printf("[%04d] État envoyé: %s", count, current_state);
        
        count++;
        sleep(3);  // Changement d'état toutes les 2 secondes
    }
    
    close(client_fd);
    
    printf("\n✅ Simulateur arrêté proprement\n");
    return 0;
}
