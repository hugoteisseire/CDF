// server.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

int main() {
    const char *socket_path = "/tmp/robot.sock";

    // Supprimer l'ancien fichier s'il existe
    unlink(socket_path);

    // Créer le socket
    int server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_sock < 0) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Configurer l'adresse
    struct sockaddr_un addr;
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


    
    // Accepter la connexion
    int client_sock = accept(server_sock, NULL, NULL);
    if (client_sock < 0) {
        perror("accept");
        close(server_sock);
        exit(EXIT_FAILURE);
    }

    printf("Client Python connecté.\n");

    // Boucle d'envoi
    float data[3];
    while (1) {
        data[0] = 1.23512892f;
        data[1] = 4.56f;
        data[2] = 7.89f;
        ssize_t sent = write(client_sock, data, sizeof(data));
        if (sent < 0) {
            perror("write");
            break;
        }
        usleep(10000); // 10 ms = 100 Hz
    }

    close(client_sock);
    close(server_sock);
    unlink(socket_path);
    return 0;
}
