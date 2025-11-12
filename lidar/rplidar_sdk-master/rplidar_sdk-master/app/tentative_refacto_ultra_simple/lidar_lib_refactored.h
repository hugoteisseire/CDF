/**
 * @file lidar_lib_refactored.h
 * @brief Bibliothèque de traitement LIDAR pour localisation par piliers
 * 
 * Cette bibliothèque fournit les fonctions nécessaires pour :
 * - Suivre des piliers de référence dans un scan LIDAR
 * - Calculer la pose du robot par trilatération (2 ou 3 piliers)
 * - Filtrer les points dans la table de jeu
 * - Initialiser et configurer le système LIDAR + IMU
 */

#ifndef LIDAR_LIB_REFACTORED_H
#define LIDAR_LIB_REFACTORED_H

#include "robot_config.h"
#include "ImuOTOS.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <vector>
#include <deque>

// ============================================================================
// STRUCTURES DE DONNÉES
// ============================================================================

/**
 * @brief Résultat du suivi d'un point (pilier)
 */
struct TrackResult {
    float angle_rad;    ///< Angle du point détecté en radians
    float distance_mm;  ///< Distance au point en millimètres
    bool found;         ///< true si le point a été trouvé dans le scan
};

/**
 * @brief Structure pour stocker un scan LIDAR complet
 * Index = angle_deg / LIDAR_RESOLUTION_DEG
 * Valeur = distance en mm (0 si pas de mesure)
 */
using LidarScan = std::vector<float>;

// ============================================================================
// INITIALISATION ET CONFIGURATION
// ============================================================================

/**
 * @brief Initialise les positions des piliers selon l'équipe
 * @param team Équipe sélectionnée (TEAM0, TEAM1 ou DEBUG)
 * @return Vecteur des positions des 3 piliers
 */
std::vector<Position> initPillars(Team team);

/**
 * @brief Initialise les points de tracking à partir de la position du robot
 * @param robotPos Position actuelle du robot
 * @param pillarPositions Positions des piliers dans le repère global
 * @return Vecteur des TrackResult initialisés (angle et distance relatifs)
 */
std::vector<TrackResult> initTrackedPoints(const Position& robotPos, 
                                           const std::vector<Position>& pillarPositions);

// ============================================================================
// TRAITEMENT SCAN LIDAR
// ============================================================================

/**
 * @brief Acquiert un scan LIDAR et le stocke dans un vecteur
 * @param[out] scan Vecteur de distances (indexé par angle / résolution)
 * @param[in] drv Driver LIDAR SLAMTEC
 * @return true si l'acquisition a réussi, false sinon
 */
bool grabAndUpdateScan(LidarScan& scan, sl::ILidarDriver* drv);

/**
 * @brief Filtre les points du scan qui se trouvent dans la table de jeu
 * @param[in] scan Scan LIDAR brut
 * @param[out] pointsInTable Points filtrés dans le repère global
 * @param[in] robotPos Position actuelle du robot
 */
void filterPointsInTable(const LidarScan& scan, 
                         std::vector<Position>& pointsInTable,
                         const Position& robotPos);

// ============================================================================
// TRACKING DES PILIERS
// ============================================================================

/**
 * @brief Suit les piliers dans le scan et met à jour leur position
 * @param[in] scan Scan LIDAR actuel
 * @param[in,out] trackedPoints Points suivis (mis à jour avec nouvelles positions)
 * @param[in] robotPos Position actuelle du robot (pour validation)
 * @param[in] pillarPositions Positions de référence des piliers
 */
void trackPillars(const LidarScan& scan,
                  std::vector<TrackResult>& trackedPoints,
                  const Position& robotPos,
                  const std::vector<Position>& pillarPositions);

/**
 * @brief Trouve le centre d'un pilier par recherche du point le plus proche
 * @param[in] scan Scan LIDAR
 * @param[in] estimatedAngle_rad Angle estimé du centre (radians)
 * @param[in] estimatedDistance_mm Distance estimée (mm)
 * @return Angle du centre réel en radians
 */
float findPillarCenter(const LidarScan& scan, 
                       float estimatedAngle_rad, 
                       float estimatedDistance_mm);

// ============================================================================
// CALCUL DE POSE (TRILATÉRATION)
// ============================================================================

/**
 * @brief Calcule la pose du robot à partir de 2 ou 3 piliers détectés
 * @param[in] measurements Mesures des piliers (angle, distance relatifs)
 * @param[in] pillarPositions Positions de référence des piliers
 * @return Position calculée du robot (x, y, orientation)
 */
Position computeRobotPose(const std::vector<TrackResult>& measurements,
                          const std::vector<Position>& pillarPositions);

// ============================================================================
// CONVERSIONS
// ============================================================================

/**
 * @brief Convertit une Position en ImuPose (mm -> m)
 * @param pos Position en mm
 * @return ImuPose en mètres
 */
ImuPose positionToImuPose(const Position& pos);

/**
 * @brief Convertit une mesure polaire en coordonnées cartésiennes relatives
 * @param measurement Mesure (angle, distance)
 * @return Position relative au robot
 */
Position polarToCartesian(const TrackResult& measurement);

// ============================================================================
// INITIALISATION MATÉRIEL
// ============================================================================

/**
 * @brief Initialise l'IMU OTOS (reset, calibration, config)
 * @param[in,out] imu Instance de l'IMU
 * @return true si succès, false sinon
 */
bool initIMU(ImuOTOS& imu);

/**
 * @brief Initialise le driver LIDAR SLAMTEC
 * @param[in] argc Nombre d'arguments
 * @param[in] argv Arguments (channel, serial/udp, port, baudrate)
 * @param[out] devinfo Informations du device LIDAR
 * @param[out] channelType Type de canal (série ou UDP)
 * @param[out] channel Pointeur vers le canal créé
 * @return Pointeur vers le driver LIDAR, nullptr si échec
 */
sl::ILidarDriver* initLidar(int argc, const char* argv[], 
                            sl_lidar_response_device_info_t& devinfo,
                            int& channelType, 
                            sl::IChannel*& channel);

/**
 * @brief Vérifie l'état de santé du LIDAR SLAMTEC
 * @param drv Driver LIDAR
 * @return true si OK, false si erreur interne
 */
bool checkLidarHealth(sl::ILidarDriver* drv);

// ============================================================================
// COMMUNICATION SOCKET
// ============================================================================

/**
 * @brief Initialise un socket UNIX serveur
 * @param[out] serverSock Descripteur du socket serveur
 * @param[out] clientSock Descripteur du socket client (-1 initialement)
 * @param[out] addr Structure d'adresse UNIX
 * @param[in] socketPath Chemin du fichier socket
 */
void initSocket(int& serverSock, int& clientSock, 
                struct sockaddr_un& addr, const char* socketPath);

/**
 * @brief Tente d'accepter un client (non-bloquant)
 * @param[in,out] clientSock Descripteur du socket client (mis à jour si connexion)
 * @param[in] serverSock Descripteur du socket serveur
 */
void tryAcceptClient(int& clientSock, int serverSock);

// ============================================================================
// UTILITAIRES
// ============================================================================

/**
 * @brief Affiche l'usage du programme
 * @param argc Nombre d'arguments
 * @param argv Arguments du programme
 */
void printUsage(int argc, const char* argv[]);

#endif // LIDAR_LIB_REFACTORED_H
