/**
 * @file robot_config.h
 * @brief Configuration centralisée pour le robot de la Coupe de France de Robotique
 * 
 * Ce fichier regroupe toutes les constantes de configuration :
 * - Paramètres géométriques (dimensions table, piliers)
 * - Paramètres LIDAR (résolution, limites)
 * - Paramètres de localisation (seuils, tolérances)
 * - Positions initiales par équipe
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <cmath>
#include <vector>

// ============================================================================
// TYPES ET ENUMS
// ============================================================================

/**
 * @brief Équipe du robot (détermine côté de la table et positions initiales)
 */
enum class Team {
    TEAM0 = 0,  ///< Équipe 0 - côté droit de la table
    TEAM1 = 1,  ///< Équipe 1 - côté gauche de la table
    DEBUG = 3   ///< Mode debug - position centrale pour tests
};

/**
 * @brief Structure représentant une position 2D avec orientation
 */
struct Position {
    float x_mm;      ///< Position X en millimètres
    float y_mm;      ///< Position Y en millimètres
    float angle_rad; ///< Orientation en radians (repère mathématique direct)
};

// ============================================================================
// CONSTANTES MATHÉMATIQUES
// ============================================================================

constexpr float PI = 3.14159265358979323846f;

// ============================================================================
// DIMENSIONS TABLE DE JEU (Coupe de France Robotique)
// ============================================================================

constexpr float TABLE_WIDTH_MM = 3000.0f;   ///< Largeur de la table en mm
constexpr float TABLE_HEIGHT_MM = 2000.0f;  ///< Hauteur de la table en mm

// ============================================================================
// PARAMÈTRES LIDAR SLAMTEC
// ============================================================================

constexpr float LIDAR_RESOLUTION_DEG = 0.25f;           ///< Résolution angulaire en degrés
constexpr int LIDAR_NUM_ANGLES = 1440;                  ///< Nombre d'angles (360° / 0.25°)
constexpr float LIDAR_MAX_DISTANCE_MM = 6000.0f;        ///< Distance max mesurable en mm
constexpr float LIDAR_VALID_DISTANCE_MAX_MM = 3700.0f;  ///< Distance max valide pour filtrage

// ============================================================================
// PARAMÈTRES PILIERS DE LOCALISATION
// ============================================================================

constexpr float PILLAR_DIAMETER_MM = 100.0f;            ///< Diamètre réel d'un pilier
constexpr float PILLAR_RADIUS_MM = PILLAR_DIAMETER_MM / 2.0f; ///< Rayon d'un pilier

// Tolérance de position pour validation de détection
constexpr float PILLAR_POSITION_TOLERANCE_MM = 450.0f;  ///< Erreur max acceptée (mm)

// Span angulaire de recherche du centre d'un pilier (en fonction distance)
constexpr float PILLAR_SEARCH_RADIUS_MM = 150.0f;       ///< Rayon de recherche autour du centre estimé

// Filtrage distance lors de la recherche du centre
constexpr float PILLAR_DISTANCE_FILTER_MM = 190.0f;     ///< Écart max distance accepté

// ============================================================================
// PARAMÈTRES DE TRACKING
// ============================================================================

constexpr float TRACKING_MAX_DISTANCE_MM = 2000.0f;     ///< Distance de normalisation pour le score
constexpr int TRACKING_MIN_QUALITY = 1;                 ///< Qualité minimale scan LIDAR

// ============================================================================
// PARAMÈTRES IMU (OTOS)
// ============================================================================

constexpr int IMU_CALIBRATION_TIME_MS = 3 * 255;       ///< Temps de calibration en ms
constexpr int IMU_SMOOTHING_WINDOW_SIZE = 5;           ///< Taille fenêtre moyenne glissante

// ============================================================================
// POSITIONS DES PILIERS PAR ÉQUIPE
// ============================================================================

/**
 * @brief Positions des piliers pour l'équipe 0 (côté droit)
 * Coordonnées en mm dans le repère table (origine coin bas-gauche)
 */
const std::vector<Position> PILLARS_TEAM0 = {
    {2950.0f, 50.0f, 0.0f},     // Pilier 1 - coin bas-droit
    {2950.0f, 1950.0f, 0.0f},   // Pilier 2 - coin haut-droit
    {50.0f, 1000.0f, 0.0f}      // Pilier 3 - milieu gauche
};

/**
 * @brief Positions des piliers pour l'équipe 1 (côté gauche)
 */
const std::vector<Position> PILLARS_TEAM1 = {
    {50.0f, 50.0f, 0.0f},       // Pilier 1 - coin bas-gauche
    {50.0f, 1950.0f, 0.0f},     // Pilier 2 - coin haut-gauche
    {2950.0f, 1000.0f, 0.0f}    // Pilier 3 - milieu droit
};

/**
 * @brief Positions des piliers pour le mode debug (mesurées pour tests)
 */
const std::vector<Position> PILLARS_DEBUG = {
    {0.0f, 457.0f, 0.0f},       // Pilier 1
    {1759.0f, 859.0f, 0.0f},    // Pilier 2
    {1769.0f, 56.0f, 0.0f}      // Pilier 3
};

// ============================================================================
// POSITIONS INITIALES DU ROBOT PAR ÉQUIPE
// ============================================================================

/**
 * @brief Position initiale du robot pour l'équipe 0
 * x=2200mm, y=1000mm, orientation=0 rad (vers +X)
 */
constexpr Position ROBOT_START_TEAM0 = {2200.0f, 1000.0f, 0.0f};

/**
 * @brief Position initiale du robot pour l'équipe 1
 * x=800mm, y=1000mm, orientation=π rad (vers -X)
 */
const Position ROBOT_START_TEAM1 = {800.0f, 1000.0f, PI};

/**
 * @brief Position initiale du robot pour le mode debug
 * x=750mm, y=145mm, orientation=π/2 rad (vers +Y)
 */
const Position ROBOT_START_DEBUG = {750.0f, 145.0f, PI / 2.0f};

// ============================================================================
// CONFIGURATION COMMUNICATION
// ============================================================================

constexpr const char* SOCKET_PATH = "/tmp/robot.sock"; ///< Chemin socket UNIX vers Python

// ============================================================================
// FONCTIONS UTILITAIRES
// ============================================================================

/**
 * @brief Récupère les positions des piliers pour une équipe donnée
 * @param team Équipe sélectionnée
 * @return Vecteur des positions des piliers (3 piliers)
 */
inline const std::vector<Position>& getPillarPositions(Team team) {
    switch (team) {
        case Team::TEAM1:
            return PILLARS_TEAM1;
        case Team::DEBUG:
            return PILLARS_DEBUG;
        case Team::TEAM0:
        default:
            return PILLARS_TEAM0;
    }
}

/**
 * @brief Récupère la position initiale du robot pour une équipe donnée
 * @param team Équipe sélectionnée
 * @return Position initiale (x, y, orientation)
 */
inline Position getRobotStartPosition(Team team) {
    switch (team) {
        case Team::TEAM1:
            return ROBOT_START_TEAM1;
        case Team::DEBUG:
            return ROBOT_START_DEBUG;
        case Team::TEAM0:
        default:
            return ROBOT_START_TEAM0;
    }
}

/**
 * @brief Convertit un entier en enum Team
 * @param team_int Valeur entière (0, 1 ou 3)
 * @return Enum Team correspondant
 */
inline Team intToTeam(int team_int) {
    switch (team_int) {
        case 1: return Team::TEAM1;
        case 3: return Team::DEBUG;
        default: return Team::TEAM0;
    }
}

/**
 * @brief Obtient le nom lisible d'une équipe
 * @param team Équipe
 * @return Chaîne de caractères (ex: "Équipe 0 (côté droit)")
 */
inline const char* getTeamName(Team team) {
    switch (team) {
        case Team::TEAM0: return "Équipe 0 (côté droit)";
        case Team::TEAM1: return "Équipe 1 (côté gauche)";
        case Team::DEBUG: return "Mode DEBUG (central)";
        default: return "Équipe inconnue";
    }
}

#endif // ROBOT_CONFIG_H
