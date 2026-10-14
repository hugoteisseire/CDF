#include "lidar_lib.h"
#include "ImuOTOS.h"
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <iostream>
#include <thread>
#include <chrono>  

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace std::chrono;

using namespace sl;
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// Positions des piliers - initialisées selon l'équipe
std::vector<position> pillars;

// Positions des piliers pour chaque équipe
const std::vector<position> pillars_team0 = {
    {2950, 50, 0},    // Pilier 1
    {2950, 1950, 0},  // Pilier 2
    {50, 1000, 0}     // Pilier 3
};

const std::vector<position> pillars_team1 = {
    {50, 50, 0},      // Pilier 1
    {50, 1950, 0},    // Pilier 2
    {2950, 1000, 0}   // Pilier 3
};

const std::vector<position> pillars_teamdebug = {
    {200 , 1800, 0},    // Pilier 1 
    {-30, 30, 0},  // Pilier 2
    {3030, 1000, 0}     // Pilier 3
};

/**
 * Initialise les positions des piliers selon l'équipe.
 * @param team 0, 1 ou 3 (debug)
 */
void initPillars(int team) {
    if (team == 1) {
        pillars = pillars_team1;
        std::cout << "🔵 Piliers équipe 1 (côté gauche) chargés\n";
    } else if (team == 3) {
        pillars = pillars_teamdebug;
        std::cout << "� MODE DEBUG activé\n";
    } else {
        pillars = pillars_team0;
        std::cout << "🟡 Piliers équipe 0 (côté droit) chargés\n";
    }
    
    // Affichage des positions pour vérification
    for (size_t i = 0; i < pillars.size(); ++i) {
        std::cout << "  Pilier " << (i+1) << ": x=" << pillars[i].x 
                  << " mm, y=" << pillars[i].y << " mm\n";
    }
}



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

void point_in_table(const std::vector<float>& scan,std::vector<position>& points_in_table, float resolution,position robot){
  //boucle parcourant scan
    for(int i=0;i<scan.size();i++){
        if(scan[i]>0){
        float angle=(float)(i*resolution)*PI/180.0f+robot.angle; //angle en radian
        float x=robot.x+((scan[i])*cos(angle));
        float y=robot.y+((scan[i])*sin(angle));
        if(x>=0 && x<=3000 && y>=0 && y<=2000){ //si le point est dans la table

            position p;
            p.x=x;
            p.y=y;
            points_in_table.push_back(p);
        }
        
        }
    }

}

void point_in_table2(const std::vector<float>& scan, 
                    std::vector<position>& points_in_table,
                    float resolution, 
                    const position& robot)
{   points_in_table.clear();
    points_in_table.reserve(scan.size());  // évite réallocations

    const float res_rad = resolution * PI / 180.0f; // une seule fois
    float cos_r = std::cos(robot.angle);
    float sin_r = std::sin(robot.angle);

    // pré-calcule cos/sin du pas angulaire
    const float cos_step = std::cos(res_rad);
    const float sin_step = std::sin(res_rad);

    // angle initial (orientation robot)
    float cos_a = cos_r;
    float sin_a = sin_r;

    for (int i = 0; i < (int)scan.size(); ++i) {
        float r = scan[i];
        if (r > 0 && r < 3700) { // filtre distance max 6m
            float dist = r;
            float x = robot.x + dist * cos_a;
            float y = robot.y + dist * sin_a;

            // test AABB (table 3000 x 2000)
            if (x >= 0 && x <= 3000 && y >= 0 && y <= 2000) {
                position p;
                p.x = x;
                p.y = y;
                points_in_table.push_back(p);
            }

        }

        // rotation incrémentale : (cos, sin) = rotation(angle + res_rad)
        float tmp_cos = cos_a * cos_step - sin_a * sin_step;
        sin_a = sin_a * cos_step + cos_a * sin_step;
        cos_a = tmp_cos;
    }
}


// ================== Détection du robot adverse via clustering ==================
// Hypothèse: seul le robot adverse constitue une masse significative de points dans la table
// (en dehors de petits bruits ou des piliers déjà suivis). On extrait donc le plus gros cluster
// en distance euclidienne puis on en déduit sa position comme le centroïde.

static constexpr float OPP_CLUSTER_RADIUS_MM      = 180.0f; // rayon max pour regrouper des points dans le même cluster
static constexpr int   OPP_CLUSTER_MIN_POINTS     = 8;      // nombre minimal de points pour considérer un cluster valide
static constexpr float OPP_PRIOR_SEARCH_RADIUS_MM = 600.0f; // rayon dans lequel on cherche d'abord autour de la position précédente
static constexpr float OPP_SMOOTH_ALPHA           = 0.35f;  // lissage (optionnel) sur la mise à jour de la position

struct OpponentDetection {
    position centroid; // position estimée
    int points = 0;    // nombre de points utilisés
    bool found = false;
};

// Cluster naive O(n^2), suffisant pour quelques centaines de points. Si >10k points, prévoir grille.
static OpponentDetection cluster_largest(const std::vector<position>& pts,
                                         float clusterRadius,
                                         int minPoints) {
    const float r2 = clusterRadius * clusterRadius;
    const int n = static_cast<int>(pts.size());
    std::vector<char> visited(n, 0);
    OpponentDetection best;
    std::vector<int> queue;
    queue.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (visited[i]) continue;
        // BFS/expansion
        queue.clear();
        std::vector<int> cluster;
        cluster.reserve(32);
        queue.push_back(i);
        visited[i] = 1;
        for (size_t qi = 0; qi < queue.size(); ++qi) {
            int idx = queue[qi];
            cluster.push_back(idx);
            const position &p = pts[idx];
            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                float dx = pts[j].x - p.x;
                float dy = pts[j].y - p.y;
                if (dx*dx + dy*dy <= r2) {
                    visited[j] = 1;
                    queue.push_back(j);
                }
            }
        }
        if ((int)cluster.size() >= minPoints) {
            // calcul centroïde
            double sx = 0.0, sy = 0.0;
            for (int id : cluster) { sx += pts[id].x; sy += pts[id].y; }
            position c; c.x = (float)(sx / cluster.size()); c.y = (float)(sy / cluster.size()); c.angle = 0.0f;
            if (cluster.size() > best.points) {
                best.centroid = c;
                best.points = (int)cluster.size();
                best.found = true;
            }
        }
    }
    return best;
}

// 1) Détection simple du robot adverse (plus gros cluster) après extraction des points dans la table.
bool detectOpponentRobot(const std::vector<position>& points_in_table,
                         position &opponent_out) {
    OpponentDetection det = cluster_largest(points_in_table,
                                            OPP_CLUSTER_RADIUS_MM,
                                            OPP_CLUSTER_MIN_POINTS);
    if (!det.found) return false;
    opponent_out = det.centroid;
    return true;
}

// 2) Détection privilégiant la zone autour d'une position précédente connue. Si rien trouvé dans
//    le voisinage prioritaire, on retombe sur la stratégie du plus gros cluster global.
bool detectOpponentRobotWithPrior(const std::vector<position>& points_in_table,
                                  const position& previous,
                                  position &opponent_out) {
    // Filtrer points proches du prior
    const float searchR2 = OPP_PRIOR_SEARCH_RADIUS_MM * OPP_PRIOR_SEARCH_RADIUS_MM;
    std::vector<position> local;
    local.reserve(points_in_table.size());
    for (const auto &p : points_in_table) {
        float dx = p.x - previous.x;
        float dy = p.y - previous.y;
        if (dx*dx + dy*dy <= searchR2) local.push_back(p);
    }

    OpponentDetection det;
    if (!local.empty()) {
        det = cluster_largest(local, OPP_CLUSTER_RADIUS_MM, OPP_CLUSTER_MIN_POINTS);
    }

    if (!det.found) {
        // fallback global
        det = cluster_largest(points_in_table, OPP_CLUSTER_RADIUS_MM, OPP_CLUSTER_MIN_POINTS);
        if (!det.found) return false;
        // pas de lissage si on part de zéro
        opponent_out = det.centroid;
        return true;
    }

    // Lissage léger vers le nouveau centroïde
    opponent_out.x = previous.x + OPP_SMOOTH_ALPHA * (det.centroid.x - previous.x);
    opponent_out.y = previous.y + OPP_SMOOTH_ALPHA * (det.centroid.y - previous.y);
    opponent_out.angle = 0.0f; // angle non pertinent ici
    return true;
}



// ================== Paramètres de configuration du suivi des piliers ==================
// Tous regroupés ici pour faciliter le réglage et éviter la "magie" de nombres en dur.
// Ajustez selon le bruit du capteur, la précision de la pose robot et la taille réelle des piliers.
static constexpr float TRACK_EXTRA_MARGIN_MM = 120.0f;          // marge ajoutée au rayon du pilier pour calcul de la largeur angulaire apparente
static constexpr float TRACK_MIN_SPAN_DEG    = 3.0f;            // largeur angulaire minimale (en degrés) de la fenêtre de recherche
static constexpr float TRACK_MAX_SPAN_DEG    = 50.0f;           // largeur angulaire maximale (en degrés) de la fenêtre de recherche
static constexpr float TRACK_GATE_MIN_MM     = 180.0f;          // tolérance radiale minimale autour de la distance attendue
static constexpr float TRACK_GATE_FRAC       = 0.25f;           // fraction de la distance attendue utilisée pour élargir la tolérance radiale
static constexpr float TRACK_GATE_MAX_MM     = 500.0f;          // tolérance radiale maximale
static constexpr float TRACK_VALIDATE_ERR_MM = 450.0f;          // erreur monde maxi (mm) pour accepter une détection nominale
static constexpr float TRACK_VALIDATE_ERR_FALLBACK_MM = 550.0f; // erreur monde maxi (mm) pour accepter une détection en mode fallback
static constexpr float TRACK_FALLBACK_EXTRA_SEARCH_DEG = 25.0f; // extension angulaire (de chaque côté) en mode fallback
static constexpr float TRACK_DEG_PENALTY_MM  = 20.0f;           // pénalité (mm) par degré d'écart angulaire dans le score fallback
static constexpr float TRACK_MIN_DISTANCE_MM = 50.0f;           // distance minimale (sécurité) pour éviter valeurs négatives
static constexpr float TRACK_MAX_VALID_MM    = 8000.0f;         // distance maximale considérée comme plausible (filtre brut)

// =======================================================================================

void trackPoints(const std::vector<float>& scan,
                 std::vector<TrackResult>& trackedPoints,
                 float /*resolution*/ ,
                 position posrobot) // Traque tous les piliers
{
    // On s'aligne sur l'indexation utilisée par grabAndUpdateScan (RESOLUTION/NUM_ANGLES)
    const float resDeg = RESOLUTION;
    const int numAngles = static_cast<int>(scan.size());
    if (numAngles <= 0) return;

    auto normAngle = [](float a) {
        while (a < 0) a += 2.0f * PI;
        while (a >= 2.0f * PI) a -= 2.0f * PI;
        return a;
    };

    auto angleToIndex = [&](float angleRad) {
        float angleDeg = angleRad * 180.0f / PI;
        int idx = static_cast<int>(std::round(angleDeg / resDeg));
        idx %= NUM_ANGLES;
        if (idx < 0) idx += NUM_ANGLES;
        return idx;
    };

    auto indexToAngle = [&](int idx){
        idx = (idx % NUM_ANGLES + NUM_ANGLES) % NUM_ANGLES;
        return (idx * resDeg) * PI / 180.0f;
    };

    auto median3 = [&](float a, float b, float c){
        // médiane sans std::swap pour limiter les includes
        if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
        if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
        return c;
    };

    for (size_t p = 0; p < trackedPoints.size(); ++p) {
        // -------------------- 1. Prédiction à partir de la carte et de la pose robot --------------------
        float dx = pillars[p].x - posrobot.x;
        float dy = pillars[p].y - posrobot.y;
        float pred_dist = std::sqrt(dx * dx + dy * dy) - RAYON_PILIER; // distance au bord du cylindre
        if (pred_dist < TRACK_MIN_DISTANCE_MM) pred_dist = TRACK_MIN_DISTANCE_MM; // évite 0/valeurs négatives
        float pred_angle = normAngle(std::atan2(dy, dx) - posrobot.angle); // angle dans le repère LIDAR

        // -------------------- 2. Point de référence (historique vs prédiction) --------------------
        // Si le pilier était déjà trouvé précédemment on exploite sa dernière position (suivi temporel implicite).
        float angle_ref = trackedPoints[p].found ? trackedPoints[p].angle : pred_angle; // rad
        float dist_ref  = (trackedPoints[p].found && trackedPoints[p].distance > 0) ? trackedPoints[p].distance : pred_dist; // mm

        // -------------------- 3. Construction de la fenêtre angulaire --------------------
        // Largeur apparente ~ 2*atan((rayon+ marge)/distance). On borne pour éviter une fenêtre trop étroite ou énorme.
        float spanRad = 2.0f * std::atan2(RAYON_PILIER + TRACK_EXTRA_MARGIN_MM, std::max(100.0f, dist_ref));
        float minSpan = TRACK_MIN_SPAN_DEG * (PI / 180.0f);
        float maxSpan = TRACK_MAX_SPAN_DEG * (PI / 180.0f);
        if (spanRad < minSpan) spanRad = minSpan;
        if (spanRad > maxSpan) spanRad = maxSpan;
        int halfWin = std::max(2, static_cast<int>(std::ceil((spanRad * 180.0f / PI) / resDeg)));

        int idx_center = angleToIndex(angle_ref);

        // -------------------- 4. Gating radial (tolérance sur la distance) --------------------
        // Combine une composante fixe (bruit, calibration) et une composante proportionnelle à la distance (divergence angulaire).
        float gate_mm = std::max(TRACK_GATE_MIN_MM, TRACK_GATE_FRAC * dist_ref);
        gate_mm = std::min(gate_mm, TRACK_GATE_MAX_MM);

        // -------------------- 5. Recherche d'un "run" cohérent --------------------
        // On parcourt les échantillons dans la fenêtre angulaire et détecte des segments contigus (runs)
        // de points dont la distance est dans la tolérance gate. Pour chaque run on prend le point
        // le plus proche (flanc du cylindre), puis on retient globalement le meilleur run.
        int bestIdx = -1;
        float bestRunMin = 1e9f;
        int runStart = -1;
        int runLen = 0;

        auto inGate = [&](float d){ return (d > 0.0f && d < TRACK_MAX_VALID_MM && std::fabs(d - dist_ref) <= gate_mm); };

        for (int di = -halfWin; di <= halfWin; ++di) {
            int idx = (idx_center + di + NUM_ANGLES) % NUM_ANGLES;
            float d = scan[idx];
            bool ok = inGate(d);
            if (ok) {
                if (runStart < 0) { runStart = idx; runLen = 1; }
                else { runLen++; }
            } else {
                if (runStart >= 0 && runLen > 0) {
                    // On prend l'indice du minimum dans ce run
                    int minIdx = runStart;
                    float minVal = 1e9f;
                    for (int k = 0; k < runLen; ++k) {
                        int j = (runStart + k) % NUM_ANGLES;
                        float v = scan[j];
                        if (inGate(v) && v < minVal) { minVal = v; minIdx = j; }
                    }
                    if (minVal < bestRunMin) { bestRunMin = minVal; bestIdx = minIdx; }
                    runStart = -1; runLen = 0;
                }
            }
        }
        // Flush dernier run si la fenêtre se termine sur un run
        if (runStart >= 0 && runLen > 0) {
            int minIdx = runStart;
            float minVal = 1e9f;
            for (int k = 0; k < runLen; ++k) {
                int j = (runStart + k) % NUM_ANGLES;
                float v = scan[j];
                if (inGate(v) && v < minVal) { minVal = v; minIdx = j; }
            }
            if (minVal < bestRunMin) { bestRunMin = minVal; bestIdx = minIdx; }
        }

        TrackResult best = {0.0f, 0.0f, false};

        if (bestIdx >= 0) {
            // Raffinement local: médiane 3 points puis recentrage via findcenter
            float vL = scan[(bestIdx - 1 + NUM_ANGLES) % NUM_ANGLES];
            float vC = scan[bestIdx];
            float vR = scan[(bestIdx + 1) % NUM_ANGLES];
            float vMed = median3(vL, vC, vR);

            float coarse_angle = indexToAngle(bestIdx);
            float refined_angle = findcenter(scan, coarse_angle, dist_ref);

            // Si findcenter échoue, on garde l'angle brut
            if (!(refined_angle >= 0.0f)) refined_angle = coarse_angle;

            int refined_idx = angleToIndex(refined_angle);
            float dsel = scan[refined_idx];
            if (dsel <= 0.0f) dsel = (vMed > 0.0f ? vMed : vC);

            best.angle = refined_angle;
            best.distance = dsel;
            best.found = true;

            // Validation dans le repère monde par rapport à la position connue du pilier
            float adjusted_angle_rad = best.angle + posrobot.angle; // rad
            float x = posrobot.x + (best.distance + RAYON_PILIER) * std::cos(adjusted_angle_rad);
            float y = posrobot.y + (best.distance + RAYON_PILIER) * std::sin(adjusted_angle_rad);
            float ex = x - pillars[p].x;
            float ey = y - pillars[p].y;
            float err = std::sqrt(ex * ex + ey * ey);
            if (err > TRACK_VALIDATE_ERR_MM) {
                best.found = false; // rejet outlier
            }
        } else {
            // Fallback: élargir la recherche angulaire, score simple distance + pénalité angulaire
            int extraWin = std::max(halfWin, static_cast<int>(std::ceil(TRACK_FALLBACK_EXTRA_SEARCH_DEG / resDeg)));
            float bestScore = 1e9f;
            int candIdx = -1;
            for (int di = -extraWin; di <= extraWin; ++di) {
                int idx = (idx_center + di + NUM_ANGLES) % NUM_ANGLES;
                float d = scan[idx];
                if (d <= 0.0f || d > TRACK_MAX_VALID_MM) continue;
                float ang = indexToAngle(idx);
                float da = std::fabs(ang - angle_ref);
                da = std::fmin(da, 2.0f * PI - da);
                float dd = std::fabs(d - dist_ref);
                // Pèse plus fort la distance, pénalise l'écart angulaire
                float score = dd + (da * 180.0f / PI) * TRACK_DEG_PENALTY_MM; // pénalité mm par degré
                if (score < bestScore) { bestScore = score; candIdx = idx; }
            }
            if (candIdx >= 0) {
                float coarse_angle = indexToAngle(candIdx);
                float refined_angle = findcenter(scan, coarse_angle, dist_ref);
                if (!(refined_angle >= 0.0f)) refined_angle = coarse_angle;
                int refined_idx = angleToIndex(refined_angle);
                float dsel = scan[refined_idx];
                if (dsel <= 0.0f) dsel = scan[candIdx];
                best = { refined_angle, dsel, true };

                // Validation monde
                float adjusted_angle_rad = best.angle + posrobot.angle;
                float x = posrobot.x + (best.distance + RAYON_PILIER) * std::cos(adjusted_angle_rad);
                float y = posrobot.y + (best.distance + RAYON_PILIER) * std::sin(adjusted_angle_rad);
                float ex = x - pillars[p].x;
                float ey = y - pillars[p].y;
                float err = std::sqrt(ex * ex + ey * ey);
                if (err > TRACK_VALIDATE_ERR_FALLBACK_MM) best.found = false; // seuil plus tolérant en fallback
            }
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
    // Objectif: trouver l'angle vers le CENTRE du pilier. Pour un cylindre, l'angle du point le plus proche
    // (sur le flanc face au lidar) est colinéaire avec le centre du pilier. On cherche donc le minimum local
    // de distance dans une petite fenêtre autour de l'angle attendu, puis on affine par interpolation parabolique.

    if (distance <= 0.0f) return -1.0f; // invalide

    // 1) Fenêtre angulaire adaptative autour de l'angle fourni
    const float resDeg = RESOLUTION;
    const float angleDeg = angle * 180.0f / PI;
    const int centerIdx = static_cast<int>(std::round(angleDeg / resDeg));

    // largeur apparente ≈ 2*atan((rayon + marge)/distance)
    float spanRad = 2.0f * std::atan2(RAYON_PILIER + TRACK_EXTRA_MARGIN_MM, std::max(100.0f, distance));
    float minSpan = TRACK_MIN_SPAN_DEG * (PI / 180.0f);
    float maxSpan = TRACK_MAX_SPAN_DEG * (PI / 180.0f);
    if (spanRad < minSpan) spanRad = minSpan;
    if (spanRad > maxSpan) spanRad = maxSpan;
    int halfWin = std::max(2, static_cast<int>(std::ceil((spanRad * 180.0f / PI) / resDeg)));

    // 2) Gating radial (tolérance) et recherche du minimum local dans la fenêtre (avec wrap-around)
    float gate_mm = std::max(TRACK_GATE_MIN_MM, TRACK_GATE_FRAC * distance);
    gate_mm = std::min(gate_mm, TRACK_GATE_MAX_MM);

    int bestIdx = -1;
    float bestDist = 1e9f;

    auto inGate = [&](float d){ return (d > 0.0f && d < TRACK_MAX_VALID_MM && std::fabs(d - distance) <= gate_mm); };

    for (int di = -halfWin; di <= halfWin; ++di) {
        int idx = (centerIdx + di) % NUM_ANGLES;
        if (idx < 0) idx += NUM_ANGLES;
        float d = scan[idx];
        if (!inGate(d)) continue;
        if (d < bestDist) { bestDist = d; bestIdx = idx; }
    }

    // Fallback: si aucun point ne passe le gate, on cherche le mini brut dans la fenêtre
    if (bestIdx < 0) {
        for (int di = -halfWin; di <= halfWin; ++di) {
            int idx = (centerIdx + di) % NUM_ANGLES;
            if (idx < 0) idx += NUM_ANGLES;
            float d = scan[idx];
            if (d <= 0.0f || d > TRACK_MAX_VALID_MM) continue;
            if (d < bestDist) { bestDist = d; bestIdx = idx; }
        }
    }

    if (bestIdx < 0) return -1.0f; // rien de pertinent trouvé

    // 3) Affinage sub-indice par interpolation parabolique (quadratic fit) autour du minimum discret
    int i0 = bestIdx;
    int iL = (i0 - 1 + NUM_ANGLES) % NUM_ANGLES;
    int iR = (i0 + 1) % NUM_ANGLES;
    float yL = scan[iL];
    float y0 = scan[i0];
    float yR = scan[iR];

    // Parabolic vertex offset (en indices): delta = 0.5*(yL - yR) / (yL - 2*y0 + yR)
    float denom = (yL - 2.0f * y0 + yR);
    float delta = 0.0f;
    if (std::fabs(denom) > 1e-6f) {
        delta = 0.5f * (yL - yR) / denom;
        // On borne l'offset à [-1,1] pour éviter les extrapolations absurdes
        if (delta < -1.0f) delta = -1.0f;
        if (delta >  1.0f) delta =  1.0f;
    }

    // 4) Conversion de l'indice affinée vers l'angle (radians)
    float refinedIdx = i0 + delta;
    float refinedDeg = refinedIdx * resDeg;
    float angle_rad  = refinedDeg * PI / 180.0f;
    // Normalisation [0, 2pi)
    while (angle_rad < 0.0f) angle_rad += 2.0f * PI;
    while (angle_rad >= 2.0f * PI) angle_rad -= 2.0f * PI;

    return angle_rad;
}


// ============================================================================
// FONCTIONS SOCKET SERVEUR SUPPRIMÉES
// On utilise maintenant le mode CLIENT (connexion dans main.cpp)
// Les anciennes fonctions try_accept_client() et init_socket() ne sont plus nécessaires
// ============================================================================


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
    auto t3 = high_resolution_clock::now();
    sl_result op_result = drv->grabScanDataHq(nodes, count);
    auto t4 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t4 - t3).count();
    //std::cout << "Durée acquisition scan: " << duration << " ms\n";
    if (SL_IS_OK(op_result)) {
        for (size_t pos = 0; pos < count; ++pos) {
            // Conversion angle Q14 -> degrés
            float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;

            // Conversion distance Q2 -> mm
            float distance = nodes[pos].dist_mm_q2 / 4.0f;

            // Qualité
            int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

            int index = static_cast<int>(std::round(angle / RESOLUTION));
            // je retourne le scan , 1° devient 359°
            index = (NUM_ANGLES - index) % NUM_ANGLES;
            // Seulement si qualité > 0
            if (quality > 0) {
                if (index >= NUM_ANGLES) index = 0; // sécurité pour 360°

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

