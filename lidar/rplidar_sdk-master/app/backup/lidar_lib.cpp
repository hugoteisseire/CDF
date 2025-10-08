#include "lidar_lib.h"
#include "ImuOTOS.h"
#include <cmath>

const std::vector<position> pillars = {
    {3094, 50, 0},
    {3094, 1950, 0},
    {-94, 1000, 0}
};


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
        if (error > 250) {
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
    float angle_span = atan2(100.0f, distance) * 180.0f / PI; // angle de recherche en degrés (rayon 100 mm)
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