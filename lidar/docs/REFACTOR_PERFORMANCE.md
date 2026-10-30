# Refactorisation des performances de la localisation lidar

Branche : `refactor/ekf-performance`
Base : `main`

## Objectifs et invariants

Cette branche améliore les coûts CPU et mémoire du pipeline de localisation sans réécriture globale.
La branche `main` reste inchangée. Les changements doivent préserver :
- la convention de pose terrain (mm, radians, angle anti-horaire) ;
- les corrections lidar datées à l'instant réel de la mesure ;
- le traitement chronologique des tours et le rattrapage de file ;
- le comportement de rejet des mesures hors historique ou incohérentes ;
- les interfaces publiques utilisées par le robot et les outils de test.

## Plan de travail

| Élément | Évolution |
|---|---|
| Filtrage des points | Conserver le filtrage NumPy vectorisé déjà présent sur `main`. Éviter tout retour à un appel `state_at()` par point. |
| Historique EKF | Mesurer séparément prédiction, enregistrement, interpolation et correction. Réduire les copies et allocations. |
| Covariances | Étudier une représentation à mémoire bornée et moins coûteuse que le tenseur dense `N × N × 3 × 3`. Ne pas supprimer les covariances croisées sans démontrer l'équivalence des corrections retardées. |
| Corrections lidar | Mutualiser les opérations communes et évaluer une correction groupée des mesures compatibles. Garder le gating et les limites de correction. |
| Gestion temporelle | Formaliser les horloges, l'alignement epoch/monotone, la latence lidar et les responsabilités acquisition/fusion. |
| Diagnostics | Garder les temps par étape et ajouter les métriques EKF : taille d'historique, temps de prédiction, enregistrement et correction. |

## Ordre d'implémentation

1. Instrumenter sans changer les équations ni les décisions du filtre.
2. Établir un profil de référence sur PC et Raspberry Pi, robot immobile puis en mouvement.
3. Optimiser l'historique et les covariances avec tests numériques de non-régression.
4. Évaluer le regroupement des corrections lidar.
5. Clarifier les interfaces temporelles et documenter les contrats.
6. Comparer précision, taux d'acceptation, latence et mémoire avec la branche `main`.

## Critères d'acceptation

- Aucun changement non expliqué de pose, covariance, gating ou décision de relocalisation sur les scénarios de référence.
- Les corrections retardées restent cohérentes avec les mesures datées.
- Pas de croissance mémoire non bornée.
- Le coût de chaque étape est observable sans activer l'interface graphique.
- Les résultats sont comparés sur le même jeu de données et la même configuration.

## Points de vigilance

L'historique actuel conserve les covariances croisées entre chaque paire d'instants. Cette information permet de propager une correction retardée à tous les états historiques. Une simplification naïve en ne conservant que les covariances diagonales changerait le filtre. Toute évolution de cette structure doit donc être validée mathématiquement et par rejeu de données.
