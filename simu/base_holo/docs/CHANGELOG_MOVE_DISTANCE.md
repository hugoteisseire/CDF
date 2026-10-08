# Résumé des modifications - Méthode `move_distance()`

## 🎯 Objectif
Ajouter la possibilité de déplacer un moteur d'une **distance linéaire précise** (en millimètres) au lieu de spécifier un nombre de pulses.

---

## ✅ Modifications apportées

### 1. **Classe `Motor` - Nouveaux paramètres du constructeur**

Ajout de 4 paramètres mécaniques dans `__init__()` :
- `wheel_diameter_mm` (float) : Diamètre de la roue en mm (défaut: 60.0)
- `steps_per_revolution` (int) : Pas par tour moteur (défaut: 200)
- `subdivisions` (int) : Subdivisions micro-stepping (défaut: 16)
- `gear_ratio` (float) : Rapport de réduction (défaut: 1.0)

**Calculs automatiques** :
- Périmètre de la roue : `π × diamètre`
- Pulses par tour de roue : `pas × subdivisions × ratio`
- Conversions `mm ↔ pulses`

### 2. **Nouvelles méthodes de conversion**

#### `_update_conversion_constants()` (privée)
Calcule et met à jour les constantes de conversion :
- `wheel_perimeter_mm`
- `pulses_per_wheel_revolution`
- `pulses_per_mm`
- `mm_per_pulse`

#### `distance_to_pulses(distance_mm: float) -> int`
Convertit une distance en millimètres vers un nombre de pulses.

#### `pulses_to_distance(pulses: int) -> float`
Convertit un nombre de pulses vers une distance en millimètres.

#### `speed_mm_s_to_rpm(speed_mm_s: float) -> int`
Convertit une vitesse linéaire (mm/s) vers une vitesse de rotation (RPM).

#### `rpm_to_speed_mm_s(rpm: int) -> float`
Convertit une vitesse de rotation (RPM) vers une vitesse linéaire (mm/s).

### 3. **Méthode principale : `move_distance()`**

```python
def move_distance(
    self,
    distance_mm: float,              # Distance à parcourir (+ ou -)
    speed_mm_s: Optional[float],     # Vitesse en mm/s (prioritaire)
    speed_rpm: Optional[int],        # Vitesse en RPM (alternatif)
    acceleration: int = 100,         # Accélération
    direction: Optional[int] = None  # Direction (auto si None)
) -> None
```

**Fonctionnalités** :
- Conversion automatique distance → pulses
- Conversion automatique vitesse mm/s → RPM
- Gestion automatique de la direction selon le signe
- Affichage des informations de déplacement
- Utilise `move_relative()` en interne

---

## 📝 Fichiers modifiés

### `motor_controller.py`
- ✅ Ajout des paramètres mécaniques au constructeur
- ✅ Ajout de `_update_conversion_constants()`
- ✅ Ajout de 4 méthodes de conversion
- ✅ Ajout de `move_distance()`
- **Lignes modifiées** : ~150 lignes ajoutées

### `example_motor_class.py`
- ✅ Ajout de `example_move_distance()` : Démonstration complète
- ✅ Ajout de `example_conversion_utilities()` : Tests des conversions
- ✅ Mise à jour des commentaires dans le programme principal
- **Lignes modifiées** : ~80 lignes ajoutées

### `README_MOVE_DISTANCE.md` (nouveau)
- ✅ Guide complet d'utilisation
- ✅ Explications des paramètres mécaniques
- ✅ Exemples pratiques
- ✅ Méthode de vérification de la précision
- ✅ Conseils de configuration

---

## 🚀 Utilisation rapide

```python
# Création du moteur avec paramètres mécaniques
motor1 = Motor(
    bus=bus,
    can_id=1,
    wheel_diameter_mm=60.0,    # À adapter à votre roue
    steps_per_revolution=200,
    subdivisions=16,
    gear_ratio=1.0
)

# Déplacement de 100mm à 50mm/s
motor1.move_distance(distance_mm=100, speed_mm_s=50)
motor1.wait_until_stopped()

# Retour en arrière de 50mm à 100 RPM
motor1.move_distance(distance_mm=-50, speed_rpm=100)
motor1.wait_until_stopped()
```

---

## 🔍 Formules de conversion

### Distance → Pulses
```
pulses = distance_mm × (pas_par_tour × subdivisions × ratio) / (π × diamètre)
```

### Vitesse mm/s → RPM
```
RPM = (vitesse_mm_s / périmètre_roue) × 60 × ratio
```

---

## ✨ Avantages

1. **Simplicité** : Plus besoin de calculer manuellement les pulses
2. **Précision** : Conversions automatiques basées sur les paramètres mécaniques
3. **Flexibilité** : Vitesse en mm/s ou RPM au choix
4. **Cohérence** : Direction automatiquement déduite du signe
5. **Traçabilité** : Affichage des informations de déplacement

---

## 🧪 Tests recommandés

1. **Test de calibration** : Vérifier qu'1 tour = périmètre de la roue
2. **Test de précision** : Mesurer physiquement plusieurs distances
3. **Test de répétabilité** : Effectuer plusieurs fois le même déplacement
4. **Test de direction** : Vérifier que les signes +/- fonctionnent correctement

---

## 📚 Documentation

- **Guide complet** : `README_MOVE_DISTANCE.md`
- **Exemples pratiques** : `example_motor_class.py` (fonctions `example_move_distance()` et `example_conversion_utilities()`)
- **Code source** : `motor_controller.py` (lignes ~200-360)

---

## 🔧 Configuration pour votre système

Pour adapter à votre matériel, modifiez ces paramètres lors de la création du moteur :

```python
motor = Motor(
    bus=bus,
    can_id=1,
    wheel_diameter_mm=XX,      # Mesurez avec un pied à coulisse
    steps_per_revolution=200,  # NEMA 17/23 = 200 (vérifiez datasheet)
    subdivisions=16,           # Configuré sur le driver MKS
    gear_ratio=1.0             # >1 si réducteur (ex: 3.0 pour 3:1)
)
```

---

## ⚠️ Points d'attention

- La précision dépend de l'adhérence des roues (pas de glissement)
- Vitesse automatiquement limitée à 3000 RPM
- Les conversions sont bidirectionnelles et cohérentes
- Compatible avec le mode synchrone ET asynchrone
