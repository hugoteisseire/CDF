#!/usr/bin/env python3
"""
Calculateur de paramètres pour move_distance()

Ce script vous aide à déterminer les paramètres mécaniques corrects
pour votre système et à prévisualiser les conversions.
"""

import math

def calculate_parameters():
    """Interface interactive pour calculer les paramètres."""
    
    print("=" * 70)
    print("CALCULATEUR DE PARAMÈTRES POUR move_distance()")
    print("=" * 70)
    
    # Collecte des paramètres
    print("\n📐 Entrez les paramètres de votre système :\n")
    
    # Diamètre de la roue
    while True:
        try:
            wheel_diameter_mm = float(input("Diamètre de la roue (mm) [défaut: 60]: ") or "60")
            if wheel_diameter_mm <= 0:
                print("❌ Le diamètre doit être positif")
                continue
            break
        except ValueError:
            print("❌ Veuillez entrer un nombre valide")
    
    # Pas par tour
    print("\nNombre de pas par tour du moteur :")
    print("  - NEMA 17/23 standard : 200 pas/tour (1.8° par pas)")
    print("  - Moteur 0.9° : 400 pas/tour")
    while True:
        try:
            steps_per_revolution = int(input("Pas par tour [défaut: 200]: ") or "200")
            if steps_per_revolution <= 0:
                print("❌ Le nombre de pas doit être positif")
                continue
            break
        except ValueError:
            print("❌ Veuillez entrer un nombre entier")
    
    # Subdivisions
    print("\nSubdivisions (micro-stepping) configurées sur le driver :")
    print("  - Valeurs courantes : 8, 16, 32, 64, 128")
    while True:
        try:
            subdivisions = int(input("Subdivisions [défaut: 16]: ") or "16")
            if subdivisions <= 0:
                print("❌ Les subdivisions doivent être positives")
                continue
            break
        except ValueError:
            print("❌ Veuillez entrer un nombre entier")
    
    # Rapport de réduction
    print("\nRapport de réduction (gear ratio) :")
    print("  - Pas de réducteur : 1.0")
    print("  - Réducteur 3:1 : 3.0")
    print("  - Réducteur 5:1 : 5.0")
    while True:
        try:
            gear_ratio = float(input("Rapport de réduction [défaut: 1.0]: ") or "1.0")
            if gear_ratio <= 0:
                print("❌ Le rapport doit être positif")
                continue
            break
        except ValueError:
            print("❌ Veuillez entrer un nombre valide")
    
    # Calculs
    print("\n" + "=" * 70)
    print("RÉSULTATS DES CALCULS")
    print("=" * 70)
    
    perimeter = math.pi * wheel_diameter_mm
    pulses_per_revolution = steps_per_revolution * subdivisions * gear_ratio
    pulses_per_mm = pulses_per_revolution / perimeter
    mm_per_pulse = perimeter / pulses_per_revolution
    
    print(f"\n📊 Constantes calculées :")
    print(f"  - Périmètre de la roue        : {perimeter:.2f} mm")
    print(f"  - Pulses par tour de roue     : {pulses_per_revolution:.0f} pulses")
    print(f"  - Résolution (pulses/mm)      : {pulses_per_mm:.2f} pulses/mm")
    print(f"  - Résolution (mm/pulse)       : {mm_per_pulse:.4f} mm/pulse")
    print(f"  - Résolution (microns/pulse)  : {mm_per_pulse * 1000:.1f} µm/pulse")
    
    # Exemples de conversions
    print(f"\n🔄 Exemples de conversions distance → pulses :")
    print(f"  {'Distance (mm)':>15} | {'Pulses':>10} | {'Retour (mm)':>15}")
    print(f"  {'-'*15}-+-{'-'*10}-+-{'-'*15}")
    
    test_distances = [1, 10, 50, 100, perimeter]
    for dist in test_distances:
        pulses = int(abs(dist) * pulses_per_mm)
        back = pulses * mm_per_pulse
        print(f"  {dist:15.2f} | {pulses:10d} | {back:15.2f}")
    
    # Exemples de vitesses
    print(f"\n🔄 Exemples de conversions vitesse mm/s → RPM :")
    print(f"  {'Vitesse (mm/s)':>15} | {'RPM':>10} | {'Retour (mm/s)':>15}")
    print(f"  {'-'*15}-+-{'-'*10}-+-{'-'*15}")
    
    test_speeds = [10, 50, 100, 200, 500]
    for speed in test_speeds:
        revolutions_per_s = speed / perimeter
        rpm = int(abs(revolutions_per_s) * 60 * gear_ratio)
        rpm = min(rpm, 3000)  # Limite à 3000 RPM
        back = (rpm / 60 / gear_ratio) * perimeter
        print(f"  {speed:15.1f} | {rpm:10d} | {back:15.1f}")
    
    # Vitesse maximale
    max_rpm = 3000
    max_speed = (max_rpm / 60 / gear_ratio) * perimeter
    print(f"\n⚡ Vitesse maximale :")
    print(f"  - RPM max             : {max_rpm} RPM")
    print(f"  - Vitesse linéaire max: {max_speed:.0f} mm/s ({max_speed/1000:.2f} m/s)")
    
    # Code Python généré
    print("\n" + "=" * 70)
    print("CODE PYTHON À UTILISER")
    print("=" * 70)
    print(f"""
motor = Motor(
    bus=bus,
    can_id=1,
    mks_servo=servo1,
    enable_async_control=False,
    # Paramètres mécaniques
    wheel_diameter_mm={wheel_diameter_mm},
    steps_per_revolution={steps_per_revolution},
    subdivisions={subdivisions},
    gear_ratio={gear_ratio}
)

# Exemple d'utilisation :
motor.move_distance(distance_mm=100, speed_mm_s=50, acceleration=150)
motor.wait_until_stopped()
""")
    
    # Conseils de calibration
    print("=" * 70)
    print("CONSEILS DE CALIBRATION")
    print("=" * 70)
    print(f"""
1. Test de périmètre :
   - Marquer la position de départ sur la roue
   - Faire exactement 1 tour : move_relative(pulses={pulses_per_revolution:.0f}, ...)
   - Mesurer la distance réelle parcourue
   - Elle devrait être ≈ {perimeter:.1f} mm

2. Test de précision :
   - Faire parcourir exactement 1000 mm
   - Mesurer physiquement la distance réelle
   - Calculer l'erreur en %
   - Si erreur > 2%, vérifier le diamètre de la roue

3. Calibration fine :
   - Si la distance réelle est différente, ajuster wheel_diameter_mm
   - Nouveau diamètre = ancien_diamètre × (distance_mesurée / distance_théorique)

4. Points d'attention :
   - Vérifier qu'il n'y a pas de glissement des roues
   - Mesurer le diamètre avec un pied à coulisse précis
   - Tenir compte de la déformation sous charge
""")
    
    print("\n✅ Calculs terminés !")
    print("=" * 70)


def quick_conversion():
    """Mode de conversion rapide."""
    print("\n" + "=" * 70)
    print("MODE CONVERSION RAPIDE")
    print("=" * 70)
    
    # Paramètres par défaut
    wheel_diameter_mm = 60.0
    steps_per_revolution = 200
    subdivisions = 16
    gear_ratio = 1.0
    
    perimeter = math.pi * wheel_diameter_mm
    pulses_per_revolution = steps_per_revolution * subdivisions * gear_ratio
    pulses_per_mm = pulses_per_revolution / perimeter
    mm_per_pulse = perimeter / pulses_per_revolution
    
    print(f"\nParamètres utilisés : roue {wheel_diameter_mm}mm, {steps_per_revolution} pas, {subdivisions}x, ratio {gear_ratio}")
    print("(Utilisez le mode complet pour personnaliser)\n")
    
    while True:
        print("\nConversions disponibles :")
        print("  1. Distance (mm) → Pulses")
        print("  2. Pulses → Distance (mm)")
        print("  3. Vitesse (mm/s) → RPM")
        print("  4. RPM → Vitesse (mm/s)")
        print("  0. Quitter")
        
        choice = input("\nVotre choix : ")
        
        if choice == "0":
            break
        elif choice == "1":
            try:
                dist = float(input("Distance (mm) : "))
                pulses = int(abs(dist) * pulses_per_mm)
                print(f"  → {pulses} pulses")
            except ValueError:
                print("❌ Valeur invalide")
        elif choice == "2":
            try:
                pulses = int(input("Pulses : "))
                dist = pulses * mm_per_pulse
                print(f"  → {dist:.2f} mm")
            except ValueError:
                print("❌ Valeur invalide")
        elif choice == "3":
            try:
                speed = float(input("Vitesse (mm/s) : "))
                rpm = int(abs(speed / perimeter) * 60 * gear_ratio)
                rpm = min(rpm, 3000)
                print(f"  → {rpm} RPM")
            except ValueError:
                print("❌ Valeur invalide")
        elif choice == "4":
            try:
                rpm = int(input("RPM : "))
                speed = (rpm / 60 / gear_ratio) * perimeter
                print(f"  → {speed:.1f} mm/s")
            except ValueError:
                print("❌ Valeur invalide")
        else:
            print("❌ Choix invalide")


def main():
    """Menu principal."""
    print("\n" + "=" * 70)
    print("   CALCULATEUR DE PARAMÈTRES - move_distance()")
    print("=" * 70)
    
    while True:
        print("\nModes disponibles :")
        print("  1. Calculateur complet (recommandé)")
        print("  2. Conversions rapides")
        print("  0. Quitter")
        
        choice = input("\nVotre choix : ")
        
        if choice == "0":
            print("\n👋 Au revoir !")
            break
        elif choice == "1":
            calculate_parameters()
        elif choice == "2":
            quick_conversion()
        else:
            print("❌ Choix invalide")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Programme interrompu")
    except Exception as e:
        print(f"\n❌ Erreur : {e}")
        import traceback
        traceback.print_exc()
