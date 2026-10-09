"""
Test rapide de la méthode move_distance()
Exécutez ce script pour vérifier que la nouvelle fonctionnalité fonctionne.
"""

import time
import can
from core.motor_controller import Motor
from mks_servo_can import MksServo

def test_move_distance():
    """Test simple de la méthode move_distance()."""
    
    print("=" * 60)
    print("TEST DE LA MÉTHODE move_distance()")
    print("=" * 60)
    
    # Configuration CAN
    print("\n[1/5] Configuration du bus CAN...")
    bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
    notifier = can.Notifier(bus, [])
    bus.socket.setblocking(False)
    print("✅ Bus CAN initialisé")
    
    # Création du moteur avec paramètres mécaniques
    print("\n[2/5] Création du moteur avec paramètres mécaniques...")
    servo1 = MksServo(bus, notifier, 2)
    motor1 = Motor(
        bus=bus,
        can_id=1,
        mks_servo=servo1,
        enable_async_control=False,
        # Paramètres mécaniques (à adapter à votre système)
        wheel_diameter_mm=60.0,      # Roue de 60mm
        steps_per_revolution=200,    # NEMA 17/23
        subdivisions=16,             # Configuré sur le driver
        gear_ratio=1.0               # Pas de réducteur
    )
    print("✅ Moteur créé")
    
    # Affichage des paramètres calculés
    print("\n[3/5] Paramètres calculés automatiquement :")
    print(f"  - Diamètre roue       : {motor1.wheel_diameter_mm} mm")
    print(f"  - Périmètre roue      : {motor1.wheel_perimeter_mm:.2f} mm")
    print(f"  - Pas par tour        : {motor1.steps_per_revolution}")
    print(f"  - Subdivisions        : {motor1.subdivisions}")
    print(f"  - Rapport réduction   : {motor1.gear_ratio}")
    print(f"  - Pulses/tour roue    : {motor1.pulses_per_wheel_revolution}")
    print(f"  - Pulses/mm           : {motor1.pulses_per_mm:.2f}")
    print(f"  - mm/pulse            : {motor1.mm_per_pulse:.4f}")
    
    # Test des conversions
    print("\n[4/5] Test des conversions :")
    test_distances = [10, 50, 100, 188.5]  # 188.5 ≈ périmètre pour roue 60mm
    print("\n  Distance (mm) | Pulses | Retour (mm)")
    print("  " + "-" * 45)
    for dist in test_distances:
        pulses = motor1.distance_to_pulses(dist)
        back = motor1.pulses_to_distance(pulses)
        print(f"  {dist:12.1f}  | {pulses:6d} | {back:11.2f}")
    
    print("\n  Vitesse (mm/s) | RPM | Retour (mm/s)")
    print("  " + "-" * 45)
    test_speeds = [50, 100, 200, 500]
    for speed in test_speeds:
        rpm = motor1.speed_mm_s_to_rpm(speed)
        back = motor1.rpm_to_speed_mm_s(rpm)
        print(f"  {speed:13d}  | {rpm:3d} | {back:13.2f}")
    
    # Test de déplacement réel
    print("\n[5/5] Test de déplacement réel :")
    
    # Reset à zéro
    print("\n  📍 Reset de la position à zéro...")
    motor1.reset_zero()
    time.sleep(0.5)
    
    # Test 1 : Avancer de 100mm
    print("\n  📍 Test 1: Avancer de 100mm à 50mm/s")
    motor1.move_distance(distance_mm=100, speed_mm_s=50, acceleration=150)
    time.sleep(0.1)  # Petit délai pour démarrage
    motor1.wait_until_stopped(timeout=10.0)
    
    pos1 = motor1.read_position()
    if pos1 is not None:
        dist1 = motor1.pulses_to_distance(pos1)
        print(f"     Position: {pos1} pulses = {dist1:.2f} mm")
    else:
        print("     ⚠️ Impossible de lire la position")
    
    time.sleep(1)
    
    # Test 2 : Reculer de 50mm
    print("\n  📍 Test 2: Reculer de 50mm à 100 RPM")
    motor1.move_distance(distance_mm=-50, speed_rpm=100, acceleration=150)
    time.sleep(0.1)
    motor1.wait_until_stopped(timeout=10.0)
    
    pos2 = motor1.read_position()
    if pos2 is not None:
        dist2 = motor1.pulses_to_distance(pos2)
        print(f"     Position: {pos2} pulses = {dist2:.2f} mm")
        expected_dist = 100 - 50
        error = abs(dist2 - expected_dist)
        print(f"     Attendu: {expected_dist:.2f} mm | Erreur: {error:.2f} mm")
    else:
        print("     ⚠️ Impossible de lire la position")
    
    time.sleep(1)
    
    # Test 3 : Retour au zéro
    print("\n  📍 Test 3: Retour à la position zéro")
    if pos2 is not None:
        dist_to_zero = -motor1.pulses_to_distance(pos2)
        print(f"     Distance à parcourir: {dist_to_zero:.2f} mm")
        motor1.move_distance(distance_mm=dist_to_zero, speed_mm_s=80, acceleration=200)
        time.sleep(0.1)
        motor1.wait_until_stopped(timeout=10.0)
        
        final_pos = motor1.read_position()
        if final_pos is not None:
            final_dist = motor1.pulses_to_distance(final_pos)
            print(f"     Position finale: {final_pos} pulses = {final_dist:.2f} mm")
        else:
            print("     ⚠️ Impossible de lire la position finale")
    
    # Arrêt de sécurité
    print("\n  🛑 Arrêt du moteur...")
    motor1.stop(acceleration=150)
    time.sleep(0.5)
    
    print("\n" + "=" * 60)
    print("✅ TEST TERMINÉ")
    print("=" * 60)
    print("\nSi vous observez des erreurs de précision importantes,")
    print("vérifiez que les paramètres mécaniques correspondent à votre système.")


if __name__ == "__main__":
    try:
        test_move_distance()
    except KeyboardInterrupt:
        print("\n🛑 Test interrompu par l'utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur lors du test : {e}")
        import traceback
        traceback.print_exc()
