#include "ImuOTOS.h"
#include <iostream>
#include <thread>
#include <iomanip>
// Exemple d’utilisation
int main() {
    ImuOTOS imu("/dev/i2c-1", 0x17);
    if (!imu.openBus()) {
        std::cerr << "Erreur ouverture I2C\n";
        return 1;
    }

    if (!imu.reset()) {
        std::cerr << "Erreur reset\n";
        imu.closeBus();
        return 1;
    }

    if (!imu.calibrate(255)) {
        std::cerr << "Erreur calibrage\n";
        imu.closeBus();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3 * 255));

    if (!imu.enableSignalProcessing(true, true, true, true)) {
        std::cerr << "Erreur config signal\n";
        imu.closeBus();
        return 1;
    }
    

    // Test lecture et écriture du linear scalar
    float oldLinearScalar = imu.getLinearScalar();
    std::cout << "Linear scalar actuel : " << oldLinearScalar << std::endl;

    float newLinearScalar = 1.05f;
    if (imu.setLinearScalar(newLinearScalar)) {
        std::cout << "Linear scalar modifié à : " << newLinearScalar << std::endl;
        float checkLinearScalar = imu.getLinearScalar();
        std::cout << "Linear scalar relu : " << checkLinearScalar << std::endl;
    } else {
        std::cerr << "Erreur lors de la modification du linear scalar\n";
    }

    // Test lecture et écriture du angular scalar
    float oldAngularScalar = imu.getAngularScalar();
    std::cout << "Angular scalar actuel : " << oldAngularScalar << std::endl;

    float newAngularScalar = 1.02f;
    if (imu.setAngularScalar(newAngularScalar)) {
        std::cout << "Angular scalar modifié à : " << newAngularScalar << std::endl;
        float checkAngularScalar = imu.getAngularScalar();
        std::cout << "Angular scalar relu : " << checkAngularScalar << std::endl;
    } else {
        std::cerr << "Erreur lors de la modification du angular scalar\n";
    }

    while (true) {
        ImuPose pose;
        if (imu.readVelocity(pose)) {
            // je veut que cette affichage ait toujour 9 chiffres apres la virgule
            std::cout << std::fixed << std::setprecision(9);
            std::cout << "X: " << pose.x << ", Y: " << pose.y << ", H: " << pose.h << "\n";
        } else {
            std::cerr << "Erreur lecture vitesse\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // periodiquement remettre la position a zero
        static int counter = 0;     
        if (counter >= 50) { // toutes les 5 secondes
            counter = 0;
            ImuPose zeroPose = {0.0, 0.0, 0.0};
            if (!imu.writePose(zeroPose)) {
                std::cerr << "Erreur remise a zero position\n";
            } else {
                std::cout << "Position remise a zero\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    imu.closeBus();
    return 0;
}


