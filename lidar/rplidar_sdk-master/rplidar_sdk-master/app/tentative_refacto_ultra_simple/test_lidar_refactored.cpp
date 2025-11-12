/**
 * @file test_lidar_refactored.cpp
 * @brief Exemples de tests unitaires pour le code refactorisé
 * 
 * Utilise Google Test Framework
 * Compilation : g++ test_lidar_refactored.cpp lidar_lib_refactored.cpp -lgtest -lgtest_main -pthread
 * Exécution : ./test_lidar_refactored
 */

#include <gtest/gtest.h>
#include "robot_config.h"
#include "lidar_lib_refactored.h"
#include <cmath>

// ============================================================================
// TESTS DE CONFIGURATION
// ============================================================================

class ConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup avant chaque test
    }
};

TEST_F(ConfigTest, TeamEnumValues) {
    EXPECT_EQ(static_cast<int>(Team::TEAM0), 0);
    EXPECT_EQ(static_cast<int>(Team::TEAM1), 1);
    EXPECT_EQ(static_cast<int>(Team::DEBUG), 3);
}

TEST_F(ConfigTest, IntToTeamConversion) {
    EXPECT_EQ(intToTeam(0), Team::TEAM0);
    EXPECT_EQ(intToTeam(1), Team::TEAM1);
    EXPECT_EQ(intToTeam(3), Team::DEBUG);
    EXPECT_EQ(intToTeam(999), Team::TEAM0); // Valeur invalide → défaut
}

TEST_F(ConfigTest, PillarPositionsTeam0) {
    const auto& pillars = getPillarPositions(Team::TEAM0);
    ASSERT_EQ(pillars.size(), 3);
    
    // Vérifier coin bas-droit
    EXPECT_FLOAT_EQ(pillars[0].x_mm, 2950.0f);
    EXPECT_FLOAT_EQ(pillars[0].y_mm, 50.0f);
    
    // Vérifier coin haut-droit
    EXPECT_FLOAT_EQ(pillars[1].x_mm, 2950.0f);
    EXPECT_FLOAT_EQ(pillars[1].y_mm, 1950.0f);
    
    // Vérifier milieu gauche
    EXPECT_FLOAT_EQ(pillars[2].x_mm, 50.0f);
    EXPECT_FLOAT_EQ(pillars[2].y_mm, 1000.0f);
}

TEST_F(ConfigTest, PillarPositionsTeam1) {
    const auto& pillars = getPillarPositions(Team::TEAM1);
    ASSERT_EQ(pillars.size(), 3);
    
    // Vérifier symétrie par rapport à Team0
    EXPECT_FLOAT_EQ(pillars[0].x_mm, 50.0f);
    EXPECT_FLOAT_EQ(pillars[0].y_mm, 50.0f);
}

TEST_F(ConfigTest, RobotStartPositions) {
    Position p0 = getRobotStartPosition(Team::TEAM0);
    EXPECT_FLOAT_EQ(p0.x_mm, 2200.0f);
    EXPECT_FLOAT_EQ(p0.y_mm, 1000.0f);
    EXPECT_FLOAT_EQ(p0.angle_rad, 0.0f);
    
    Position p1 = getRobotStartPosition(Team::TEAM1);
    EXPECT_FLOAT_EQ(p1.x_mm, 800.0f);
    EXPECT_FLOAT_EQ(p1.angle_rad, PI);
    
    Position p3 = getRobotStartPosition(Team::DEBUG);
    EXPECT_FLOAT_EQ(p3.x_mm, 750.0f);
    EXPECT_FLOAT_EQ(p3.angle_rad, PI / 2.0f);
}

// ============================================================================
// TESTS DE CONVERSION
// ============================================================================

class ConversionTest : public ::testing::Test {};

TEST_F(ConversionTest, PositionToImuPose) {
    Position pos = {1000.0f, 2000.0f, PI / 4.0f};
    ImuPose imu = positionToImuPose(pos);
    
    EXPECT_FLOAT_EQ(imu.x, 1.0f);  // 1000 mm → 1 m
    EXPECT_FLOAT_EQ(imu.y, 2.0f);  // 2000 mm → 2 m
    EXPECT_FLOAT_EQ(imu.h, PI / 4.0f);
}

TEST_F(ConversionTest, PolarToCartesian) {
    // Distance 100mm, angle 0° (vers +X)
    TrackResult tr1 = {0.0f, 100.0f, true};
    Position cart1 = polarToCartesian(tr1);
    EXPECT_NEAR(cart1.x_mm, 100.0f, 0.1f);
    EXPECT_NEAR(cart1.y_mm, 0.0f, 0.1f);
    
    // Distance 100mm, angle 90° (vers +Y)
    TrackResult tr2 = {PI / 2.0f, 100.0f, true};
    Position cart2 = polarToCartesian(tr2);
    EXPECT_NEAR(cart2.x_mm, 0.0f, 0.1f);
    EXPECT_NEAR(cart2.y_mm, 100.0f, 0.1f);
    
    // Distance 141.4mm, angle 45° (diagonale)
    TrackResult tr3 = {PI / 4.0f, 141.4f, true};
    Position cart3 = polarToCartesian(tr3);
    EXPECT_NEAR(cart3.x_mm, 100.0f, 0.5f);
    EXPECT_NEAR(cart3.y_mm, 100.0f, 0.5f);
}

// ============================================================================
// TESTS D'INITIALISATION
// ============================================================================

class InitializationTest : public ::testing::Test {};

TEST_F(InitializationTest, InitPillarsTeam0) {
    std::vector<Position> pillars = initPillars(Team::TEAM0);
    ASSERT_EQ(pillars.size(), 3);
    
    // Les positions doivent correspondre à PILLARS_TEAM0
    for (size_t i = 0; i < 3; ++i) {
        const auto& ref = getPillarPositions(Team::TEAM0)[i];
        EXPECT_FLOAT_EQ(pillars[i].x_mm, ref.x_mm);
        EXPECT_FLOAT_EQ(pillars[i].y_mm, ref.y_mm);
    }
}

TEST_F(InitializationTest, InitTrackedPoints) {
    Position robotPos = {1500.0f, 1000.0f, 0.0f};
    std::vector<Position> pillars = {
        {2950.0f, 50.0f, 0.0f},
        {2950.0f, 1950.0f, 0.0f},
        {50.0f, 1000.0f, 0.0f}
    };
    
    std::vector<TrackResult> tracked = initTrackedPoints(robotPos, pillars);
    ASSERT_EQ(tracked.size(), 3);
    
    // Tous les points doivent être marqués "found"
    for (const auto& tp : tracked) {
        EXPECT_TRUE(tp.found);
        EXPECT_GT(tp.distance_mm, 0.0f);
    }
    
    // Vérifier distance au pilier 1 (2950, 50)
    float dx = 2950.0f - 1500.0f;
    float dy = 50.0f - 1000.0f;
    float expected_dist = std::sqrt(dx*dx + dy*dy);
    EXPECT_NEAR(tracked[0].distance_mm, expected_dist, 1.0f);
}

// ============================================================================
// TESTS DE CALCUL DE POSE
// ============================================================================

class PoseComputationTest : public ::testing::Test {
protected:
    std::vector<Position> pillars;
    
    void SetUp() override {
        // Configuration simple : 3 piliers en triangle
        pillars = {
            {0.0f, 0.0f, 0.0f},
            {1000.0f, 0.0f, 0.0f},
            {500.0f, 866.0f, 0.0f}  // Triangle équilatéral
        };
    }
};

TEST_F(PoseComputationTest, TwoPillarsDetected) {
    // Robot à (500, 300), orientation 0
    // Simule 2 piliers détectés
    std::vector<TrackResult> measurements = {
        {std::atan2(0.0f - 300.0f, 0.0f - 500.0f), 
         std::sqrt(500.0f*500.0f + 300.0f*300.0f), true},
        {std::atan2(0.0f - 300.0f, 1000.0f - 500.0f),
         std::sqrt(500.0f*500.0f + 300.0f*300.0f), true},
        {0.0f, 0.0f, false}  // 3ème pilier non détecté
    };
    
    Position pose = computeRobotPose(measurements, pillars);
    
    // La pose calculée doit être proche de (500, 300)
    EXPECT_NEAR(pose.x_mm, 500.0f, 50.0f);
    EXPECT_NEAR(pose.y_mm, 300.0f, 50.0f);
}

TEST_F(PoseComputationTest, NotEnoughPillars) {
    // Seulement 1 pilier détecté
    std::vector<TrackResult> measurements = {
        {0.0f, 100.0f, true},
        {0.0f, 0.0f, false},
        {0.0f, 0.0f, false}
    };
    
    Position pose = computeRobotPose(measurements, pillars);
    
    // Doit retourner position nulle (pas assez de données)
    EXPECT_FLOAT_EQ(pose.x_mm, 0.0f);
    EXPECT_FLOAT_EQ(pose.y_mm, 0.0f);
    EXPECT_FLOAT_EQ(pose.angle_rad, 0.0f);
}

// ============================================================================
// TESTS DE STRUCTURES D'ÉTAT (sans dépendances matérielles)
// ============================================================================

class StateStructuresTest : public ::testing::Test {};

TEST_F(StateStructuresTest, RobotStateHistory) {
    // Créer un RobotState et tester l'historique
    struct MockRobotState {
        std::deque<Position> posHistory;
        static const size_t HISTORY_SIZE = 5;
        
        void addToHistory(const Position& pos) {
            posHistory.push_back(pos);
            if (posHistory.size() > HISTORY_SIZE) {
                posHistory.pop_front();
            }
        }
        
        Position getSmoothedPosition() const {
            if (posHistory.empty()) return {0, 0, 0};
            
            float sum_x = 0, sum_y = 0, sum_angle = 0;
            for (const auto& p : posHistory) {
                sum_x += p.x_mm;
                sum_y += p.y_mm;
                sum_angle += p.angle_rad;
            }
            
            float n = static_cast<float>(posHistory.size());
            return {sum_x / n, sum_y / n, sum_angle / n};
        }
    };
    
    MockRobotState state;
    
    // Ajouter 5 positions identiques
    for (int i = 0; i < 5; ++i) {
        state.addToHistory({100.0f, 200.0f, 0.0f});
    }
    
    Position smoothed = state.getSmoothedPosition();
    EXPECT_FLOAT_EQ(smoothed.x_mm, 100.0f);
    EXPECT_FLOAT_EQ(smoothed.y_mm, 200.0f);
    
    // Ajouter une 6ème position (doit éjecter la 1ère)
    state.addToHistory({200.0f, 300.0f, 0.0f});
    EXPECT_EQ(state.posHistory.size(), 5);
    
    smoothed = state.getSmoothedPosition();
    // Moyenne : 4x(100,200) + 1x(200,300) / 5 = (120, 220)
    EXPECT_NEAR(smoothed.x_mm, 120.0f, 0.1f);
    EXPECT_NEAR(smoothed.y_mm, 220.0f, 0.1f);
}

TEST_F(StateStructuresTest, LocalizationStateDetectionCount) {
    struct MockLocalizationState {
        std::vector<TrackResult> trackedPoints;
        
        int countDetectedPillars() const {
            int count = 0;
            for (const auto& tp : trackedPoints) {
                if (tp.found) count++;
            }
            return count;
        }
    };
    
    MockLocalizationState state;
    state.trackedPoints = {
        {0.0f, 100.0f, true},
        {1.0f, 200.0f, false},
        {2.0f, 300.0f, true}
    };
    
    EXPECT_EQ(state.countDetectedPillars(), 2);
}

// ============================================================================
// TESTS DE VALIDATION
// ============================================================================

class ValidationTest : public ::testing::Test {};

TEST_F(ValidationTest, PillarPositionTolerance) {
    // Test de la tolérance de détection de piliers
    constexpr float tolerance = PILLAR_POSITION_TOLERANCE_MM;
    
    Position pillar = {1000.0f, 1000.0f, 0.0f};
    Position detected1 = {1200.0f, 1200.0f, 0.0f};
    Position detected2 = {1500.0f, 1500.0f, 0.0f};
    
    float error1 = std::sqrt(
        std::pow(detected1.x_mm - pillar.x_mm, 2) +
        std::pow(detected1.y_mm - pillar.y_mm, 2)
    );
    
    float error2 = std::sqrt(
        std::pow(detected2.x_mm - pillar.x_mm, 2) +
        std::pow(detected2.y_mm - pillar.y_mm, 2)
    );
    
    EXPECT_LT(error1, tolerance);  // 282mm < 450mm → OK
    EXPECT_GT(error2, tolerance);  // 707mm > 450mm → Rejet
}

TEST_F(ValidationTest, TableBoundaries) {
    // Vérifier que les positions sont dans la table
    Position valid = {1500.0f, 1000.0f, 0.0f};
    Position invalid1 = {-100.0f, 1000.0f, 0.0f};
    Position invalid2 = {1500.0f, 2500.0f, 0.0f};
    
    auto isInTable = [](const Position& p) {
        return p.x_mm >= 0 && p.x_mm <= TABLE_WIDTH_MM &&
               p.y_mm >= 0 && p.y_mm <= TABLE_HEIGHT_MM;
    };
    
    EXPECT_TRUE(isInTable(valid));
    EXPECT_FALSE(isInTable(invalid1));
    EXPECT_FALSE(isInTable(invalid2));
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// ============================================================================
// NOTES D'UTILISATION
// ============================================================================

/*
Pour compiler et exécuter ces tests :

1. Installer Google Test :
   sudo apt-get install libgtest-dev
   cd /usr/src/gtest
   sudo cmake CMakeLists.txt
   sudo make
   sudo cp *.a /usr/lib

2. Compiler les tests :
   g++ -std=c++11 test_lidar_refactored.cpp lidar_lib_refactored.cpp \
       ImuOTOS.cpp -I. -I../../../sdk/include \
       -L../../../sdk/lib -lsl_lidar_sdk \
       -lgtest -lgtest_main -lpthread -o test_lidar_refactored

3. Exécuter :
   ./test_lidar_refactored

4. Exécuter un test spécifique :
   ./test_lidar_refactored --gtest_filter=ConfigTest.TeamEnumValues

5. Exécuter avec verbosité :
   ./test_lidar_refactored --gtest_color=yes

6. Générer rapport XML (pour CI) :
   ./test_lidar_refactored --gtest_output=xml:test_results.xml
*/
