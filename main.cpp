#include <random>

#include "IcpManager.h"
#include "Performance.h"

// Initial rotation's angle from 0 to 2PI with angle's step of PI/6
#define ANGLE_STEP 12
// Number of tries for each angle
#define TRIES 10
// Maximum ICP iterations
#define MAX_ITERATIONS 100
// Random initial translation vector, different for each try, with max translation = 1.0
#define MAX_TRANSLATION 1.0
// Original point cloud (ends with 0)
#define ORIGINAL_MODEL "./model/coffee_mug_0.ply"
// Transformed point cloud (ends with 1 - moving point cloud with fewer points)
#define TRANSFORMED_MODEL "./model/coffee_mug_1.ply"

int main() {
    std::list<double> avg_error;
    std::list<double> avg_time;
    auto *performance = new Performance();

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<float> dist(0.0, std::nextafter(MAX_TRANSLATION, DBL_MAX));

    for (int theta_iter = 0; theta_iter <= ANGLE_STEP; theta_iter++) {
        double theta_radiant = (2 * M_PI * theta_iter) / ANGLE_STEP;
        std::cout << "ANGLE: " << theta_radiant * (180.0 / M_PI) << "\n" << std::endl;
        std::unique_ptr<std::list<double>> errors(new std::list<double>);
        std::unique_ptr<std::list<double>> times(new std::list<double>);
        for (int t = 1; t <= TRIES; t++) {
            std::cout << "ITERATION: " << t << std::endl;
            // Load point clouds and set maximum iterations
            std::unique_ptr<IcpManager> manager(new IcpManager(ORIGINAL_MODEL, TRANSFORMED_MODEL, MAX_ITERATIONS));

            // Apply initial transformation matrix
            manager->initialTransformation(theta_radiant, dist(mt), dist(mt), dist(mt));
            // Run ICP algorithm
            bool hasConverged = manager->runIcp();
            if (!hasConverged)
                return (-1);
            errors->push_back(manager->getError());
            times->push_back(manager->getTime());
        }
        // Calculate average error and time value
        avg_error.push_back(Performance::calculateAvg(*errors));
        avg_time.push_back(Performance::calculateAvg(*times));
    }
    // Store average values
    performance->storeData(avg_error, "./performance/error.csv");
    performance->storeData(avg_time, "./performance/time.csv");
    delete performance;

    return 0;
}
