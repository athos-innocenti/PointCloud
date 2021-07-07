#include <random>

#include <pcl/io/ply_io.h>

#include "IcpManager.h"
#include "Performance.h"

// Initial rotation's angle from 0 to 2PI with angle's step PI/12 = 15°
#define ANGLE_STEP 1
// 50 tries for each angle
#define TRIES 5
// ICP maximum iterations = 100
#define MAX_ITERATION 100
// Random initial translation vector for each try with max_translation = 1.0
#define MAX_TRANSLATION 1.0

typedef pcl::PointXYZRGB PointType;

int main() {
    pcl::PointCloud<PointType>::Ptr cloud_original(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_transformed(new pcl::PointCloud<PointType>);
    std::string original_model = "./model/office_chair_0.ply";
    std::string transformed_model = "./model/office_chair_1.ply";

    // Load point clouds
    if (pcl::io::loadPLYFile(original_model, *cloud_original) < 0) {
        PCL_ERROR("Error loading cloud %s .\n", &original_model);
        return (-1);
    }
    std::cout << "Loaded original model = " << cloud_original->size() << " points" << std::endl;
    if (pcl::io::loadPLYFile(transformed_model, *cloud_transformed) < 0) {
        PCL_ERROR("Error loading cloud %s .\n", &transformed_model);
        return (-1);
    }
    std::cout << "Loaded transformed model = " << cloud_transformed->size() << " points\n" << std::endl;

    std::list<double> avg_error;
    std::list<double> avg_time;
    auto *performance = new Performance();

    for (int theta_iter = 0; theta_iter < ANGLE_STEP; theta_iter++) {
        double theta_radiant = (2 * M_PI * theta_iter) / ANGLE_STEP;
        std::cout << "ANGLE: " << theta_radiant * (180.0 / M_PI) << std::endl;
        std::list<double> errors;
        std::list<double> times;
        for (int t = 1; t <= TRIES; t++) {
            std::cout << "ITERATION: " << t << std::endl;
            std::unique_ptr<IcpManager> manager(new IcpManager(cloud_original, cloud_transformed, MAX_ITERATION));

            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_real_distribution<float> dist(0.0, std::nextafter(MAX_TRANSLATION, DBL_MAX));

            // Apply initial transformation matrix + Backup cloud_icp into cloud_transformed for later use
            manager->initialTransformation(theta_radiant, dist(mt), dist(mt), dist(mt));
            // Run ICP algorithm
            bool hasConverged = manager->runIcp();
            if (!hasConverged)
                return (-1);
            errors.push_back(manager->getError());
            times.push_back(manager->getTime());
        }
        // Calculate average error and time value
        avg_error.push_back(Performance::calculateAvg(errors));
        avg_time.push_back(Performance::calculateAvg(times));
    }
    // Store average values
    performance->storeData(avg_error, "./performance/error.csv");
    performance->storeData(avg_time, "./performance/time.csv");
    delete performance;

    return 0;
}
