#include <random>

#include <pcl/io/ply_io.h>

#include "IcpManager.h"
#include "Performance.h"

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string original_model = "./model/table_0.ply";
    std::string transformed_model = "./model/table_1.ply";

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

    // Initial rotation's angle from 0 to 2PI with angle's step PI/12 = 15°
    // 50 tries for each angle
    // Random initial translation vector for each try with max_translation = 2.0
    int angle_step = 24;
    for (int theta_iter = 0; theta_iter <= angle_step; theta_iter++) {
        double theta_radiant = (2 * M_PI * theta_iter) / angle_step;
        std::cout << "ANGLE: " << theta_radiant * (180.0 / M_PI) << std::endl;
        std::list<double> errors;
        std::list<double> times;
        for (int iter = 1; iter <= 50; iter++) {
            std::cout << "ITERATION: " << iter << std::endl;
            auto *manager = new IcpManager(cloud_original, cloud_transformed, 300);

            std::random_device rd;
            std::mt19937 mt(rd());
            std::uniform_real_distribution<float> dist(0.0, std::nextafter(2.0, DBL_MAX));

            manager->initialTransformation(theta_radiant, dist(mt), dist(mt), dist(mt));
            bool hasConverged = manager->runIcp();
            if (!hasConverged)
                return (-1);
            errors.push_back(manager->getError());
            times.push_back(manager->getTime());
            delete manager;
        }
        // Calculate average error and time value
        avg_error.push_back(Performance::calculateAvg(errors));
        avg_time.push_back(Performance::calculateAvg(times));
    }
    // Store average values
    performance->storeData(avg_error, "./performance/error.csv");
    performance->storeData(avg_time, "./performance/time.csv");

    return 0;
}
