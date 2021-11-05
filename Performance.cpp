//
// Created by Athos Innocenti on 04/07/21.
//

#include "Performance.h"

// Radiant
#define RADIANT(count, step) ((2 * M_PI * count) / step)

Performance::Performance(int max_trs, int max_rot) : mt(rd()), trs_dist(0.0, std::nextafter(max_trs, DBL_MAX)),
                                                     rot_dist(0.0, std::nextafter(max_rot, DBL_MAX)) {}

Performance::~Performance() = default;

template<typename T>
T Performance::calculateAvg(const std::list<T> &list) {
    assert(!list.empty());
    T avg = std::accumulate(std::begin(list), std::end(list), static_cast<T>(0));
    avg /= static_cast<T>(list.size());
    return avg;
}

template<typename T>
void Performance::storeData(const std::list<T> &list, const std::string &name_file) {
    assert(!list.empty());
    data_file.open(name_file);
    for (T value: list)
        data_file << value << "\n";
    data_file.close();
}

void Performance::errorPerAngle(const std::string &original_model, const std::string &transformed_model,
                                int tries_angle, int angle_step, int max_iterations) {
    std::list<double> avg_error;
    std::list<double> avg_time;
    std::list<int> avg_iter;
    for (int angle_iter = 0; angle_iter <= angle_step; angle_iter++) {
        std::cout << "ANGLE: " << RADIANT(angle_iter, angle_step) * (180.0 / M_PI) << "\n" << std::endl;
        std::unique_ptr<std::list<double>> errors(new std::list<double>);
        std::unique_ptr<std::list<double>> times(new std::list<double>);
        std::unique_ptr<std::list<int>> iterations(new std::list<int>);
        for (int t = 1; t <= tries_angle; t++) {
            std::cout << "TRY: " << t << std::endl;
            // Load point clouds and set maximum iterations
            std::unique_ptr<IcpManager> manager(new IcpManager(original_model, transformed_model));

            // Apply initial transformation matrix (1D rotation + 3D translation)
            manager->initialTransformation(RADIANT(angle_iter, angle_step), 0, 0,
                                           trs_dist(mt), trs_dist(mt), trs_dist(mt));
            // Run ICP algorithm
            manager->runIcp(max_iterations);

            errors->push_back(manager->getError());
            times->push_back(manager->getTime());
            iterations->push_back(manager->getIcpIterations());
        }
        // Calculate average error, time and num_iter value
        avg_error.push_back(calculateAvg(*errors));
        avg_time.push_back(calculateAvg(*times));
        avg_iter.push_back(calculateAvg(*iterations));
    }
    // Store average values
    storeData(avg_error, "./performance/error.csv");
    storeData(avg_time, "./performance/time.csv");
    storeData(avg_iter, "./performance/iter.csv");
}

void Performance::errorPerIter(const std::string &original_model, const std::string &transformed_model,
                               int tries_iter, const std::list<int> &max_iter) {
    std::list<std::list<double>> error;
    std::list<std::list<double>> time;
    for (int t = 0; t <= tries_iter; t++) {
        std::unique_ptr<std::list<double>> iter_errors(new std::list<double>);
        std::unique_ptr<std::list<double>> iter_times(new std::list<double>);
        std::cout << "TRY: " << t << std::endl;

        double rot_x = rot_dist(mt);
        double rot_y = rot_dist(mt);
        double rot_z = rot_dist(mt);

        double trs_x = trs_dist(mt);
        double trs_y = trs_dist(mt);
        double trs_z = trs_dist(mt);

        for (int iter: max_iter) {
            std::cout << "MAX ICP ITERATIONS: " << iter << std::endl;
            // Load point clouds and set maximum iterations
            std::unique_ptr<IcpManager> manager(new IcpManager(original_model, transformed_model));

            // Apply initial transformation matrix (3D rotation + 3D translation)
            manager->initialTransformation(rot_x, rot_y, rot_z, trs_x, trs_y, trs_z);
            // Run ICP algorithm
            manager->runIcp(iter);

            iter_errors->push_back(manager->getError());
            iter_times->push_back(manager->getTime());
        }
        error.push_back(*iter_errors);
        time.push_back(*iter_times);
    }
}