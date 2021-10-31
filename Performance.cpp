//
// Created by Athos Innocenti on 04/07/21.
//

#include "Performance.h"

// Radiant
#define RADIANT(count, step) ((2 * M_PI * count) / step)

Performance::Performance(int max_trs, int max_rot) : mt(rd()), trs_dist(0.0, std::nextafter(max_trs, DBL_MAX)),
                                                     rot_dist(0.0, std::nextafter(max_rot, DBL_MAX)) {}

Performance::~Performance() = default;

double Performance::calculateAvg(const std::list<double> &list) {
    assert(!list.empty());
    double avg = std::accumulate(std::begin(list), std::end(list), 0.0);
    avg /= static_cast<double>(list.size());
    return avg;
}

void Performance::storeData(const std::list<double> &list, const std::string &name_file) {
    assert(!list.empty());
    data_file.open(name_file);
    for (double value: list)
        data_file << value << "\n";
    data_file.close();
}

void Performance::errorPerAngle(const std::string &original_model, const std::string &transformed_model,
                                int tries_angle, int angle_step, int max_iterations) {
    std::list<double> avg_error;
    std::list<double> avg_time;
    for (int angle_iter = 0; angle_iter <= angle_step; angle_iter++) {
        std::cout << "ANGLE: " << RADIANT(angle_iter, angle_step) * (180.0 / M_PI) << "\n" << std::endl;
        std::unique_ptr<std::list<double>> errors(new std::list<double>);
        std::unique_ptr<std::list<double>> times(new std::list<double>);
        for (int t = 1; t <= tries_angle; t++) {
            std::cout << "TRY: " << t << std::endl;
            // Load point clouds and set maximum iterations
            std::unique_ptr<IcpManager> manager(new IcpManager(original_model, transformed_model, max_iterations));

            // Apply initial transformation matrix (1D rotation + 3D translation)
            manager->initialTransformation(0, 0, RADIANT(angle_iter, angle_step),
                                           trs_dist(mt), trs_dist(mt), trs_dist(mt));
            // Run ICP algorithm
            manager->runIcp();

            errors->push_back(manager->getError());
            times->push_back(manager->getTime());
        }
        // Calculate average error and time value
        avg_error.push_back(calculateAvg(*errors));
        avg_time.push_back(calculateAvg(*times));
    }
    // Store average values
    storeData(avg_error, "./performance/error.csv");
    storeData(avg_time, "./performance/time.csv");
}

void Performance::errorPerIter(const std::string &original_model, const std::string &transformed_model,
                               int tries_iter, const std::list<int> &max_iter) {
    std::list<std::list<double>> error;
    std::list<std::list<double>> time;
    for (int t = 1; t <= tries_iter; t++) {
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
            std::unique_ptr<IcpManager> manager(new IcpManager(original_model, transformed_model, iter));

            // Apply initial transformation matrix (3D rotation + 3D translation)
            manager->initialTransformation(rot_x, rot_y, rot_z, trs_x, trs_y, trs_z);
            // Run ICP algorithm
            manager->runIcp();

            iter_errors->push_back(manager->getError());
            iter_times->push_back(manager->getTime());
        }
        error.push_back(*iter_errors);
        time.push_back(*iter_times);
    }
}