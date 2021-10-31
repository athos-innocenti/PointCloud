//
// Created by Athos Innocenti on 04/07/21.
//

#ifndef ICP_PERFORMANCE_H
#define ICP_PERFORMANCE_H

#include <random>

#include "IcpManager.h"

class Performance {
public:
    Performance(int max_translation, int max_rotation);

    virtual ~Performance();

    void errorPerAngle(const std::string &original_model, const std::string &transformed_model,
                       int tries_angle, int angle_step, int max_iterations);

    void errorPerIter(const std::string &original_model, const std::string &transformed_model,
                      int tries_iter, const std::list<int> &max_iter);

private:
    static double calculateAvg(const std::list<double> &list);

    void storeData(const std::list<double> &list, const std::string &name_file);

    std::ofstream data_file;

    std::random_device rd;
    std::mt19937 mt;
    std::uniform_real_distribution<double> trs_dist;
    std::uniform_real_distribution<double> rot_dist;
};


#endif //ICP_PERFORMANCE_H
