//
// Created by Athos Innocenti on 28/06/21.
//

#ifndef ICP_ICPMANAGER_H
#define ICP_ICPMANAGER_H

#include <iostream>

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

#include "Plotter.h"

typedef pcl::PointXYZRGB PointType;

class IcpManager {
public:
    IcpManager(pcl::PointCloud<PointType>::Ptr cloud_original, pcl::PointCloud<PointType>::Ptr cloud_transformed);

    virtual ~IcpManager();

    int runIcp();

    static void print4x4Matrix(const Eigen::Matrix4d &matrix);

    void initialTransformation(double theta, float t_x, float t_y, float t_z);

private:
    int icp_num_iter;

    std::list<double> errors;
    std::list<double> times;

    ofstream errors_file;
    ofstream time_file;

    pcl::PointCloud<PointType>::Ptr cloud_original;
    pcl::PointCloud<PointType>::Ptr cloud_transformed;
    pcl::PointCloud<PointType>::Ptr cloud_icp;

    pcl::console::TicToc timer;

    Eigen::Matrix4d transformation_matrix;

    Plotter visualizer;
};


#endif //ICP_ICPMANAGER_H
