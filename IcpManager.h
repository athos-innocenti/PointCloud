//
// Created by Athos Innocenti on 28/06/21.
//

#ifndef ICP_ICPMANAGER_H
#define ICP_ICPMANAGER_H

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

#include "Plotter.h"

typedef pcl::PointXYZRGB PointType;

class IcpManager {
public:
    IcpManager(pcl::PointCloud<PointType>::Ptr cloud_original,
               pcl::PointCloud<PointType>::Ptr cloud_transformed, int max_iter);

    virtual ~IcpManager();

    bool runIcp();

    void initialTransformation(double theta, float t_x, float t_y, float t_z);

    double getError() const;

    double getTime() const;

private:
    static void print4x4Matrix(const Eigen::Matrix4d &matrix);

    int icp_num_iter;

    double error;
    double time;

    pcl::PointCloud<PointType>::Ptr cloud_original;
    pcl::PointCloud<PointType>::Ptr cloud_transformed;
    pcl::PointCloud<PointType>::Ptr cloud_icp;

    pcl::console::TicToc timer;

    Eigen::Matrix4d transformation_matrix;

    Plotter visualizer;
};


#endif //ICP_ICPMANAGER_H
