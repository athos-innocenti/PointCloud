//
// Created by Athos Innocenti on 28/06/21.
//

#ifndef ICP_ICPMANAGER_H
#define ICP_ICPMANAGER_H

#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

#include "Plotter.h"

typedef pcl::PointXYZ PointType;
typedef pcl::IterativeClosestPoint<PointType, PointType> IterativeClosestPoint;

class IcpManager {
public:
    IcpManager(const std::string &original_model, const std::string &transformed_model);

    virtual ~IcpManager();

    void runIcp(int max_iter);

    void initialTransformation(double rot_x, double rot_y, double rot_z, double trs_x, double trs_y, double trs_z);

    double getError() const;

    double getTime() const;

    int getIcpIterations() const;

private:
    static void print4x4Matrix(const Eigen::Matrix4d &matrix);

    int icp_iterations;

    double error;
    double time;

    pcl::PointCloud<PointType>::Ptr cloud_original;
    pcl::PointCloud<PointType>::Ptr cloud_transformed;
    pcl::PointCloud<PointType>::Ptr cloud_icp;

    pcl::console::TicToc timer;

    Eigen::Matrix4d transformation_matrix;

    Plotter visualizer;
    bool visualize;
};


#endif //ICP_ICPMANAGER_H
