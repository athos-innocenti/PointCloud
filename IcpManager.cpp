//
// Created by Athos Innocenti on 28/06/21.
//

#include "IcpManager.h"

#include <utility>

IcpManager::IcpManager(pcl::PointCloud<PointType>::Ptr cloud_original,
                       pcl::PointCloud<PointType>::Ptr cloud_transformed, int max_iter)
        : error(MAXFLOAT), time(MAXFLOAT), cloud_icp(new pcl::PointCloud<PointType>),
          transformation_matrix(Eigen::Matrix4d::Identity()), visualizer(Plotter()) {
    this->cloud_original = std::move(cloud_original);
    this->cloud_transformed = std::move(cloud_transformed);
    icp_num_iter = max_iter;
}

IcpManager::~IcpManager() = default;

void IcpManager::print4x4Matrix(const Eigen::Matrix4d &matrix) {
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void IcpManager::initialTransformation(double theta, float t_x, float t_y, float t_z) {
    // Apply initial transformation matrix + Backup cloud_icp into cloud_transformed for later use
    transformation_matrix(0, 0) = std::cos(theta);
    transformation_matrix(0, 1) = -sin(theta);
    transformation_matrix(1, 0) = sin(theta);
    transformation_matrix(1, 1) = std::cos(theta);
    transformation_matrix(0, 3) = t_x;
    transformation_matrix(1, 3) = t_y;
    transformation_matrix(2, 3) = t_z;
    std::cout << "Applying rigid transformation to: cloud_original -> cloud_icp" << std::endl;
    print4x4Matrix(transformation_matrix);
    pcl::transformPointCloud(*cloud_transformed, *cloud_icp, transformation_matrix);
    *cloud_transformed = *cloud_icp;
}

bool IcpManager::runIcp() {
    // Run ICP algorithm
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaximumIterations(icp_num_iter);
    icp.setInputSource(cloud_icp);
    icp.setInputTarget(cloud_original);
    timer.tic();
    icp.align(*cloud_icp);
    time = timer.toc();
    std::cout << "Applied " << icp_num_iter << " ICP iterations in " << time << " ms" << std::endl;
    if (icp.hasConverged()) {
        error = icp.getFitnessScore();
        std::cout << "ICP has converged, fitness score = " << error << std::endl;
        std::cout << "ICP transformation " << icp_num_iter << " : cloud_icp -> cloud_original" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);
        // Visualization = Viewports + Colors + Text + Camera (position orientation) + Size + Reference + KeyboardCallback
        // NOT WORKING
        visualizer.setViewer(cloud_original, cloud_transformed, cloud_icp, icp_num_iter);
        return true;
    } else {
        PCL_ERROR("ICP has not converged.\n");
        return false;
    }
}

double IcpManager::getError() const {
    return error;
}

double IcpManager::getTime() const {
    return time;
}
