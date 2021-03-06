//
// Created by Athos Innocenti on 28/06/21.
//

#include "IcpManager.h"

IcpManager::IcpManager(const std::string &original_model, const std::string &transformed_model)
        : error(MAXFLOAT), time(MAXFLOAT), cloud_original(new pcl::PointCloud<PointType>),
          cloud_transformed(new pcl::PointCloud<PointType>), cloud_icp(new pcl::PointCloud<PointType>),
          transformation_matrix(Eigen::Matrix4d::Identity()), visualizer(Plotter()) {
    if (pcl::io::loadPLYFile(original_model, *cloud_original) < 0)
        PCL_ERROR("Error loading original cloud.");
    assert(cloud_original->size() != 0);
    std::cout << "Loaded original model = " << cloud_original->size() << " points" << std::endl;

    if (pcl::io::loadPLYFile(transformed_model, *cloud_transformed) < 0)
        PCL_ERROR("Error loading transformed cloud.");
    assert(cloud_transformed->size() != 0);
    std::cout << "Loaded transformed model = " << cloud_transformed->size() << " points\n" << std::endl;

    icp_iterations = 0;

    // If visualize is true plot result
    visualize = false;
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

void IcpManager::initialTransformation(double r_x, double r_y, double r_z, double t_x, double t_y, double t_z) {
    transformation_matrix(0, 0) = std::cos(r_z) * std::cos(r_y);
    transformation_matrix(0, 1) = (std::cos(r_z) * std::sin(r_y) * std::sin(r_x)) - (std::sin(r_z) * std::cos(r_x));
    transformation_matrix(0, 2) = (std::sin(r_z) * std::sin(r_x)) + (std::cos(r_z) * std::sin(r_y) * std::cos(r_x));
    transformation_matrix(1, 0) = std::sin(r_z) * std::cos(r_y);
    transformation_matrix(1, 1) = (std::cos(r_z) * std::cos(r_x)) + (std::sin(r_z) * std::sin(r_y) * std::sin(r_x));
    transformation_matrix(1, 2) = (std::sin(r_z) * std::sin(r_y) * std::cos(r_x)) - (std::cos(r_z) * std::sin(r_x));
    transformation_matrix(2, 0) = -1 * std::sin(r_y);
    transformation_matrix(2, 1) = std::cos(r_y) * std::sin(r_x);
    transformation_matrix(2, 2) = std::cos(r_y) * std::cos(r_x);
    transformation_matrix(0, 3) = t_x;
    transformation_matrix(1, 3) = t_y;
    transformation_matrix(2, 3) = t_z;
    std::cout << "APPLYING INITIAL RIGID TRANSFORMATION:" << std::endl;
    print4x4Matrix(transformation_matrix);
    pcl::transformPointCloud(*cloud_transformed, *cloud_icp, transformation_matrix);

    // Backup cloud_icp into cloud_transformed for later use (Plotter)
    *cloud_transformed = *cloud_icp;
}

void IcpManager::runIcp(int max_iter) {
    std::unique_ptr<IterativeClosestPoint> icp(new IterativeClosestPoint);
    icp->setMaximumIterations(max_iter);
    icp->setInputSource(cloud_icp);
    icp->setInputTarget(cloud_original);
    timer.tic();
    icp->align(*cloud_icp);
    time = timer.toc();
    icp_iterations = icp->nr_iterations_;
    std::cout << "Applied " << icp_iterations << " ICP iterations in " << time << " ms" << std::endl;
    assert(icp->hasConverged());
    error = icp->getFitnessScore();
    std::cout << "ICP has converged, fitness score = " << error << std::endl;
    std::cout << "RESULTING ICP TRANSFORMATION:" << std::endl;
    transformation_matrix = icp->getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);

    // Visualization = Viewports + Colors + Text + Camera (position orientation) + Size + Reference + KeyboardCallback
    if (visualize) {
        visualizer.setViewer(cloud_original, cloud_transformed, cloud_icp, icp->getMaximumIterations());
        while (!visualizer.getViewer().wasStopped())
            visualizer.getViewer().spinOnce();
        visualizer.getViewer().close();
    }
}

double IcpManager::getError() const {
    return error;
}

double IcpManager::getTime() const {
    return time;
}

int IcpManager::getIcpIterations() const {
    return icp_iterations;
}
