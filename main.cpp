#include <iostream>

#include <pcl/io/ply_io.h>

#include "IcpManager.h"

int main() {
    // ICP algorithm implementation with non rigid transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
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

    IcpManager manager(cloud_original, cloud_transformed);

    manager.initialTransformation(M_PI / 6, 0.7, 0.9, 1.2);
    int exitCode = manager.runIcp();

    return exitCode;
}
