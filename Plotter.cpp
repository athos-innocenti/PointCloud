//
// Created by Athos Innocenti on 28/06/21.
//

#include "Plotter.h"

Plotter::Plotter() : viewer("ICP Demo") {
    background = 0.0;
    text = 1.0;
}

Plotter::~Plotter() = default;

void Plotter::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *x) {
    // If user press "s" close window
    if (event.getKeySym() == "s" && event.keyDown())
        viewer.close();
}

void Plotter::setViewer(const pcl::PointCloud<PointType>::Ptr &cloud_original,
                        const pcl::PointCloud<PointType>::Ptr &cloud_transformed,
                        const pcl::PointCloud<PointType>::Ptr &cloud_icp, int icp_num_iter) {
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    viewer.setBackgroundColor(background, background, background, v1);
    viewer.setBackgroundColor(background, background, background, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointType> org_clr(cloud_original, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> trn_clr(cloud_transformed, 20, 180, 20);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> icp_clr(cloud_icp, 180, 20, 20);

    viewer.addPointCloud(cloud_original, org_clr, "cloud_in_v1", v1);
    viewer.addPointCloud(cloud_original, org_clr, "cloud_in_v2", v2);
    viewer.addPointCloud(cloud_transformed, trn_clr, "cloud_tr_v1", v1);
    viewer.addPointCloud(cloud_icp, icp_clr, "cloud_icp_v2", v2);

    viewer.addText("White: Original point cloud\nGreen: Transformed point cloud", 10, 15, 16, text,
                   text, text, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, text, text,
                   text, "icp_info_2", v2);
    std::stringstream ss;
    ss << icp_num_iter;
    std::string iterations_cnt = "Number of iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, text, text, text, "iterations_cnt", v2);

    viewer.setCameraPosition(5.0, 5.0, 5.0, 0.5, 1.0, 0.5, 0);

    viewer.setSize(2400, 1500);

    viewer.addCoordinateSystem(1.0, "reference", v1);

    viewer.registerKeyboardCallback(&Plotter::keyboardEventOccurred, *this);
}

pcl::visualization::PCLVisualizer Plotter::getViewer() const {
    return viewer;
}
