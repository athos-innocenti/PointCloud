//
// Created by Athos Innocenti on 28/06/21.
//

#ifndef ICP_PLOTTER_H
#define ICP_PLOTTER_H

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointType;

class Plotter {
public:
    Plotter();

    virtual ~Plotter();

    void setViewer(const pcl::PointCloud<PointType>::Ptr &cloud_original,
                   const pcl::PointCloud<PointType>::Ptr &cloud_transformed,
                   const pcl::PointCloud<PointType>::Ptr &cloud_icp, int icp_num_iter);

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* x);

    void update(const pcl::PointCloud<PointType>::Ptr &cloud_icp, const std::string &iter_cnt);

    pcl::visualization::PCLVisualizer getViewer() const;

    bool isNextIteration() const;

    void setNextIteration(bool nextIteration);

private:
    bool next_iteration;

    float background;
    float text;

    pcl::visualization::PCLVisualizer viewer;
};


#endif //ICP_PLOTTER_H
