//
// Created by Saïd Benaissa on 21/03/2023.
//

#include "point_cloud_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <vtkAutoInit.h>
#include <dispatch/dispatch.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    dispatch_sync(dispatch_get_main_queue(), ^{
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}
