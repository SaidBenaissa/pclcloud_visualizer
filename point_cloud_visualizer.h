//
// Created by Saïd Benaissa on 21/03/2023.
//

#ifndef PCLCLOUD_EXAMPLE_POINT_CLOUD_VISUALIZER_H
#define PCLCLOUD_EXAMPLE_POINT_CLOUD_VISUALIZER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

#endif //PCLCLOUD_EXAMPLE_POINT_CLOUD_VISUALIZER_H

