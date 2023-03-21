//
// Created by Saïd Benaissa on 21/03/2023.
//

#include "point_cloud_generator.h"

#include "point_cloud_generator.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <random>

void generateRandomPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill the point cloud with random points
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-10.0, 10.0);

    for (std::size_t i = 0; i < 1000; ++i) {
        pcl::PointXYZ point;
        point.x = dist(gen);
        point.y = dist(gen);
        point.z = dist(gen);
        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Save the point cloud to a PCD file
    pcl::io::savePCDFileASCII(filename, *cloud);
}
