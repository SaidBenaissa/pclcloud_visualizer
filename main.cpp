#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// My own header files
// Include the header files for the point cloud generator and visualizer
#include "point_cloud_generator.h"
#include "point_cloud_visualizer.h"

#ifdef __APPLE__
#include <dispatch/dispatch.h>
#endif

int main(int argc, char** argv) {
    // Generate a random point cloud
    generateRandomPointCloud("input_cloud.pcd");

    // Load input point cloud from a PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("input_cloud.pcd", *cloud) == -1) {
        std::cout << "Couldn't read input PCD file." << std::endl;
        return (-1);
    }
    std::cout << "Loaded point cloud with " << cloud->size() << " points." << std::endl;

    // Create the KD-Tree for searching
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Perform Euclidean cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // Set distance tolerance for clustering (2cm)
    ec.setMinClusterSize(100);    // Minimum number of points in a cluster
    ec.setMaxClusterSize(25000);  // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Iterate through clusters and save them as separate PCD files
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int index : indices.indices) {
            cluster->points.push_back(cloud->points[index]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        // Save the cluster as a PCD file
        std::stringstream ss;
        ss << "cluster_" << cluster_id << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cluster);
        std::cout << "Saved cluster " << cluster_id << " with " << cluster->points.size() << " points to " << ss.str() << std::endl;

        ++cluster_id;
    }

#ifdef __APPLE__
    // Visualize the point cloud on the main thread for macOS
    visualizePointCloud(cloud);
#else
    // Visualize the point cloud on a separate thread for other operating systems
    std::thread viewerThread(&visualizePointCloud, cloud);
    viewerThread.join();
#endif

    return 0;
}
