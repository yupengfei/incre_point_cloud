#include <iostream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <fstream>
#include <iomanip>
#include <pcl/filters/voxel_grid.h>

namespace fs = std::filesystem;

int main() {
    std::string folder_path = "/home/yupengfei/Downloads/3D";
    const int start_frame = 1;
    const int end_frame = 420;
    float resolution = 0.1f;  // 增大分辨率，使体素更大
    
    std::ofstream outFile("pcd_reader.out");
    size_t total_original_points = 0;
    size_t total_points = 0;
    
    // 创建累积的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 创建octree
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree(resolution);
    
    // 创建体素栅格过滤器
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(resolution, resolution, resolution);
    voxel_filter.setDownsampleAllData(true);
    voxel_filter.setMinimumPointsNumberPerVoxel(3);
    
    for(int i = start_frame; i <= end_frame; i++) {
        size_t new_points = 0;
        
        std::string file_path = folder_path + "/frame_" + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
            outFile << "无法读取文件: " << file_path << std::endl;
            continue;
        }
        
        // 对点云进行体素化降采样
        voxel_filter.setInputCloud(cloud);
        voxel_filter.filter(*cloud_filtered);
        
        // 然后再用octree检测新点
        if(i == start_frame) {
            *accumulated_cloud = *cloud_filtered;
            octree.setInputCloud(accumulated_cloud);
            octree.addPointsFromInputCloud();
            total_points = cloud_filtered->size();
        } else {
            octree.switchBuffers();
            octree.setInputCloud(cloud_filtered);
            octree.addPointsFromInputCloud();
            
            std::vector<int> newPointIdxVector;
            octree.getPointIndicesFromNewVoxels(newPointIdxVector);
            
            new_points = newPointIdxVector.size();
            for(const auto& idx : newPointIdxVector) {
                accumulated_cloud->push_back(cloud_filtered->points[idx]);
            }
            total_points += new_points;
        }
        
        total_original_points += cloud->size();
        
        outFile << "文件: frame_" << i << ".pcd"
                << " 原始点数: " << cloud->size()
                << " 降采样后点数: " << cloud_filtered->size()
                << " 新增点数: " << (i == start_frame ? cloud_filtered->size() : new_points)
                << " 累计总点数: " << accumulated_cloud->size()
                << " 降采样率: " << std::fixed << std::setprecision(2) 
                << (100.0 * (cloud->size() - cloud_filtered->size()) / cloud->size()) << "%" << std::endl;
    }
    
    // 保存最终的点云
    pcl::io::savePCDFile("accumulated_cloud.pcd", *accumulated_cloud);
    outFile << "\n统计信息：" << std::endl;
    outFile << "原始总点数: " << total_original_points << std::endl;
    outFile << "降采样后总点数: " << total_points << std::endl;
    outFile << "总体降采样率: " << std::fixed << std::setprecision(2) 
            << (100.0 * (total_original_points - total_points) / total_original_points) << "%" << std::endl;
    outFile.close();
    
    return 0;
}
