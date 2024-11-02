#include <iostream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <fstream>

namespace fs = std::filesystem;

int main() {
    std::string folder_path = "/home/yupengfei/Downloads/3D";
    const int start_frame = 1;
    const int end_frame = 420;
    float resolution = 0.1f;  // octree分辨率，根据需要调整
    
    std::ofstream outFile("pcd_reader.out");
    size_t total_points = 0;
    
    // 创建累积的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 创建octree
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree(resolution);
    
    for(int i = start_frame; i <= end_frame; i++) {
        size_t new_points = 0;
        std::string file_path = folder_path + "/frame_" + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
            outFile << "无法读取文件: " << file_path << std::endl;
            continue;
        }
        
        // 如果是第一帧，直接添加所有点
        if(i == start_frame) {
            *accumulated_cloud = *cloud;
            octree.setInputCloud(accumulated_cloud);
            octree.addPointsFromInputCloud();
            total_points = cloud->size();
        } else {
            // 切换octree缓冲区
            octree.switchBuffers();
            
            // 设置新的点云
            octree.setInputCloud(cloud);
            octree.addPointsFromInputCloud();
            
            // 获取新的点的索引
            std::vector<int> newPointIdxVector;
            octree.getPointIndicesFromNewVoxels(newPointIdxVector);
            
            // 将新点添加到累积的点云中
            new_points = newPointIdxVector.size();  // 记录新增点数
            for(const auto& idx : newPointIdxVector) {
                accumulated_cloud->push_back(cloud->points[idx]);
            }
            
            total_points += new_points;  // 更新总点数
        }
        
        outFile << "文件: frame_" << i << ".pcd"
                << " 新增点数: " << (i == start_frame ? cloud->size() : new_points)
                << " 累计总点数: " << accumulated_cloud->size() << std::endl;
    }
    
    // 保存最终的点云
    pcl::io::savePCDFile("accumulated_cloud.pcd", *accumulated_cloud);
    outFile << "\n最终总点数: " << total_points << std::endl;
    outFile.close();
    
    return 0;
}
