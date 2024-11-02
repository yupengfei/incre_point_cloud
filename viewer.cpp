#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
    // 读取点云，使用 PointXYZI 替代 PointXYZ
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile<pcl::PointXYZI>("accumulated_cloud.pcd", *cloud) == -1) {
        std::cout << "无法读取文件 accumulated_cloud.pcd" << std::endl;
        return -1;
    }

    // 创建视窗对象并给一个标题
    pcl::visualization::CloudViewer viewer("点云显示");

    // 显示点云
    viewer.showCloud(cloud);

    // 保持窗口显示
    while(!viewer.wasStopped()) {}

    return 0;
}