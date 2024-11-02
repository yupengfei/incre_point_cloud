#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
    // 读取点云，使用 PointXYZI 替代 PointXYZ
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile<pcl::PointXYZI>("accumulated_cloud.pcd", *cloud) == -1) {
        std::cout << "无法读取文件 accumulated_cloud.pcd" << std::endl;
        return -1;
    }

    // 创建视窗对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0, 0, 0);  // 设置背景色为黑色
    
    // 创建颜色处理对象
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>::Ptr color_handler(
        new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(cloud, "intensity"));
    
    // 添加点云并设置显示属性
    viewer->addPointCloud<pcl::PointXYZI>(cloud, *color_handler, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    
    // 添加坐标系以便于观察
    viewer->addCoordinateSystem(1.0);
    
    // 设置相机参数
    viewer->initCameraParameters();
    
    // 保持窗口显示
    while(!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}