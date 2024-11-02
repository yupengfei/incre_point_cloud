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
    
    // 设置相机视角，使XY平面水平，Z轴向上
    viewer->setCameraPosition(
        0, 0, 20,    // 相机位置：在z轴上方
        0, 0, 0,     // 相机视点：看向原点
        0, 1, 0      // 相机朝向：y轴定义为向上方向
    );
    
    // 或者使用更简单的方式
    // viewer->setCameraPosition(0, -1, 0, 0, 0, 1); // 第一组参数是相机位置，第二组是向上方向
    
    // 保持窗口显示
    while(!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return 0;
}