#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）

    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/gc/catkin_ws/src/hdl_localization/data/2.pcd", *cloud_in) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
        return (-1);
    }
    std::cout << "Loaded "
              << cloud_in->width * cloud_in->height
              << " data points from test_file.pcd with the following fields: "
              << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i = 0; i < cloud_in->points.size();i++)
    {
        pcl::PointXYZI point_insert ;
        point_insert.x = cloud_in->points[i].z;
        point_insert.y = cloud_in->points[i].x;
        point_insert.z = cloud_in->points[i].y;
        point_insert.intensity = cloud_in->points[i].intensity;

        cloud_out->push_back(point_insert);
    }

    pcl::io::savePCDFileASCII("/home/gc/catkin_ws/src/hdl_localization/data/3.pcd",*cloud_out);


    return (0);
}