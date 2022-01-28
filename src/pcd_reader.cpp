#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 介绍了两种读取和保存到pcd格式点云的方法

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
 
  return (0);
}


// 1、使用reader

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	// Generate pointcloud data，新建指针cloud存放点云
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("jiange0.05liqun_100inliers4.pcd", *cloud);//读取pcd文件，用指针传递给cloud。
	
// 2、使用loadPCDFile

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	// Generate pointcloud data，新建指针cloud存放点云
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("resultjiange0.05liqun.pcd", *cloud) == -1)//*打开点云文件。
	{                                                                           
		PCL_ERROR("Couldn't read that pcd file\n");                         
		return(-1);//如果没找到该文件，返回-1，跳出整个main函数
	}
