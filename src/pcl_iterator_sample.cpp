#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
  	std::cout << "Loaded " << cloud->width * cloud->height << " points" << std::endl;

	//  for (size_t i = 0; i < cloud->points.size (); ++i)
	//      std::cout << "    " << cloud->points[i].x
	//  	          << " "    << cloud->points[i].y
	//  	          << " "    << cloud->points[i].z << std::endl;
	
	// for (size_t i = 0; i < cloud->points.size(); i++)
	for (auto it = cloud->begin(); it != cloud->end(); it++)
	{
		std::cout << it->z << std::endl;
		if (it->z < 2.0)
			// cloud->erase(it);
			new_cloud->push_back(*it);
	}

	pcl::io::savePCDFileASCII("new_cloud.pcd", *new_cloud);
            
    return (0);
}
