#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

// 1.如何获取pcd文件点云里点的格式，比如是pcl::PointXYZ还是pcl::PointXYZRGB等类型？
int main(int argc, char **argv)
{
	pcl::PCLPointCloud2 cloud;
	pcl::PCDReader reader;
	reader.readHeader("bun0.pcd", cloud);
	for (int i = 0; i < cloud.fields.size(); i++)
	{
		std::cout << cloud.fields[i].name;
	}
}

// 2.如何实现类似pcl::PointCloud::Ptr和pcl::PointCloud的两个类相互转换？
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud = *cloudPointer;
	cloudPointer = cloud.makeShared();
}


// 3.PointCloud和PCLPointCloud2类型如何相互转换？
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud2;
	pcl::io::loadPCDFile("C:office3-after21111.pcd", cloud2);
	pcl::fromPCLPointCloud2(cloud2, *cloud);
	pcl::toPCLPointCloud2(*cloud, cloud2);
}

// 4.如何加快ASCII格式存储，也就是记事本打开可以看到坐标数据的pcd文件读取速度？建议将pcd文件换成以Binary格式存储。
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:office3-after1.pcd", *cloud);
	pcl::io::savePCDFileBinary("C:office3-after21111.pcd", *cloud);
}

// 5.如何给pcl::PointXYZ类型的点云在显示时指定颜色？
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:office3-after21111.pcd", *cloud);
	 
	pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sig(cloud, 0, 234, 0);
	viewer.addPointCloud(cloud, sig, "cloud");
	while (!viewer.wasStopped())
	{
	    viewer.spinOnce();
	}
}

// 6.如何查找点云的x，y，z的极值？
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bun0.pcd", *cloud);
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	std::cerr << " " << minPt.x << " " << minPt.y << " " << minPt.z << std::endl;
	std::cerr << " " << maxPt.x << " " << maxPt.y << " " << maxPt.z << std::endl;
}

// 7.如何判断点云中的点为无效点，即坐标值为nan，以及如何将点设置为无效点？
int main(int argc, char **argv)
{
	pcl::PointXYZ p_valid;
	p_valid.x = 0;
	p_valid.y = 0;
	p_valid.z = 0;
	std::cout << "Is p_valid valid? " << pcl::isFinite(p_valid) << std::endl;
 
	pcl::PointXYZ p_invalid;
	p_invalid.x = std::numeric_limits<float>::quiet_NaN();
	p_invalid.y = 0;
	p_invalid.z = 0;
	std::cout << "Is p_invalid valid? " << pcl::isFinite(p_invalid) << std::endl;
 
}

// 8.如何将无效点从点云中移除？
int main(int argc, char **argv)
{
	typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
	CloudType::Ptr cloud(new CloudType);
	cloud->is_dense = false;
	CloudType::Ptr output_cloud(new CloudType);
 
	CloudType::PointType p_nan;
	p_nan.x = std::numeric_limits<float>::quiet_NaN();
	p_nan.y = std::numeric_limits<float>::quiet_NaN();
	p_nan.z = std::numeric_limits<float>::quiet_NaN();
	cloud->push_back(p_nan);
 
	CloudType::PointType p_valid;
	p_valid.x = 1.0f;
	cloud->push_back(p_valid);
 
	std::cout << "size: " << cloud->points.size() << std::endl;
 
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output_cloud, indices);
	std::cout << "size: " << output_cloud->points.size() << std::endl;
}

// 9.如果知道需要保存点的序号，如何从原点云中拷贝点到新点云？
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bun0.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int > indexs = { 1, 2, 5 };
	pcl::copyPointCloud(*cloud, indexs, *cloudOut);
}

// 10.如何从点云里删除和添加点？
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bun0.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
	cloud->erase(index);//删除第一个
	index = cloud->begin() + 5;
	cloud->erase(cloud->begin());//删除第5个
	pcl::PointXYZ point = { 1, 1, 1 };
	//在索引号为5的位置1上插入一点，原来的点后移一位
	cloud->insert(cloud->begin() + 5, point);
	cloud->push_back(point);//从点云最后面插入一点
	std::cout << cloud->points[5].x;//输出1
}

// 11.如何对点云进行全局或局部的空间变换？
int main(int argc, char **argv)
{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("path/.pcd",*cloud);
	//全局变化
 	//构造变化矩阵
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        float theta = M_PI/4;   //旋转的度数，这里是45度
        transform_1 (0,0) = cos (theta);  //这里是绕的Z轴旋转
        transform_1 (0,1) = -sin(theta);
        transform_1 (1,0) = sin (theta);
        transform_1 (1,1) = cos (theta);
        //   transform_1 (0,2) = 0.3;   //这样会产生缩放效果
        //   transform_1 (1,2) = 0.6;
        //    transform_1 (2,2) = 1;
        transform_1 (0,3) = 25; //这里沿X轴平移
        transform_1 (1,3) = 30;
        transform_1 (2,3) = 380;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud,*transform_cloud1,transform_1);  //不言而喻
        
        //局部
        pcl::transformPointCloud(*cloud,pcl::PointIndices indices,*transform_cloud1,matrix); //第一个参数为输入，第二个参数为输入点云中部分点集索引，第三个为存储对象，第四个是变换矩阵。
}

// 12..链接两个点云字段（两点云大小必须相同）
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_nomal (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_nomal);
}

