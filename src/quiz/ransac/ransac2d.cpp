/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor; 
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for (int iteration = 0; iteration < maxIterations; iteration++)
	{
		size_t i = ((size_t)(std::rand())) % cloud->points.size();
		size_t j;
		do
		{
			j = ((size_t)(std::rand())) % cloud->points.size();
		} while (j == i);
		size_t k;
		do
		{
			k = ((size_t)(std::rand())) % cloud->points.size();
		} while (k == i || k == j);

		const auto &p1 = cloud->points.at(i);
		const auto &p2 = cloud->points.at(j);
		const auto &p3 = cloud->points.at(k);

		Eigen::Vector3f v1(p2.getVector3fMap() - p1.getVector3fMap());
		Eigen::Vector3f v2(p3.getVector3fMap() - p1.getVector3fMap());
		auto cross = v1.cross(v2);

		const float A = cross[0];
		const float B = cross[1];
		const float C = cross[2];
		const float D = -(cross[0] * p1.x + cross[1] * p1.y + cross[2] * p1.z);

		const float divisor = std::sqrt(A * A + B * B + C * C);
		std::unordered_set<int> inliers;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			const auto &p = cloud->points[i];
			float dist = std::abs(A * p.x + B * p.y + C * p.z + D) / divisor;
			if (dist < distanceTol)
				inliers.insert(i);
		}
		if (inliers.size() > inliersResult.size())
			std::swap(inliers, inliersResult);
	}
	cout << "RansacPlane: " << inliersResult.size() << " inliers found" << endl;
	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	for (int iteration = 0; iteration < maxIterations; iteration++)
	{
		size_t i = ((size_t)(std::rand())) % cloud->points.size();
		size_t j;
		do
		{
			j = ((size_t)(std::rand())) % cloud->points.size();
		} while (j == i);

		const auto &p1 = cloud->points.at(i);
		const auto &p2 = cloud->points.at(j);

		const float A = p1.y - p2.y;
		const float B = p2.x - p1.x;
		const float C = p1.x * p2.y - p2.x * p1.y;

		const float divisor = std::sqrt(A * A + B * B);
		std::unordered_set<int> inliers;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			const auto &p = cloud->points[i];
			float dist = std::abs(A * p.x + B * p.y + C) / divisor;
			if (dist < distanceTol)
				inliers.insert(i);
		}
		if (inliers.size() > inliersResult.size())
			std::swap(inliers, inliersResult);
	}

	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
