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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    int i;
    int index;
    int size = cloud->size();
    int randInt;
    float a, b, c, d;
    float distance;
    int numInliers;
    int maxNumInliers = 0U;
    pcl::PointXYZ pointOne;
    pcl::PointXYZ pointTwo;
    pcl::PointXYZ pointThree;
    

    std::unordered_set<int> indicies;
	srand(time(NULL));
	
    for (i = 0U; i < maxIterations; i++)
    {
        randInt = rand() % size;
        pointOne = cloud->points[randInt];
        randInt = rand() % size;
        pointTwo = cloud->points[randInt];
        randInt = rand() % size;
        pointThree = cloud->points[randInt];
        float cross[3] = {
            ((pointTwo.y - pointOne.y) * (pointThree.z - pointOne.z)) - ((pointTwo.z - pointOne.z) * (pointThree.y - pointOne.y)),
            ((pointTwo.z - pointOne.z) * (pointThree.x - pointOne.x)) - ((pointTwo.x - pointOne.x) * (pointThree.z - pointOne.z)),
            ((pointTwo.x - pointOne.x) * (pointThree.y - pointOne.y)) - ((pointTwo.y - pointOne.y) * (pointThree.x - pointOne.x))
        };
        a = cross[0];
        b = cross[1];
        c = cross[2];
        d = -((a * pointOne.x) + (b * pointOne.y) + (c * pointOne.z));
        numInliers = 0U;
        indicies.clear();
        for (index = 0U; index < size; index++)
        {
            pcl::PointXYZ point = cloud->points[index];
            distance = abs((a * point.x) + (b * point.y) + (c * point.z) + (d)) / sqrt((a * a) + (b * b) + (c * c));
            if (distance < distanceTol)
            {
                numInliers++;
                indicies.insert(index);
            }
        }
        if (numInliers > maxNumInliers)
        {
            maxNumInliers = numInliers;
            inliersResult.clear();
            inliersResult = indicies;
        }
    }
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.6);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
        bool temp = inliers.count(index);
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
