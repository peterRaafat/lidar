// PCL lib Functions for processing point clouds 
#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*cloud);

    pcl::CropBox<PointT> roi;
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud);
    roi.filter(*cloud);

    pcl::CropBox<PointT> roiRoof;
    roiRoof.setMin(Eigen::Vector4f(-2.5, -1.8, -4.0, 1.0));
    roiRoof.setMax(Eigen::Vector4f(3, 1.8, 0, 1.0));
    roiRoof.setInputCloud(cloud);
    roiRoof.setNegative(true);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    roiRoof.getRemovedIndices(*inliers);
    roiRoof.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
   
    pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    
    std::unordered_set<int> inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);
   
    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            planeCloud->points.push_back(point);
        else
            obstacleCloud->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    
    /*
    pcl::PointIndices::Ptr inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    */
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(&obstacles, &plane);
    return segResult;
}

/*
template<typename PointT>
pcl::PointIndices::Ptr Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr inliersResult(new pcl::PointIndices());
    int i;
    int index;
    int size = cloud->size();
    int randInt;
    float a, b, c, d;
    float distance;
    int numInliers;
    int maxNumInliers = 0U;
    PointT pointOne;
    PointT pointTwo;
    PointT pointThree;


    pcl::PointIndices::Ptr indicies(new pcl::PointIndices());
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
        indicies->indices.clear();
        for (index = 0U; index < size; index++)
        {
            PointT point = cloud->points[index];
            distance = abs((a * point.x) + (b * point.y) + (c * point.z) + (d)) / sqrt((a * a) + (b * b) + (c * c));
            if (distance < distanceTol)
            {
                numInliers++;
                indicies->indices.push_back(index);

            }
        }
        if (numInliers > maxNumInliers)
        {
            maxNumInliers = numInliers;
            inliersResult->indices.clear();
            *inliersResult = *indicies;
        }
    }

    return inliersResult;

}
*/

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
    PointT pointOne;
    PointT pointTwo;
    PointT pointThree;


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
            PointT point = cloud->points[index];
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
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        j++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}