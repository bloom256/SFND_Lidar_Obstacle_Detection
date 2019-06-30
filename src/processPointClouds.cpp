// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"


namespace
{
    template<typename PointT>
    std::unordered_set<int> ransacPlane(const typename pcl::PointCloud<PointT>::Ptr & cloud, int maxIterations, float distanceTol)
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
                if (dist <= distanceTol)
                    inliers.insert(i);
            }
            if (inliers.size() > inliersResult.size())
                std::swap(inliers, inliersResult);
        }
        return inliersResult;
    }

    template<typename PointT>
    void buildCluster(
        const typename pcl::PointCloud<PointT>::Ptr & cloud, 
        size_t currentPointIndex, 
        std::unordered_set<size_t> & visitedPoints,
        KdTree * tree, 
        int maxSize,
        typename pcl::PointCloud<PointT>::Ptr & cluster)
    {
        if (visitedPoints.count(currentPointIndex))
            return;
        if (cluster->points.size() == maxSize)
            return;

        const auto & currPoint = cloud->points.at(currentPointIndex);
        cluster->points.push_back(currPoint);
        visitedPoints.insert(currentPointIndex);

        std::vector<float> searchPoint = { currPoint.x, currPoint.y, currPoint.z };
        const float serachRadius = 3.0f;
        std::vector<int> nbrIndices = tree->search(searchPoint, serachRadius);
        for (int nbrIndex : nbrIndices)
        {
            if (visitedPoints.count(nbrIndex))
                continue;
            buildCluster<PointT>(cloud, nbrIndex, visitedPoints, tree, maxSize, cluster);
        }
    }

    template<typename PointT>
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(
        const typename pcl::PointCloud<PointT>::Ptr & cloud, 
        float distanceTol,
        int minSize,
        int maxSize)
    {
        assert(cloud->points.size() < INT_MAX); // kd-tree store indices as ints

        KdTree * tree = new KdTree;
        for (int i = 0; i < cloud->points.size(); i++)
        {
            const auto p = cloud->points[i];
            std::vector<float> treePoint = { p.x, p.y, p.z };
            tree->insert(treePoint, i); 
        }

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        std::unordered_set<size_t> visitedPoints;
        
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            if (visitedPoints.count(i))
                continue;
        
            typename pcl::PointCloud<PointT>::Ptr cluster(new typename pcl::PointCloud<PointT>);
            buildCluster<PointT>(cloud, i, visitedPoints, tree, maxSize, cluster);
            if (cluster->points.size() >= minSize)
                clusters.push_back(cluster);
        } 
        return clusters;
    }
}

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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes,
    Eigen::Vector4f minPoint, 
    Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::CropBox<PointT> cropFilter; 
    cropFilter.setInputCloud (cloud); 
    cropFilter.setMin(minPoint); 
    cropFilter.setMax(maxPoint); 
    typename pcl::PointCloud<PointT>::Ptr cropped(new typename pcl::PointCloud<PointT>);
    cropFilter.filter (*cropped); 

    cropFilter.setInputCloud (cropped); 
    cropFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1)); 
    cropFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1)); 
    pcl::PointIndices::Ptr roof(new pcl::PointIndices);
    cropFilter.filter (roof->indices); 

    typename pcl::ExtractIndices<PointT> extract;    
    extract.setInputCloud(cropped);
    extract.setIndices(roof);
    extract.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr croppedNoRoof(new typename pcl::PointCloud<PointT>);
    extract.filter(*croppedNoRoof);

    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (croppedNoRoof);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    typename pcl::PointCloud<PointT>::Ptr croppedNoRoofDownsampled(new typename pcl::PointCloud<PointT>);
    vg.filter (*croppedNoRoofDownsampled);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedNoRoofDownsampled;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new typename pcl::PointCloud<PointT>), cloud_f(new typename pcl::PointCloud<PointT>);

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    extract.setNegative(true);
    extract.filter(*cloud_f);
    std::cerr << "PointCloud representing the nonplanar component: " << cloud_f->width * cloud_f->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_p, cloud_f);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int maxIterations,
    float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); 
    std::unordered_set<int> inlierIndices = ransacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    for (const auto index : inlierIndices)
        inliers->indices.push_back(index);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster<PointT>(cloud, clusterTolerance, minSize, maxSize);
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