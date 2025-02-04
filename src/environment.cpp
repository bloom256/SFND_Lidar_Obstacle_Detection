/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "sensors/lidar.h"
#include "render/render.h"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

Color getRandomColor()
{
    return Color(std::abs(std::rand() % 100) / 100.0, std::abs(std::rand() % 100) / 100.0, std::abs(std::rand() % 100) / 100.0);
}

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI> & processor,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & inCloud)
{
    auto cloud = processor.FilterCloud(inCloud, 0.1, Eigen::Vector4f(-10, -5, -3, 1), Eigen::Vector4f(30, 5, 5, 1));

    auto roadAndOther = processor.SegmentPlane(cloud, 300, 0.3);
    renderPointCloud(viewer, roadAndOther.first, "road", Color(1, 0, 0));
    renderPointCloud(viewer, roadAndOther.second, "not road", Color(0, 1, 0));

    auto clusters = processor.Clustering(roadAndOther.second, 0.3, 20, 10000);
    for (size_t i = 0; i < clusters.size(); i++)
    {
        Color c = getRandomColor();
        Box box = processor.BoundingBox(clusters[i]);
        renderBox(viewer,box, i, c);
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars, 0);
    auto cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor;
    auto pair = processor.SegmentPlane(cloud, 300, 0.2);
    renderPointCloud(viewer, pair.first, "road", Color(1, 0, 0));
    renderPointCloud(viewer, pair.second, "not road", Color(0, 1, 0));

    auto clusters = processor.Clustering(pair.second, 1.0, 3, 30);
    for (size_t i = 0; i < clusters.size(); i++)
    {
        Color c = getRandomColor();
        Box box = processor.BoundingBox(clusters[i]);
        renderBox(viewer,box, i, c);
    }
    delete lidar;
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    ProcessPointClouds<pcl::PointXYZI>  pointProcessor;
    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessor.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloudI);
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce();
    }
}