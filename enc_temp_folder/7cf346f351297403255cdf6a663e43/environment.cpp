/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
	boost::shared_ptr<Lidar> lidar(new Lidar(cars, 0));
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
	//renderRays(viewer, lidar->position, inputCloud);
	//renderPointCloud(viewer, inputCloud, "inputCloud");

    // Create point processor
	ProcessPointClouds<pcl::PointXYZ> process;
	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = process.SegmentPlane(inputCloud, 10, 0.2);
	//renderPointCloud(viewer, segmentCloud.first, "obstacles", Color(1, 0, 0));
	//renderPointCloud(viewer, segmentCloud.second, "road", Color(0, 1, 0));

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = process.Clustering(segmentCloud.first, 1.0, 3, 30);

	int clusterId = 0;
	std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

	for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
	{
		std::cout << "cluster size " << cluster->points.size() << std::endl;
		auto color = colors[clusterId % 3];
		renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), color);

		Box box = process.BoundingBox(cluster);
		renderBox(viewer, box, clusterId, color);

		++clusterId;
	}
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	// ----------------------------------------------------
	// -----Open 3D viewer and display City Block     -----
	// ----------------------------------------------------

	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("src/sensors/data/pcd/data_1/0000000000.pcd");
	float area_back = 10;
	float area_front = 20;
	float area_side = 7;
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.02 , Eigen::Vector4f(-area_back,-area_side,-5, 1), Eigen::Vector4f(area_front, area_side, 5, 1));
	renderPointCloud(viewer, filterCloud, "filterCloud");

	/* car roof test:
	float car_half_width = 1.3;
	Box carbox;
	carbox.x_min = -1.3;
	carbox.x_max = 2;
	carbox.y_min = -car_half_width-0.2;
	carbox.y_max = car_half_width;
	carbox.z_max = 0;
	carbox.z_min = -1;
	renderBox(viewer, carbox, 123, Color(1,1,0));
	*/
	//pointProcessorI->FilterCloud(cloud);
	//renderPointCloud(viewer, filterCloud, "filteredCloud");
	//renderPointCloud(viewer, inputCloud, "inputCloud");
}

namespace quiz_ransac2d {
	int main();
}

namespace quiz_cluster {
	int main();
}


int main (int argc, char** argv)
{
	//return quiz_ransac2d::main();
	//return quiz_cluster::main();
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
	cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}