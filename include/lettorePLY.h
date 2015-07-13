#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB point;

class lettorePLY{
	private:
	pcl::PointCloud<point>::Ptr cloud;
	
public:
    void Visualize();
bool loadCloud (const std::string &filename);
void saveCloud (const std::string &filename);
pcl::PointCloud<point>::Ptr getCloud(){return this->cloud;};
};
