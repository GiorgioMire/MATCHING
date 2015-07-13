/* 
 * File:   VisualizzatorePatch.h
 * Author: giorgio
 *
 * Created on 28 giugno 2015, 16.00
 */

#pragma ones




#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include<boost/filesystem.hpp>
#include <Eigen/Dense>
#include <vtkRenderWindowInteractor.h>
typedef pcl::PointXYZRGB point;
class VisualizzatorePatch {
    
    
public:
    VisualizzatorePatch();
    VisualizzatorePatch(const VisualizzatorePatch& orig);
    virtual ~VisualizzatorePatch();
    
    void setFile(std::string file);
    pcl::visualization::PCLVisualizer::Ptr Viewer;
private:
    std::string PCDFile;
    std::string ReferenceFrameFile;
    
    pcl::PointCloud<point>::Ptr cloud;
    Eigen::Matrix4f frame;
};



