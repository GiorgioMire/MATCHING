/* 
 * File:   EliminaPiano.h
 * Author: giorgio
 *
 * Created on 6 luglio 2015, 9.52
 */

#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB point;
typedef pcl::PointCloud<point>::Ptr CloudT;
class EliminaPiano {
public:
    bool dofiltering;
    bool rimuovipiano;
    void filter();
    void compute();
    void visualizza();
    EliminaPiano();
   float distance_threshold;
   int max_iterations;
   CloudT   cloud;
   CloudT previous;
   float Rfilter;
   float percent;
   
private:
    
    

};

