/* 
 * File:   SGURF.h
 * Author: giorgio
 *
 * Created on 22 giugno 2015, 13.56
 */


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <mylibforpcl.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <pcl/features/principal_curvatures.h>



#ifndef SGURF_H
#define	SGURF_H

class SGURF {
private:

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  Eigen::Affine3f frame;
   float f1;
float f2; 

public:
  
   void compute();
    void  addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
Eigen::Affine3f getFrame(){return this->frame;};
float getF1(){return this->f1;};
float getF2(){return this->f2;};
};


#endif	/* SGURF_H */

