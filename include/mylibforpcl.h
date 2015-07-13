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
//#include <pcl/common>
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizzatoreptr;
typedef pcl::visualization::PCLVisualizer visualizzatore;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorhandle;
typedef pcl::PointCloud<pcl::Normal>::Ptr norptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZptr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizzatoreptr;
typedef pcl::visualization::PCLVisualizer visualizzatore;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorhandle;
typedef pcl::PointCloud<pcl::Normal>::Ptr norptr;

void visualizza(pcptr cloudptr, visualizzatoreptr& v );

template<class PointT>
float diameter(typename pcl::PointCloud< PointT >::Ptr  cloudptr);


template<class T>
void filtra( typename pcl::PointCloud<T>::Ptr cloud, 
  typename pcl::PointCloud<T>::Ptr  cloud_filtered_ptr,float dim);

void visualizzaXYZ(pcXYZptr cloudptr );
void visualizzaRGBA(pcl::PointCloud<pcl::PointXYZRGBA> cloudptr );
void visualizza(pcptr cloudptr );

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,int ogni, float lunghezza);


template<class PointType>
typename pcl::PointCloud<PointType>::Ptr
FindAndSubtractPlane (typename pcl::PointCloud<PointType>::Ptr input, float distance_threshold, float max_iterations);


template<class T>
void visualizzanormali(typename pcl::PointCloud<T>::Ptr cloudptr,norptr normalptr,int ogni, float lunghezza);