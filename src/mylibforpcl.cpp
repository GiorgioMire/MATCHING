/* Definizioni di tipi*/
#include <mylibforpcl.h>
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
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <eigen3/Eigen/src/Geometry/Translation.h>
#include <eigen3/Eigen/src/Geometry/RotationBase.h>
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZptr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizzatoreptr;
typedef pcl::visualization::PCLVisualizer visualizzatore;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorhandle;
typedef pcl::PointCloud<pcl::Normal>::Ptr norptr;

template<class PointT>
float diameter(typename pcl::PointCloud< PointT >::Ptr  cloudptr){

PointT    min_pt;
PointT    max_pt;

pcl::getMinMax3D(  *cloudptr,
    
      min_pt,
      max_pt 
  ) ;
    

Eigen::Vector3f diff=max_pt.getVector3fMap(); -min_pt.getVector3fMap(); ;
return diff.norm();

}

template
float diameter<pcl::PointXYZ>(pcl::PointCloud< pcl::PointXYZ >::Ptr  cloudptr);



template<class T>
void filtra( typename pcl::PointCloud<T>::Ptr cloud, 
  typename pcl::PointCloud<T>::Ptr  cloud_filtered_ptr,float dim){
  pcl::VoxelGrid<T> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (dim, dim,dim);
  sor.filter (*cloud_filtered_ptr);
}

template void filtra<pcl::PointXYZ>(  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
   pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_filtered_ptr,float dim);




void visualizzaXYZ(pcXYZptr cloudptr )
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloudptr, 0, 0, 0);

  visualizzatoreptr viewer (new visualizzatore ("3D Viewer"));

  viewer->setBackgroundColor (255, 255, 255);

//viewer->addCoordinateSystem(0.5);
  viewer->addPointCloud<pcl::PointXYZ> (cloudptr,single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  while (!viewer->wasStopped ()) {
viewer->spinOnce ();
}
}


void visualizzaRGBA(pcl::PointCloud<pcl::PointXYZRGBA> cloudptr )
{
  visualizzatoreptr viewer (new visualizzatore ("3D Viewer"));

  viewer->setBackgroundColor (255, 255, 255);



pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::copyPointCloud(cloudptr,*cloudrgb);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudrgb);
viewer->addPointCloud(cloudrgb,rgb,"sample cloud");

viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"sample cloud");

while (!viewer->wasStopped ()) {
viewer->spinOnce ();
}
}



void visualizza(pcptr cloudptr )
{

  visualizzatoreptr viewer (new visualizzatore ("3D Viewer"));

  viewer->setBackgroundColor (255, 255, 255);
 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudptr);
 viewer->addCoordinateSystem(0.08,Eigen::Affine3f::Identity());
 viewer->addText3D<pcl::PointXYZ>("Or",pcl::PointXYZ(0,0,0),0.01,0,0,0);
//viewer->setCameraPosition (-0.166918, 0.0997064, -0.757201
//, 0.0177513, -0.999335, 0.0318579);
 Eigen::Affine3f Camera;
 Camera=Eigen::Translation3f(cloudptr->sensor_origin_.segment<3>(0))*(cloudptr->sensor_orientation_);
 viewer->addCoordinateSystem(0.08,Camera);

 viewer->addText3D<pcl::PointXYZ>("CAM",pcl::PointXYZ(cloudptr->sensor_origin_[0],cloudptr->sensor_origin_[1],
         cloudptr->sensor_origin_[2]),0.01,0,0,0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudptr, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  while (!viewer->wasStopped ()) {
viewer->spinOnce ();
}
}

void visualizza(pcptr cloudptr, visualizzatoreptr& v )
{

  visualizzatoreptr viewer (new visualizzatore ("3D Viewer"));

  viewer->setBackgroundColor (255, 255, 255);
 pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudptr);
 viewer->addCoordinateSystem(0.08,Eigen::Affine3f::Identity());
 viewer->addText3D<pcl::PointXYZ>("Or",pcl::PointXYZ(0,0,0),0.01,0,0,0);
//viewer->setCameraPosition (-0.166918, 0.0997064, -0.757201
//, 0.0177513, -0.999335, 0.0318579);
 Eigen::Affine3f Camera;
 Camera=Eigen::Translation3f(cloudptr->sensor_origin_.segment<3>(0))*(cloudptr->sensor_orientation_);
 viewer->addCoordinateSystem(0.08,Camera);

 viewer->addText3D<pcl::PointXYZ>("CAM",pcl::PointXYZ(cloudptr->sensor_origin_[0],cloudptr->sensor_origin_[1],
         cloudptr->sensor_origin_[2]),0.01,0,0,0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudptr, rgb, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
   v=viewer;}



template<class T>
boost::shared_ptr<pcl::visualization::PCLVisualizer> 
 normalsVis (
    typename pcl::PointCloud<T>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,int ogni, float lunghezza)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0,0);
  viewer->addPointCloud<T> (cloud,"sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem(0.2);

  viewer->addPointCloudNormals<T, pcl::Normal> (cloud, normals, ogni, lunghezza, "normals");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255,0,0, "normals");
  return (viewer);
}

template<class T>
void visualizzanormali(typename pcl::PointCloud<T>::Ptr cloudptr,norptr normalptr ,int ogni,float lunghezza)
{
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

viewer=normalsVis<T>(cloudptr,normalptr,ogni,lunghezza);
  while (!viewer->wasStopped ()) {
viewer->spinOnce ();
}
}



template void visualizzanormali<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr,norptr normalptr,int ogni,float lunghezza);
template void visualizzanormali<pcl::PointXYZRGB> (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr,norptr normalptr,int ogni,float lunghezza);









template<class PointType>
typename pcl::PointCloud<PointType>::Ptr
FindAndSubtractPlane (typename pcl::PointCloud<PointType>::Ptr input, float distance_threshold, float max_iterations)
{
  // Find the dominant plane
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (distance_threshold);
  seg.setMaxIterations (max_iterations);
  seg.setInputCloud (input);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment (*inliers, *coefficients);

  // Extract the inliers
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (true);
  typename pcl::PointCloud<PointType>::Ptr output (new  pcl::PointCloud<PointType> ());
  extract.filter (*output);

  return (output);
}

template pcl::PointCloud<pcl::PointXYZ>::Ptr 
FindAndSubtractPlane<pcl::PointXYZ>(pcl::PointCloud<pcl::PointXYZ>::Ptr input, float distance_threshold, float max_iterations);
