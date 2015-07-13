/* 
 * File:   EliminaPiano.cpp
 * Author: giorgio
 * 
 * Created on 6 luglio 2015, 9.52
 */



#include <EliminaPiano.h>

EliminaPiano::EliminaPiano() {
}
void
EliminaPiano::filter(){

    cout<<"\nFILTRO LA CLOUD\n";
  pcl::VoxelGrid<point> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (this->Rfilter, this->Rfilter,this->Rfilter);
  sor.filter (*(this->cloud));
     cout<<"\nLA CLOUD ADESSO HA PUNTI\n"<<this->cloud->size();
}

void
EliminaPiano::compute ()

{
while(1){
  
    pcl::SACSegmentation<point> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (this->distance_threshold);
  seg.setMaxIterations (this->max_iterations);
  seg.setInputCloud (this->cloud);
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment (*inliers, *coefficients);
  if (coefficients->values.size()==0){
      std::cout<<"\nvettore dei coefficienti vuoto,esco dal ciclo\n";
  break;}
  
  else 
  
  if(inliers->indices.size()<this->percent/100.0*float(this->cloud->points.size())){
      std::cout<<"\n non abbastanza inliers,esco dal ciclo\n";
      cout<<"\npercentuale di inliers\n"<<inliers->indices.size()*100.0/cloud->points.size();
      break;}
  else
  {
     std::cout<<"\n abbastanza punti, filtro via il piano\n";
     cout<<"\npercentuale di inliers\n"<<inliers->indices.size()*100.0/cloud->points.size();
     cout<<"\n"<<this->distance_threshold<<"\t"<<this->Rfilter;
     // Extract the inliers
  pcl::ExtractIndices<point> extract;
  extract.setInputCloud (this->cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*(this->cloud));
  }
};};

void
EliminaPiano::visualizza (){
    cout<<"\nvisualizzo la cloud che adesso ha "<<this->cloud->size()<<"punti\n";
    pcl::visualization::PCLVisualizer v;   
    pcl::visualization::PointCloudColorHandlerCustom<point>(this->cloud,255,255,255);
     v.addPointCloud<point>(this->cloud);
     while(!v.wasStopped()){
     v.spinOnce();}
     v.setSize(1,1);
};

