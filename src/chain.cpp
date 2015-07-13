/* 
 * File:   main.cpp
 * Author: giorgio
 *
 * Created on 3 luglio 2015, 17.38
 */
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <mylibforpcl.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <boost/filesystem.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <SEGMENTA.h>
#include <lettorePLY.h>
#include <SGURF.h>
#include <FPFH.h>
#include <fstream>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>

#include "SEGMENTA.h"
#include "EliminaPiano.h"


typedef pcl::PointXYZRGB point;

int main(int argc, char** argv) {
    
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%% PLY Reader %%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*          |                     */
    /*          |   cloud             */
    /*          |                     */
    /*          V                     */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%% Segmenta   %%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
     pcl::PointCloud<point>::Ptr cloud=pcl::PointCloud<pcl::PointXYZRGB>::Ptr (
            new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string argomento=argv[1];
    if(argomento.substr(argomento.size()-4,argomento.size())==".ply"){
    lettorePLY lply;
    lply.loadCloud(argv[1]);
    lply.saveCloud("FullCloud.pcd");
    lply.Visualize();
    cloud=lply.getCloud();};
    
    if(argomento.substr(argomento.size()-4,argomento.size())==".pcd"){
    
   
    pcl::io::loadPCDFile<point>(argomento,*cloud);
    pcl::visualization::PCLVisualizer v;
    v.addPointCloud<point>(cloud,"cloud");
    while(!v.wasStopped()){v.spinOnce();};
    
    
    };
    
    std::string dofiltering=argv[2];
    if(std::stoi(dofiltering)==1){
        EliminaPiano ep;
        ep.Rfilter=0.01;
        ep.distance_threshold=0.05;
        ep.max_iterations=1000;
        ep.cloud=cloud;
        ep.filter();
        ep.visualizza();
        ep.percent=50;
        ep.compute();
        ep.visualizza();
        
    }
    
    
    
    SEGMENTA sc;
    
    /*This is the first link(arrow)*/
    sc.setNomeOggetto(argv[1]);
    sc.setCloud(cloud);
    
    
    sc.color_importance=0;
    sc.normal_importance=0;
    sc.spatial_importance=1;
    
    /*TO DO: rendere meno arbitraria questa scelta*/
    sc.voxel_resolution=0.008;
    sc.seed_resolution=0.03;
    
    
    pcl::visualization::PCLVisualizer v;
    v.addPointCloud<point>(cloud,"cloud");
    v.addCube(-0.01,0.01,-0.01,0.01,-0.01,0.01,255,0,0,"voxel");
    v.addCube(-0.03+0.1,0.03+0.1,-0.03+0.1,0.03+0.1,-0.03+0.1,0.03+0.1,0,255,0,"seed");
    while(!v.wasStopped()){v.spinOnce();};
    
    sc.compute();
    sc.SalvaSuDisco();
    sc.Visualize();
    
        /*%%%%%%%%%%%%%%%%%%          */
    /*    %%% PLY Reader %%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*          |                     */
    /*          |   cloud             */
    /*          |                     */
    /*          V                     */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%% Segmenta   %%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
    /*    %%%%%%%%%%%%%%%%%%          */
   
     
            
            
            
 
    cout<<("\nScrivo CSV\n");
     
    ofstream csv(sc.RootPath+"FullModel.csv");
    csv<<   
            "ID"<<","<<
            "cx"<<","<<
            "cy"<<","<<
            "cz"<<","<<
            "qx"<<","<<
            "qy"<<","<<
            "qz"<<","<<
            "qw";
       for(int i=0;i<33;i++)csv<<","<<"FPFH_"<<std::to_string(i);
    csv<< "\n";
       for(int i=0;i<sc.getClusters().size();i++){
           if(sc.isValidCluster[i]){
           csv<<sc.getIDs()[i]<<",";
    FPFH fpfh;
    fpfh.loadCloud(sc.getClusters()[i]);
    /*TO DO : rendere il setting di questo numero autonomo*/   
    fpfh.setRNormal(0.01);
    fpfh.computeNormals();

    fpfh.compute();
    //fpfh.VisualizzaIstogramma();
   
    cout<<"\nscrivo centroide";
    for(int j=0;j<3;j++)
    csv<<(sc.getCentroids()[i])[j]<<",";
    cout<<"\nscrivo quaternione";
    for(int j=0;j<4;j++)
    csv<<sc.getSgurfFrames()[i].coeffs()[j]<<",";
    cout<<"\nscrivo descrittore";
    for(int j=0;j<33;j++){
        csv<<std::to_string(fpfh.fpfhs->points[0].histogram[j])<<((j==32)? " \n ":" , ");
        ;};};
    
    };
    
csv.close();
    return 0;
}

