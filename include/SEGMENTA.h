/* 
 * File:   Segmenta_class.h
 * Author: giorgio
 *
 * Created on 3 luglio 2015, 15.23
 */

#pragma once
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

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <lettorePLY.h>
#include <SGURF.h>

typedef pcl::PointXYZRGB point;
typedef pcl::PointCloud<point>::Ptr CloudT;

class SEGMENTA {
    
    
    
public:
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    /*Methods*/
    
    
    void Visualize();
    void setNomeOggetto(std::string Nomefile);
    void setCloud(CloudT cloud);
    void compute();
    void SalvaSuDisco();
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    
    /* MODULI */
    SGURF sg;
    
    
    /* Attributes*/
    std::string nomeOggetto;
        /* Risoluzione della griglia dei voxel*/
    float voxel_resolution;
    
    /* Risoluzione dei seed*/
    float seed_resolution;

    /* Peso della distanza nello spazio colori nella segmentazione*/
    float color_importance;

    /* Peso della distanza spaziale nella segmentazione*/
    float spatial_importance;

    /* Peso della distanza tra le normali nella segmentazione*/
    float normal_importance;
    std::string RootPath;
    std::map <uint32_t, pcl::Supervoxel<point>::Ptr > supervoxel_clusters;
     std::map <uint32_t, Eigen::Vector3f>  centroidi;
   std::map <uint32_t, Eigen::Quaternionf >  sgurfframes;
   
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    
    
private:


    
    CloudT cloud;
    
    

};



