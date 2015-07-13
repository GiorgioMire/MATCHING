/* 
 * File:   DescrizioneCloud.h
 * Author: giorgio
 *
 * Created on 7 luglio 2015, 13.17
 */

#ifndef DESCRIZIONECLOUD_H
#define	DESCRIZIONECLOUD_H

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
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
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/segmentation/supervoxel_clustering.h> 

#include "SEGMENTA.h"
#include "EliminaPiano.h"
typedef pcl::PointXYZRGB point;
typedef pcl::PointCloud<point>::Ptr CloudT;

class DescrizioneCloud {
public:
    /* COSTRUTTORE*/
    DescrizioneCloud();

    /* METODI */
    void compute();
    void GeneraCSV();

    /* INGRESSO */
    CloudT cloud;
    std::string name;

    /* SOTTOMODULI */
    FPFH fpfh;
    EliminaPiano ep;
    SEGMENTA sc;

    /* USCITE */

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorCloud;
    void descriviCloud();
    void loadCloud(std::string file);
    std::vector<CloudT> clusters;
    std::string nomefile;


    std::vector<int> IDs;
    std::map<uint32_t, bool> valid;
    std::map<int, uint32_t> mapIndices2IDs;


   
    std::map<uint32_t, pcl::FPFHSignature33>signature;
private:

};

#endif	/* DESCRIZIONECLOUD_H */

