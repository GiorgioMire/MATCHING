/* 
 * File:   Correspondance.h
 * Author: giorgio
 *
 * Created on 7 luglio 2015, 15.53
 */

#ifndef CORRESPONDANCE_H
#define	CORRESPONDANCE_H

#include <iostream>
#include <fstream>

#include <pcl-1.8/pcl/console/print.h>
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
#include <pcl/visualization/pcl_visualizer.h>

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
#include <pcl/correspondence.h>
#include "SEGMENTA.h"
#include "EliminaPiano.h"
#include <DescrizioneCloud.h>
typedef pcl::PointXYZRGB point;
typedef pcl::PointCloud<point>::Ptr CloudT;

struct triplet {
    int match;
    int query;
    float dist;
};

class Correspondance {
public:
    /* COSTRUTTORE */
    Correspondance();

    /* SOTTOMODULI */
    DescrizioneCloud DescrizioneModello;
    DescrizioneCloud DescrizioneScena;

    /* INGRESSI */
    float HistDistTreshSquare;
    int kmatch;

    CloudT Modello;
    CloudT Scena;

    std::map<int, std::vector<triplet>> mapQuery2Match;

    /* METODI */
    void visualizza();
    void compute();

private:

};

#endif	/* CORRESPONDANCE_H */

