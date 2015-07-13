/* 
 * File:   main.cpp
 * Author: giorgio
 *
 * Created on 8 luglio 2015, 12.19
 */

#include <Correspondance.h>
#include<iostream>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/common/copy_point.h>

#include <eigen3/Eigen/Geometry>
using namespace std;
typedef pcl::PointXYZRGB point;









int kmatchglobal = 10;
float HistDistTreshSquare = 5000;
float PlanePercent = 30;
float distance_treshold = 0.02;
int PlaneMaxIter = 2000;
float Rfilter = 0.003;
float RNormal = 0.006;
float color_importance = 0;
float normal_importance = 1;
float spatial_importance = 1;
float seed_resolution = 0.05;
float voxel_resolution = 0.005;
bool modelremoveplane = false;
bool sceneremoveplane = true;
std::string modstr;
std::string scestr;

bool transformscene = false;
float tx = 0, ty = 0, tz = 0, qx = 0, qy = 0, qz = 0, qw = 1;







void
showHelp(char *filename);
void
parseCommandLine(int argc, char *argv[]);

/*
 * 
 */
int main(int argc, char** argv) {

    parseCommandLine(argc, argv);

    /* CARICO GLI ARGOMENTI*/




    /* CREO L'OGGETTO CHE CALCOLA LE CORRISPONDENZE*/
    Correspondance corr;

    /* LEGGO LE POINT CLOUD*/
    if (pcl::io::loadPCDFile<point> (modstr, *(corr.Modello)) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }




    if (pcl::io::loadPCDFile<point> (scestr, *(corr.Scena)) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }



    if (transformscene) {

        Eigen::Affine3f T;
        Eigen::Quaternionf q = Eigen::Quaternionf(qw, qx, qy, qz);
        q.normalize();
        T = Eigen::Translation<float, 3>(tx, ty, tz) * q;

        CloudT old(new pcl::PointCloud<point>);
        pcl::copyPointCloud(*corr.Scena, *old);
        pcl::transformPointCloud(*corr.Scena, *corr.Scena, T);
        pcl::visualization::PointCloudColorHandlerCustom<point> white(old, 255, 255, 255);
        pcl::visualization::PointCloudColorHandlerCustom<point> green(corr.Scena, 0, 255, 0);
        pcl::visualization::PCLVisualizer tv;
        tv.addPointCloud<point>(old, white, "old");
        tv.addPointCloud<point>(corr.Scena, green, "new");

        while (!tv.wasStopped()) {
            tv.spinOnce(100);
        }

        tv.setSize(1, 1);

    }
    /* VISUALIZZO LE POINTCLOUD*/



    pcl::visualization::PCLVisualizer::Ptr v(new pcl::visualization::PCLVisualizer);
    pcl::visualization::PCLVisualizer::Ptr w(new pcl::visualization::PCLVisualizer);

    pcl::visualization::PointCloudColorHandlerCustom<point> whiteM(corr.Modello, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<point> whiteS(corr.Scena, 255, 255, 255);


    v->addPointCloud<point>(corr.Modello, whiteM, "modello");
    v->setPosition(10, 10);
    v->setSize(640, 480);
    w->addPointCloud<point>(corr.Scena, whiteS, "scena");
    w->setPosition(650, 10);
    w->setSize(640, 480);
    v->setWindowName("MODELLO");
    w->setWindowName("SCENA");

    while (!(v->wasStopped())&&!(w->wasStopped())) {
        v->spinOnce(10);
        w->spinOnce(10);

    };
    w->setSize(1, 1);
    v->setSize(1, 1);




    corr.kmatch = kmatchglobal;
    corr.HistDistTreshSquare = HistDistTreshSquare;
    corr.DescrizioneModello.name="Modello";
    corr.DescrizioneScena.name="Scene";
    
    /* FILTRAGGIO E ELIMINAZIONE DEL PIANO */
    corr.DescrizioneModello.ep.rimuovipiano = modelremoveplane;
    corr.DescrizioneModello.ep.percent = PlanePercent;
    corr.DescrizioneModello.ep.distance_threshold = distance_treshold;
    corr.DescrizioneModello.ep.max_iterations = PlaneMaxIter;
    corr.DescrizioneModello.ep.Rfilter = Rfilter;

    corr.DescrizioneScena.ep.rimuovipiano =sceneremoveplane;
    corr.DescrizioneScena.ep.percent = PlanePercent;
    corr.DescrizioneScena.ep.distance_threshold = distance_treshold;
    corr.DescrizioneScena.ep.max_iterations = PlaneMaxIter;
    corr.DescrizioneScena.ep.Rfilter = Rfilter;



    /* DESCRIZIONE*/
    corr.DescrizioneModello.fpfh.RNormal = RNormal;
    corr.DescrizioneModello.sc.color_importance = color_importance;
    corr.DescrizioneModello.sc.normal_importance = normal_importance;
    corr.DescrizioneModello.sc.spatial_importance = spatial_importance;
    corr.DescrizioneModello.sc.seed_resolution = seed_resolution;
    corr.DescrizioneModello.sc.voxel_resolution = voxel_resolution;

    corr.DescrizioneScena.fpfh.RNormal = RNormal;
    corr.DescrizioneScena.sc.color_importance = color_importance;
    corr.DescrizioneScena.sc.normal_importance = normal_importance;
    corr.DescrizioneScena.sc.spatial_importance = spatial_importance;
    corr.DescrizioneScena.sc.seed_resolution = seed_resolution;
    corr.DescrizioneScena.sc.voxel_resolution = voxel_resolution;



    corr.compute();
    corr.visualizza();
    corr.DescrizioneModello.fpfh.SalvaDatiPatchCSV();
    corr.DescrizioneScena.fpfh.SalvaDatiPatchCSV();


    return 0;
}

void
parseCommandLine(int argc, char *argv[]) {
    if (pcl::console::find_switch(argc, argv, "-h")) {
        showHelp(argv[0]);
        exit(0);
    }
    //Show help
    if (pcl::console::find_switch(argc, argv, "-h")) {

        exit(0);
    }

    //Model & scene filenames
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() != 2) {
        std::cout << "Filenames missing.\n";

        exit(-1);
    }

    modstr = argv[filenames[0]];
    scestr = argv[filenames[1]];

    //Program behavior
    if (pcl::console::find_switch(argc, argv, "-MRP")) {
        modelremoveplane = true;
    } else {
        modelremoveplane = false;
    }

    if (pcl::console::find_switch(argc, argv, "-SRP")) {
        sceneremoveplane = true;
    } else {
        sceneremoveplane = false;
    }

    if (pcl::console::find_switch(argc, argv, "-t")) {
        transformscene = true;
    } else {
        transformscene = false;
    }



    //General parameters
    pcl::console::parse_argument(argc, argv, "--HD", HistDistTreshSquare);
    pcl::console::parse_argument(argc, argv, "--PP", PlanePercent);
    pcl::console::parse_argument(argc, argv, "--PD", distance_treshold);
    pcl::console::parse_argument(argc, argv, "--PMI", PlaneMaxIter);
    pcl::console::parse_argument(argc, argv, "--RF", Rfilter);
    pcl::console::parse_argument(argc, argv, "--RN", RNormal);
    pcl::console::parse_argument(argc, argv, "--CI", color_importance);
    pcl::console::parse_argument(argc, argv, "--NI", normal_importance);
    pcl::console::parse_argument(argc, argv, "--SI", spatial_importance);
    pcl::console::parse_argument(argc, argv, "--SR", seed_resolution);
    pcl::console::parse_argument(argc, argv, "--VR", voxel_resolution);
    pcl::console::parse_argument(argc, argv, "--k", kmatchglobal);
    pcl::console::parse_argument(argc, argv, "-tx", tx);
    pcl::console::parse_argument(argc, argv, "-ty", ty);
    pcl::console::parse_argument(argc, argv, "-tz", tz);
    pcl::console::parse_argument(argc, argv, "-qx", qx);
    pcl::console::parse_argument(argc, argv, "-qy", qy);
    pcl::console::parse_argument(argc, argv, "-qz", qz);
    pcl::console::parse_argument(argc, argv, "-qw", qw);
}

void
showHelp(char *filename) {
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*             Correspondance main usage                                   *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl << std::endl;
    std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
    std::cout << "          -h                     Show this help" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     --PP:                    Plane Percent" << std::endl;
    std::cout << "     --PD:                    Distance from plane" << std::endl;
    std::cout << "     --PMI:                   Max iter for RANSAC plane search" << std::endl;
    std::cout << "     --RF:                    Filter radious" << std::endl;
    std::cout << "     --RN:                    Normal  radious" << std::endl;
    std::cout << "      --CI                    Color Importance" << std::endl;
    std::cout << "     --NI                     Normal Importance" << std::endl;
    std::cout << "     --SI                     Spatial importance" << std::endl;
    std::cout << "     --SR                     Seed Resolution" << std::endl;
    std::cout << "     --VR                     Voxel resolution" << std::endl;
    std::cout << "     -MRP                     model remove plane" << std::endl;
    std::cout << "     -SRP:                    Scene remove plane" << std::endl;
    std::cout << "     --K:                    Number of maches" << std::endl;
}


