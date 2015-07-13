/* 
 * File:   FPFH.cpp
 * Author: giorgio
 * 
 * Created on 4 luglio 2015, 16.07
 */

#include <FPFH.h>



#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/features/fpfh_omp.h>
#include <pcl-1.8/pcl/PointIndices.h>
#include <pcl-1.8/pcl/visualization/histogram_visualizer.h>
#include <pcl-1.8/pcl/io/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>

#include <fstream>
#include <pcl-1.7/pcl/console/print.h>



#define DEBUG 1

FPFH::FPFH() {



    this->fpfhs = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
    this->cloud = pcl::PointCloud<point>::Ptr(new pcl::PointCloud<point>);
    this->normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal> ());
    
}


void FPFH::compute() {

    pcl::FPFHEstimationOMP<point, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(this->cloud);
    fpfh.setInputNormals(this->normals);

    pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
    fpfh.setKSearch(this->cloud->points.size()-1);
    fpfh.setSearchMethod(tree);
    
    pcl::PointIndicesPtr idx(new pcl::PointIndices);
    int nearest;
    
    float dist=std::numeric_limits<float>::infinity();
    
    
    for(int i=0;i<this->cloud->points.size();i++){
        
        float currdist=0;
        for(int j=0;j<3;j++) 
            
          
        currdist+=std::pow(((this->cloud->points[i].data)[j]-this->Centroid[j]),2);
    if(dist>currdist){nearest=i;dist=currdist;};};

    idx->indices.push_back(nearest);
    fpfh.setIndices(idx);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());
    fpfh.compute(*(this->fpfhs));
}

void FPFH::computeNormals() {
    pcl::NormalEstimationOMP<point, pcl::Normal> ne;
    ne.setInputCloud(this->cloud);
    pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point> ());
    ne.setSearchMethod(tree);
    pcl::console::print_info("\nRNormal %f\n",RNormal);
    ne.setRadiusSearch(RNormal);
    ne.compute(*(this->normals));
};

void FPFH::loadNormals(pcl::PointCloud<pcl::Normal>::Ptr n) {
    this->normals = n;
};

void FPFH::loadPCD(std::string nomefile) {


    pcl::io::loadPCDFile<point>(nomefile, *(this->cloud));
    this->nomefile = nomefile;

    this->CalcolaCentroide();

};

void FPFH::CalcolaCentroide() {

    Eigen::Vector4f c;
    pcl::compute3DCentroid<point>(*(this->cloud), c);
    this->Centroid<<c[0],c[1],c[2];
}

void
FPFH::VisualizzaIstogramma() {
    //pcl::FPFHSignature33 hist=fpfhs->points[0];
    pcl::visualization::PCLHistogramVisualizer hview;
    hview.addFeatureHistogram<pcl::FPFHSignature33>(*(this->fpfhs), 33, "fpfh", 640, 480);



    hview.spinOnce();
  

}

void
FPFH::SalvaDatiPatchCSV() {


    bool primaCifra = true;
    int num_idx;
    std::stringstream number;
    for (unsigned int i = 0; i < this->nomefile.size(); i++) {
        if (isdigit(this->nomefile[i])) {
            if (primaCifra) {
                num_idx = i;
                primaCifra = false;
            };
            number << this->nomefile[i];
        };
    }

    std::string filePoseFrame = this->nomefile.substr(0, num_idx - 9) + "_SGURF" + number.str() + ".txt";
    cout << endl << "file:" << filePoseFrame << endl;

    ifstream PoseStream(filePoseFrame);
    Eigen::Matrix4f frame;

    for (int x = 0; x < 4; x++)
        for (int y = 0; y < 4; y++) {

            PoseStream>> frame(x, y);
        };
    
   PoseStream.close();



    //ofstream f(this->nomefile.substr(0, this->nomefile.size() - 4)+"_summary.csv");
    std::string separator=",";
    std::string newline="\n";
    
    std::stringstream intestazioneCSV;
    intestazioneCSV<<"ID"+separator;
    intestazioneCSV<<"Centroid_x"+separator+"Centroid_y"+separator+"Centroid_z"+separator;
    intestazioneCSV<<"Quaternion_x"+separator+"Quaternion_y"+separator+"Quaternion_z"+separator+"Quaternion_w";
    for(int i=0; i<33;i++)    intestazioneCSV<<separator+"FPFH_"+std::to_string(i);
   intestazioneCSV<<newline;
   
   
   
    Eigen::Vector3f centroid;
    centroid<<frame.block<3,1>(0,3);
    
     std::stringstream valoriCSV;
    valoriCSV<<number.str()+separator;
   valoriCSV<<std::to_string(centroid[0])
            +separator+
            std::to_string(centroid[1])+separator+
            std::to_string(centroid[2])+separator;
    
    Eigen::Quaternionf q(frame.block<3,3>(0,0));
    valoriCSV<<std::to_string(q.x())+separator+std::to_string(q.y())+separator+std::to_string(q.z())+separator+std::to_string(q.w());
    for(int i=0; i<33;i++)   valoriCSV<<separator+std::to_string(this->fpfhs->points[0].histogram[i]);
    //valoriCSV.close();

    this->intestazioneCSV=intestazioneCSV.str();
    this->valoriCSV=valoriCSV.str();



}

void FPFH::setCentroid(Eigen::Vector3f c){
    this->Centroid=c;
};
