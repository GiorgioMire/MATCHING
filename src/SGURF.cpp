/* 
 * File:   SGURF.cpp
 * Author: giorgio
 * 
 * Created on 22 giugno 2015, 13.56
 */


#include <pcl/features/principal_curvatures.h>
#include "SGURF.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <CGAL/Eigen_svd.h>
#include <pcl-1.8/pcl/common/centroid.h>


typedef pcl::PointXYZ pointxyz;
typedef pcl::PointNormal pointnormal;
typedef pcl::PointXYZRGB point;
typedef pcl::PointCloud<point> pointcloud;

typedef pcl::PointCloud<pointnormal> pointnormalcloud;
typedef pcl::PointXYZRGBL pointlabelled;
typedef pcl::PointCloud<pointlabelled> pointlabelledcloud;


float step(float x);
float sign(float x);
double calcolaCurvature(pcl::PointCloud<point>::Ptr cloud);
void SGURF::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    this->cloud = cloud;
}

void SGURF::compute() {

    /*Implementa la ricerca di un sistema di riferimento Semi-
    Global Unique Reference Frame andando a scrivere nella classe i campi
     SGURF::frame (rototraslazione che sovrappone il frame world al frame SGURF)
     SGURF::f1   (qualità della disambiguazione del verso degli assi)
     SGURF::f2  */

    /*Calcolo il centroide della cloud (vettore di 4 elementi per l'allineamento) */
    Eigen::Vector4f centroide;
    pcl::compute3DCentroid<pcl::PointXYZRGB>(*(this->cloud), centroide);

    /*Centroide con 3 elementi*/
    Eigen::Vector3f c;
    c << centroide.segment<3>(0);

    /*ca Calcolo la lunghezza del segmento più lungo che congiunge due punti della cloud*/
    point minp, maxp;
    float Diameter = pcl::getMaxSegment<point>(*(this->cloud), minp, maxp);

    
    /* Covarianza pesata*/
    Eigen::Matrix3f WCov = Eigen::Matrix3f::Zero();

    /*distanza tra un punto generico e il centroide*/
    float dist[(this->cloud)->size()];

    /*Addendi per ottere la covarianza pesata*/
    std::vector<Eigen::Matrix3f> tmp;
    tmp.resize((this->cloud)->size());

#pragma omp parallel for


    /*Questo ciclo calcola i singoli addendi per la covarianza pesata*/
    for (int i = 0; i < (this->cloud)->size(); i++) {

        /* Punto selezionato dalla corrente iterazione*/
        Eigen::Vector3f p;
        p << (this->cloud)->points[i].x, (this->cloud)->points[i].y, (this->cloud)->points[i].z;


        /*vettore con punta nel punto corrente e coda nel centroide*/
        Eigen::Vector3f d;
        d = p - c;

        /* Salvo la distanza dal centroide per calcolare il denominatore della
         *  media pesata*/

        dist[i] = d.norm();

        /* Salvo gli addendi che costituiranno la media pesata*/
        tmp[i] = (Diameter - dist[i]) * d * d.transpose();

    }


    /* Sommo gli addendi*/
    for (int i = 0; i < (this->cloud)->size(); i++) {
        WCov = WCov + tmp[i];
    }

    /* Costruisco la covarianza pesata*/
    float SumOfWeights;

    for (int i = 0; i < (this->cloud)->size(); i++) {
        SumOfWeights = SumOfWeights + Diameter - dist[i];
    }

    /*Costruisco la covarianza pesata */

    WCov = WCov / SumOfWeights;


    /* Decomposizione SVD che sfrutta la simmetria della covarianza*/
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(WCov);

    /*Matrice degli autovalori*/
    Eigen::Vector3cf D;
    D << es.eigenvalues();

    /* Matrice degli autovettori */
    Eigen::Matrix3cf V = es.eigenvectors();
 Eigen::Matrix3f VReal=V.real();
    double k=calcolaCurvature(cloud);
  
    /*Calcolo il verso dell'asse Z*/

    Eigen::Vector3f asseZ = -sign(k)
            *(VReal).block<3, 1>(0, 0);

    /* Candidati ad essere l'asse X*/
    Eigen::Vector3f v1 = (VReal).block<3, 1>(0, 1);
    Eigen::Vector3f v2 = (VReal).block<3, 1>(0, 2);


    /* "Voti" per il segno*/
    float S1plus_tmp[(this->cloud)->points.size()];
    float S1minus_tmp[(this->cloud)->points.size()];
    float S2plus_tmp[(this->cloud)->points.size()];
    float S2minus_tmp[(this->cloud)->points.size()];


#pragma omp parallel for

    for (int i = 0; i < (this->cloud)->size(); i++) {

        Eigen::Vector3f p;

        p << (this->cloud)->points[i].x, (this->cloud)->points[i].y, (this->cloud)->points[i].z;

        Eigen::Vector3f d;

        d = p - c;
        S1plus_tmp[i] = std::abs(d.dot(v1)) * step(d.dot(v1));
        S1minus_tmp[i] = std::abs(d.dot(v1)) * step(d.dot(-v1));
        S2plus_tmp[i] = std::abs(d.dot(v2)) * step(d.dot(v2));
        S2plus_tmp[i] = std::abs(d.dot(v2)) * step(d.dot(-v2));
    }

    float S1plus = 0, S1minus = 0, S2plus = 0, S2minus = 0;

    for (int i = 0; i < (this->cloud)->points.size(); i++) {

        S1plus += S1plus_tmp[i];
        S1minus += S1minus_tmp[i];
        S2plus += S2plus_tmp[i];
        S2minus += S2minus_tmp[i];


    }
    /* Decisione del segno*/
    if (std::abs(S1plus) < std::abs(S1minus)) v1 = -v1;

    if (std::abs(S2plus) < std::abs(S2minus)) v2 = -v2;

    /* Bontà della disambiguazione*/
   this->f1 = std::min(std::abs(S1plus), std::abs(S1minus)) / std::max(std::abs(S1plus), std::abs(S1minus));
    this-> f2 = std::min(std::abs(S2plus), std::abs(S2minus)) / std::max(std::abs(S2plus), std::abs(S2minus));




    Eigen::Vector3f asseX = (f1 < f2) ? v1 : v2;
    Eigen::Vector3f asseY = asseZ.cross(asseX);

  
   

//    /*Compongo la trasformazione*/
   this->frame = Eigen::Affine3f::Identity();
    this->frame.matrix().block<3, 1>(0, 0) << asseX;
    this->frame.matrix().block<3, 1>(0, 1) << asseY;
    this->frame.matrix().block<3, 1>(0, 2) << asseZ;
 this->frame.matrix().block<3, 1>(0, 3) << c;


}

double calcolaCurvature(pcl::PointCloud<point>::Ptr cloud){
    Eigen::Vector4f Centroide;
    
    pcl::compute3DCentroid(*cloud,Centroide);
pcl::KdTree<point>::Ptr tree_ (new pcl::KdTreeFLANN<point>);
tree_->setInputCloud(cloud);

std::vector<int> nn_indices (1);
std::vector<float> nn_dists (1);

tree_->nearestKSearch(pcl::PointXYZRGB(Centroide[0],Centroide[1],Centroide[2]), 1, nn_indices, nn_dists);

    
    typedef double DFT;
typedef CGAL::Simple_cartesian<DFT> Data_Kernel;
typedef Data_Kernel::Point_3 DPoint;
typedef CGAL::Monge_via_jet_fitting<Data_Kernel, CGAL::Simple_cartesian<double>  , CGAL::Eigen_svd > My_Monge_via_jet_fitting;
typedef My_Monge_via_jet_fitting::Monge_form My_Monge_form;
 
std::vector<DPoint> in_points;
size_t d_fitting = 4;
size_t d_monge = 4;

int i=nn_indices[0];

   float  x=cloud->points[i].x;
   float y=cloud->points[i].y;
   float z=cloud->points[i].z;
DPoint p(x,y,z);
    in_points.push_back(p);
 

for(int i=0;i<cloud->size();i++) {
    if(i==nn_indices[0]) continue;
   float  x=cloud->points[i].x;
   float y=cloud->points[i].y;
   float z=cloud->points[i].z;
DPoint p(x,y,z);
in_points.push_back(p);
};

// fct parameters
My_Monge_form monge_form;
My_Monge_via_jet_fitting monge_fit;
monge_form = monge_fit(in_points.begin(), in_points.end(), d_fitting, d_monge);
//OUTPUT on std::cout
//CGAL::set_pretty_mode(std::cout);
//std::cout << "vertex : " << in_points[0] << std::endl
//<< "number of points used : " << in_points.size() << std::endl
//<< monge_form;
return
(std::abs(monge_form.principal_curvatures(0))
        >
        std::abs(monge_form.principal_curvatures(1)))?
         monge_form.principal_curvatures(0):
          monge_form.principal_curvatures(1);
}



float step(float x) {
    if (x >= 0)
        return 1;

    else
        return 0;
}

float sign(float x) {
    if (x > 0)
        return 1;
    else if (x == 0)
        return 0;
    else
        return -1;
}

