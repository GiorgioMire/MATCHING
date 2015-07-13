/* 
 * File:   Correspondance.cpp
 * Author: giorgio
 * 
 * Created on 7 luglio 2015, 15.53
 */

#include <Correspondance.h>
#include <pcl-1.8/pcl/impl/point_types.hpp>
#include <pcl-1.8/pcl/console/print.h>
#include <pcl-1.7/pcl/console/print.h>
#include <pcl-1.8/pcl/impl/point_types.hpp>

float HistogramDistance(float* A, float* B);

Correspondance::Correspondance() {
    Modello = CloudT(new pcl::PointCloud<point>);
    Scena = CloudT(new pcl::PointCloud<point>);
};

bool comparetriplets(triplet a, triplet b) {
    return (a.dist < b.dist);
};

void Correspondance::compute() {

     

    std::cout << "\nCLCOLO DESCRIZIONE DEL MODELLO\n";
    DescrizioneModello.cloud=Modello;
    DescrizioneModello.compute();
    DescrizioneModello.GeneraCSV();
    
    DescrizioneScena.cloud=Scena;
    DescrizioneScena.compute();
    DescrizioneScena.GeneraCSV();



    std::cout << "\nCREO KDTREE PER IL MATCHING\n";




    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (int i = 0; i < DescrizioneModello.descriptorCloud->size(); i++) {



        std::vector<triplet> tripletvector;

        for (int j = 0; j < DescrizioneScena.descriptorCloud->size(); j++) {

            float * model_hist;
            model_hist = &(DescrizioneModello.descriptorCloud->points[i].histogram[0]);
            float *scene_hist;
            scene_hist = &(DescrizioneScena.descriptorCloud->points[j].histogram[0]);


            

            float distance = HistogramDistance(model_hist, scene_hist);

            triplet t;
            t.query = i;
            t.match = j;
            t.dist = distance;
            tripletvector.push_back(t);



        }

        std::sort(tripletvector.begin(), tripletvector.end(), comparetriplets);

        for (auto itr = tripletvector.begin(); itr != tripletvector.end(); itr++)
            pcl::console::print_value("\ntripletta dist:%f query:%i match:%i \n", itr->dist, itr->query, itr->match);

        std::vector<triplet> tmp;

        for (int i = 0; i < kmatch; i++)
            tmp.push_back(tripletvector[i]);

        mapQuery2Match[i] = tmp;
        tmp.clear();


    }

}

void Correspondance::visualizza() {
    
    cout << "\nVISUALIZZO CORRISPONDENZE\n";

    pcl::visualization::PointCloudColorHandlerCustom<point> color1(Modello, 50, 50, 50);
    pcl::visualization::PointCloudColorHandlerCustom<point> color2(Scena, 50, 50, 50);
    
    char uscire = 'n';
    while (uscire != 'y') {
        
        pcl::console::print_highlight("\n Scegli la patch da matchare da 0 a %i\n", mapQuery2Match.size() - 1);
        int which;
        cin>> which;
        
        auto itr = mapQuery2Match.begin() ;
        std::advance(itr,which);

        pcl::visualization::PCLVisualizer v;
        pcl::visualization::PCLVisualizer w;
        v.setPosition(0, 0);
        v.setWindowName("QUERY");
        w.setSize(640, 700);
        w.setWindowName("MATCH");
        v.setSize(640, 700);
        w.setPosition(645, 0);
        v.addPointCloud<point>(Modello, color1, "modello");
        w.addPointCloud<point>(Scena, color2, "scena");

        std::vector<triplet> tripletvect;

        tripletvect = itr->second;


        for (int l = 0; l < tripletvect.size(); l++) {

            int q = DescrizioneModello.mapIndices2IDs[tripletvect[l].query];
            int m = DescrizioneScena.mapIndices2IDs[tripletvect[l].match];
            float d = tripletvect[l].dist;
            pcl::console::print_info("\n correspondance %i -> % i ,dist=: %f  ", q, m, d);

            CloudT query = DescrizioneModello.sc.supervoxel_clusters[ q]->voxels_;
            CloudT match = DescrizioneScena.sc.supervoxel_clusters[m]->voxels_;

            int b = 255 * float(l) / float(tripletvect.size()); //std::rand()%255;
            int g = 0; //std::rand()%255;
            int r = 255 * (1 - l / float(tripletvect.size())); //std::rand()%255;

            pcl::visualization::PointCloudColorHandlerCustom<point> colorquery(query,
                    255, 0, 0);


            pcl::visualization::PointCloudColorHandlerCustom<point> colormatch(match,
                    r, g, b);



            v.addPointCloud(query, colorquery, "cloudmodello" + std::to_string(itr->first) + std::to_string(l));
            w.addPointCloud(match, colormatch, "cloudscena" + std::to_string(itr->first) + std::to_string(l));
            v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudmodello" + std::to_string(itr->first) + std::to_string(l));
            w.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudscena" + std::to_string(itr->first) + std::to_string(l));

        };

        while (!w.wasStopped() && !v.wasStopped()) {
            v.spinOnce();
            w.spinOnce();
        }




        for (int l = 0; l < tripletvect.size(); l++) {
            v.removePointCloud("cloudmodello" + std::to_string(itr->first) + std::to_string(l));
            w.removePointCloud("cloudscena" + std::to_string(itr->first) + std::to_string(l));
        };
        v.setSize(1, 1);
        w.setSize(1, 1);

        cout<<"\nUscire?\n y \\ n";
        cin>>uscire;

    }
}


float HistogramMax(float* A) {
    float max = A[0];

    for (int i = 1; i < 33; i++) {
        if (A[i] > max)
            max = A[i];
    }

    return max;


};

float HistogramDistance(float* A, float* B) {
    float d = 0;
    float maxA = HistogramMax(A);
    float maxB = HistogramMax(B);
    pcl::console::print_highlight("ISTOGRAMMI");
    for (int i = 0; i < 33; i++) {

        d += (A[i] / maxA - B[i] / maxB)*(A[i] / maxA - B[i] / maxB);
        pcl::console::print_value("\nA[%i]=%f B[%i]=%f normalizzati", i, A[i] / maxA, i, B[i] / maxB);


    };



    return d;

}