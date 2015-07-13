/* 
 * File:   DescrizioneCloud.cpp
 * Author: giorgio
 * 
 * Created on 7 luglio 2015, 13.17
 */

#include <DescrizioneCloud.h>

typedef pcl::PointXYZRGB point;

void
DescrizioneCloud::loadCloud(std::string file) {
    nomefile = file;
    std::string argomento = nomefile;
    if (argomento.substr(argomento.size() - 4, argomento.size()) == ".ply") {
        lettorePLY lply;
        lply.loadCloud(nomefile);
        lply.Visualize();
        cloud = lply.getCloud();
    };


    if (argomento.substr(argomento.size() - 4, argomento.size()) == ".pcd") {


        pcl::io::loadPCDFile<point>(argomento, *(cloud));
        pcl::visualization::PCLVisualizer v;
        v.addPointCloud<point>(cloud, "cloud");
        while (!v.wasStopped()) {
            v.spinOnce();
        };


    };
};

void DescrizioneCloud::compute() {

    if (ep.dofiltering) {


        std::cout << "\nCERCO PIANI DA ELIMINARE\n";


        ep.cloud = cloud;
        ep.filter();

        if (ep.rimuovipiano) ep.compute();

        std::cout << "\nVISUALIZZO DOPO AVER FILTRATO VIA IL PIANO\n";
        if (ep.rimuovipiano) ep.visualizza();

    };





    sc.setCloud(cloud);
    sc.compute();
    sc.Visualize();


    cout << "\nVADO A CALCOLARE I DESCRITTORI FPFH\n";

    descriptorCloud->clear();
    clusters.clear();

    int indx = -1;
    for (auto itr = sc.supervoxel_clusters.begin(); itr != sc.supervoxel_clusters.end(); itr++) {

        int id = itr->first;


        if (itr->second->voxels_->points.size() > 4) {
            indx++;

            cout << "\ncluster " << id << " valido\n ";
            cout << "\n salvo id " << id << " \n ";
            valid[id] = true;



            cout << "\n carico la cloud\n";
            fpfh.loadCloud(itr->second->voxels_);

            cout << "\n calcolo le normali\n";
            fpfh.computeNormals();

            cout << "\n calcolo il descrittore\n";
            fpfh.compute();

            cout << "\n scrivo  descriptor cloud\n";
            descriptorCloud->push_back(fpfh.fpfhs->points[0]);

            cout << "\n scrivo la mappa\n ";
            signature[id] = fpfh.fpfhs->points[0];
            mapIndices2IDs[indx] = id;
            cout << indx << "\t " << id;

        }
        else {
            cout << "CLUSTER " << id << " NON VALIDO";
            valid[id] = false;

        };
    };
    
    for (auto itr = sc.supervoxel_clusters.begin(); itr != sc.supervoxel_clusters.end(); itr++)
        cout << "\n supervoxels id : " << itr->first << "\n";
    
    for (auto itr = mapIndices2IDs.begin(); itr != mapIndices2IDs.end(); itr++)
        cout << "\n mappa : " << itr->first << "\t" << itr->second << "\n";

}

DescrizioneCloud::DescrizioneCloud() {

    descriptorCloud = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
};

void DescrizioneCloud::GeneraCSV() {


    std::string separator = ",";
    std::string newline = "\n";

    std::stringstream intestazioneCSV;
    intestazioneCSV << "ID" + separator;
    intestazioneCSV << "Centroid_x" +
            separator + "Centroid_y" +
            separator + "Centroid_z" +
            separator;

    intestazioneCSV << "Quaternion_x" + separator + "Quaternion_y" + separator + "Quaternion_z" + separator + "Quaternion_w";
    for (int i = 0; i < 33; i++) intestazioneCSV << separator + "FPFH_" + std::to_string(i);

    intestazioneCSV << newline;



    std::stringstream valoriCSV;

    for (auto itr = signature.begin(); itr != signature.end(); itr++) {

        
        Eigen::Vector3f c = sc.centroidi[itr->first];
        Eigen::Quaternionf q =sc.sgurfframes[itr->first];
        float* hist = &(signature[itr->first].histogram[0]);

        valoriCSV <<
                std::to_string(itr->first) + separator;
        valoriCSV <<
                std::to_string(c[0]) + separator +
                std::to_string(c[1]) + separator +
                std::to_string(c[2]) + separator;

        valoriCSV <<
                std::to_string(q.x()) + separator +
                std::to_string(q.y()) + separator +
                std::to_string(q.z()) + separator +
                std::to_string(q.w());

        for (int i = 0; i < 33; i++) valoriCSV <<
                separator + std::to_string(hist[i]);
if ( itr != std::prev(signature.end())) valoriCSV << newline;

    }

    ofstream f(name + ".csv");

    f << intestazioneCSV.str()
            << valoriCSV.str();
    f.close();



}

