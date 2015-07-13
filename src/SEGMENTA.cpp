/* 
 * File:   SEGMENTA.cpp
 * Author: giorgio
 * 
 * Created on 3 luglio 2015, 15.23
 */

#include <SEGMENTA.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/segmentation/supervoxel_clustering.h>




void SEGMENTA::setNomeOggetto(std::string Nomefile) {

    /* Rimuove l'estensione*/
    this->nomeOggetto = Nomefile.substr(0, Nomefile.size() - 4);


};

void SEGMENTA::setCloud(CloudT cloud) {
    this->cloud = cloud;
};

void SEGMENTA::compute() {



    /*Oggetto segmentatore*/
    pcl::SupervoxelClustering<point> super(voxel_resolution,seed_resolution);

    /* Trasformazione che tiene di conto dell'abbassamento della densità di
     *  punti con la distanza*/

    super.setUseSingleCameraTransform(false);

    /* Cloud da segmentare*/
    super.setInputCloud(cloud);

    /*imposto i pesi*/
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);


    /*Mappa che conterrà i cluster*/
    super.extract(supervoxel_clusters);
    
    

    
    /* Ciclo sui supervoxel*/
    for (auto itr = supervoxel_clusters.begin(); itr != supervoxel_clusters.end(); itr++) 
    {
cout<<"\nSupervoxel contenente "<<itr->second->voxels_->points.size()<<"punti";



        /* Oggetto della classe che calcola il sistema di riferimento riproducibile*/


        /*TO DO: SGURF Potrebbe non calcolare il centroide che è già calcolato*/
        
        
        sg.addCloud(itr->second->voxels_);
        sg.compute();
        sgurfframes[itr->first]=(Eigen::Quaternionf(sg.getFrame().matrix().block<3, 3>(0, 0)));
        Eigen::Vector3f tmpvec;
        tmpvec << sg.getFrame().matrix().block<3, 1>(0, 3);
        centroidi[itr->first]=tmpvec;
        super.getSupervoxelAdjacency(supervoxel_adjacency);

    };
}

void SEGMENTA::SalvaSuDisco() {
    
    
    std::vector<std::string> cartelle;
    cartelle.push_back("./SEGMENTAZIONE_"+this->nomeOggetto+"/");
    cartelle.push_back("./SEGMENTAZIONE_"+this->nomeOggetto+"/ClusterClouds/");
    cartelle.push_back("./SEGMENTAZIONE_"+this->nomeOggetto+"/SGURFs/");
    cartelle.push_back("./SEGMENTAZIONE_"+this->nomeOggetto+"/Adiacenza/");
    
    for(auto itr=cartelle.begin();itr!=cartelle.end();itr++){
    /*Si crea la nuova cartella tramite la libreria boost*/
    boost::filesystem::path nuovaCartella = *itr;
    if(itr==cartelle.begin()) RootPath=*itr;
    boost::system::error_code returnedError;
    boost::filesystem::create_directories(nuovaCartella, returnedError);

    if (returnedError) {
        cout << endl << "Nuova cartella non creata" << endl;
    }//did not successfully create directories
    else {
        cout << endl << "Nuova cartella  creata" << endl;
        cout << *itr << endl;
    }};
    
        

    
    
    
    /*Salvo una pointcloud per ogni supervoxel*/
    for (auto itr=this->supervoxel_clusters.begin();itr!=this->supervoxel_clusters.end();itr++) {
        int id=itr->first;
        pcl::Supervoxel<point>::Ptr supervoxel=itr->second;
        
        
        
        cout << endl << " Salvo la cloud numero : " << id <<endl;
        pcl::io::savePCDFile(cartelle[1] + this->nomeOggetto
                + "_cluster_" + std::to_string(id) + ".pcd", *(supervoxel->voxels_));

        std::string id_riferimento = "ref_" + std::to_string(id);

        /* Salvo il sistema di riferimento*/
        ofstream file_SGURF(cartelle[2]+ this->nomeOggetto + "_SGURF" + std::to_string(id) + ".txt");

        file_SGURF << this->sgurfframes[id].x()<<" "
                << this->sgurfframes[id].y()<<" "
                << this->sgurfframes[id].z()<<" "
                << this->sgurfframes[id].w()<<" "
        << this->centroidi[id];
        
    }


   


        ofstream file_adiacenza(cartelle[3]+"adiacenza.txt");
        cout << endl << "salvo adiacenza nel file : \n" <<cartelle[3]+"adiacenza.txt"<< endl;
        
 std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
    for (; label_itr != supervoxel_adjacency.end();) {      
             cout << endl << "label itr: " <<label_itr->first<<endl;
            
            //First get the label
            uint32_t supervoxel_label = label_itr->first;
            std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;

            // std::cout << "voxel " << std::to_string(label_itr->first) << " has adjacencies:" << std::endl;

            for (; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr) {
                std::cout << "\t" << std::to_string(adjacent_itr->second) << std::endl;
                file_adiacenza << "\n" + std::to_string(label_itr->first) + "," + std::to_string(adjacent_itr->second) ;
               
            }
label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
        }
    };
    

void SEGMENTA::Visualize(){
    pcl::visualization::PCLVisualizer v;
    for(auto itr=this->supervoxel_clusters.begin();itr!=this->supervoxel_clusters.end();itr++){
       pcl::PointCloud<point>::Ptr cloud=itr->second->voxels_;
       int id=itr->first;
        
        
    pcl::visualization::PointCloudColorHandlerCustom<point> rnd_color (cloud,std::rand()%255, std::rand()%255, std::rand()%255);
    v.addPointCloud<point>(cloud,rnd_color,std::to_string(id)+"cl");
    Eigen::Affine3f T=Eigen::Affine3f::Identity();
    T=Eigen::Translation3f(this->centroidi[id])*(this->sgurfframes[id]);
    v.addCoordinateSystem(0.02,T,std::to_string(id)+"t");
    }
    while(!v.wasStopped()){v.spinOnce();};
    v.setSize(1,1);



};



    