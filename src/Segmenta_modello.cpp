#include <iostream>
#include <fstream>
#include <vector>
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
#include <SGURF.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <boost/filesystem.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <lettorePLY.h>
typedef pcl::PointXYZRGB point;
typedef pcl::PointCloud<point> pointcloud;
typedef pcl::PointXYZRGBL pointlabelled;
typedef pcl::PointCloud<pointlabelled> pointlabelledcloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizzatoreptr;
typedef pcl::visualization::PCLVisualizer visualizzatore;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorhandle;
typedef pcl::PointCloud<pcl::Normal>::Ptr norptr;
typedef pcl::PointXYZ pointxyz;
typedef pcl::PointNormal pointnormal;
typedef pcl::PointCloud<pointnormal> pointnormalcloud;
float step(float x);
float sign(float x);

// Funzione per visualizzare

/*_________________________________________________________*/
void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA &supervoxel_center,
        pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
        std::string supervoxel_name,
        boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

int
main(int argc, char** argv) {

    /*Conterrà il nome del soggetto con l'estensione ply*/
    std::string nomeSoggetto;
    /*Conterrà il nome del soggetto privato dell'estensione ply*/
    std::string nomeSoggetto_noExt;

    /*Controlla che in input ci sia un file PLY*/
    std::vector<int> ply_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    if (ply_file_indices.size() != 1) {
        pcl::console::print_error("\n Need one input PLY file.\n");
        return (-1);
    }



    /*Acquisisce la stringa del primo argomento passato*/
    nomeSoggetto = argv[1];

    /* Rimuove l'estensione*/
    nomeSoggetto_noExt = nomeSoggetto.substr(0, nomeSoggetto.size() - 4);

    /*Oggetto lettorePLY*/
    lettorePLY lply;

    /*Chiama il metodo di lettura (la cloud viene creata e un puntatore ad essa 
     creato nel'attributo "cloud" dell'oggetto*/
    lply.loadCloud(nomeSoggetto);

    /* Si crea il nome della nuova cartella concatenando il nome del soggetto e la parola SEGMENTAZIONE*/
    const std::string nuovaCartellaString = "./" + nomeSoggetto_noExt + "_SEGMENTAZIONE/";

    /*Si crea la nuova cartella tramite la libreria boost*/
    boost::filesystem::path nuovaCartella = nuovaCartellaString;
    boost::system::error_code returnedError;
    boost::filesystem::create_directories(nuovaCartella, returnedError);

    if (returnedError) {
        cout << endl << "Nuova cartella non creata" << endl;
    }//did not successfully create directories
    else {
        cout << endl << "Nuova cartella  creata" << endl;
        cout << nuovaCartellaString << endl;
    }


    pcl::console::TicToc tt;

    /*Acquisizione della cloud*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = lply.getCloud();



    cloud->sensor_origin_ << 0, 0, 0, 0.0;
    cloud->sensor_orientation_ = Eigen::Quaternionf(0, 0, 0, 1);
    visualizza(cloud);



    
    
    /* Risoluzione della griglia dei voxel*/
    float voxel_resolution = 0.003f;

    /* Risoluzione dei seed*/
    float seed_resolution = 0.03;

    /* Peso della distanza nello spazio colori nella segmentazione*/
    float color_importance = 0.0f;

    /* Peso della distanza spaziale nella segmentazione*/
    float spatial_importance = 1.0f;

    /* Peso della distanza tra le normali nella segmentazione*/
    float normal_importance = 1.0f;

    /*Oggetto segmentatore*/
    pcl::SupervoxelClustering<point> super(voxel_resolution, seed_resolution);
    
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
    std::map <uint32_t, pcl::Supervoxel<point>::Ptr > supervoxel_clusters;
    
    /*Estrazione*/
    pcl::console::print_highlight("Extracting supervoxels!\n");
    super.extract(supervoxel_clusters);
    
    
    pcl::console::print_highlight("\n Save supervoxel clouds \n");





    pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());
    
    /*Visualizzatore*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    /* Acquisisco la cloud dei centroidi*/
    pcl::PointCloud<point>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
    
    /* Converto la cloud dei centroidi in una cloud di punti senza colore*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centroid_cloudxyz(new
            pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*voxel_centroid_cloud, *voxel_centroid_cloudxyz);
    
    
    
       pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> bianco(voxel_centroid_cloudxyz, 255, 255, 255);
    pcl::PointCloud< pcl::PointXYZL>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
    viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
  
    /* Visualizzo i centroidi*/
    //viewer->addPointCloud(voxel_centroid_cloudxyz, bianco, "centroidi");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "labeled voxels");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");


/* Ciclo sui supervoxel*/
    for (auto itr = supervoxel_clusters.begin(); itr != supervoxel_clusters.end(); itr++) {

        pcl::console::print_highlight("\n Numero di punti del supervoxel: %i \n", itr->second->voxels_->size());
        
        /*Salvo una pointcloud per ogni supervoxel*/
        pcl::io::savePCDFile(nuovaCartellaString + nomeSoggetto_noExt  +"_cluster_" + std::to_string(itr->first) + ".pcd", *(itr->second->voxels_));
        
        pcl::console::print_highlight("\n Salvata in\n");
        std::cout << nuovaCartellaString+nomeSoggetto_noExt+ "_cluster_" + std::to_string(itr->first) + ".pcd";

        /* Oggetto della classe che calcola il sistema di riferimento riproducibile*/
        SGURF sg;
        sg.addCloud(itr->second->voxels_);
        sg.compute();

        std::string id_riferimento = "ref_" + std::to_string(itr->first);
        
        /* Visualizzo il sistema di riferimento */
        viewer->addCoordinateSystem(0.005, sg.getFrame(), id_riferimento);
        
        /* Salvo il sistema di riferimento*/
        ofstream file_SGURF(nuovaCartellaString + nomeSoggetto_noExt + "_SGURF" + std::to_string(itr->first)+".txt");
        file_SGURF << sg.getFrame().matrix();

        /*Scommentare queste righe se si vuole visualizzare anche tutte le patch*/
        //                pcl::visualization::PCLVisualizer::Ptr visualizza_patch(new pcl::visualization::PCLVisualizer);
        //                visualizza_patch->addPointCloud(itr->second->voxels_);
        //                visualizza_patch->addCoordinateSystem(0.02, sg.getFrame(), id_riferimento);
        //                while(!visualizza_patch->wasStopped()){
        //                visualizza_patch->spinOnce();}
        //    



    };




/*Cloud delle normali ai supervoxel*/
//    pointnormalcloud::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);


    /* Adiacenza dei supervoxel ottenuta in forma di multimap*/
    pcl::console::print_highlight("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    /*File dove salverò l'adiacenza*/
    ofstream file_adiacenza(nuovaCartellaString + nomeSoggetto_noExt+ "_adiacenza.txt");



    
    std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
    for (; label_itr != supervoxel_adjacency.end();) {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<point>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
        std::cout << "voxel " << std::to_string(label_itr->first) << " has adjacencies:" << std::endl;

        for (; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr) {
            std::cout << "\t" << std::to_string(adjacent_itr->second) << std::endl;
            file_adiacenza << "\n" + std::to_string(label_itr->first) + "," + std::to_string(adjacent_itr->second) + "\n";
            pcl::Supervoxel<point>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);





            adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;

        label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);

    }
    file_adiacenza.close();

    std::cerr << "Saving...\n", tt.tic();
    pcl::io::savePCDFile("output.pcd", *labeled_voxel_cloud);


    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }












    return (0);
}

void
addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA &supervoxel_center,
        pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
        std::string supervoxel_name,
        boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer) {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    pcl::PointCloud<pcl::PointXYZRGBA> ::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
    for (; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr) {
        points->InsertNextPoint(supervoxel_center.data);
        points->InsertNextPoint(adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    // Add the points to the dataset
    polyData->SetPoints(points);
    polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
    for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
        polyLine->GetPointIds()->SetId(i, i);
    cells->InsertNextCell(polyLine);
    // Add the lines to the dataset
    polyData->SetLines(cells);
    viewer->addModelFromPolyData(polyData, supervoxel_name);
}