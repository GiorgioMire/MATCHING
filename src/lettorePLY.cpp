
#include <lettorePLY.h>

void lettorePLY::Visualize(){
pcl::visualization::PCLVisualizer v;
v.addPointCloud<point>(this->cloud);
v.spinOnce(100);}

bool
lettorePLY::loadCloud (const std::string &filename)
{

pcl::console::print_highlight ("Loading "); pcl::console::print_value ("%s ", filename.c_str ());
pcl::PLYReader reader;
pcl::PointCloud<point>::Ptr cloud(new pcl::PointCloud<point>);
if (reader.read (filename, *cloud) < 0)
return (false);

pcl::console::print_info ("[done, "); 
pcl::console::print_value ("%d", cloud->width * cloud->height); 
pcl::console::print_info (" points]\n");


this->cloud=cloud;
return (true);
}

void
lettorePLY::saveCloud (const std::string &filename)
{

pcl::console::print_highlight ("Saving "); pcl::console::print_value ("%s ", filename.c_str ());
pcl::PCDWriter writer;
writer.write<point> (filename, *(this->cloud), true);
pcl::console::print_info ("[done, "); pcl::console::print_value ("%d", this->cloud->width * this->cloud->height); pcl::console::print_info (" points]\n");
}