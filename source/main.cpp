#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <string>
#include <pcl/filters/passthrough.h>
#include <time.h>
using namespace std;
pcl::PointXYZ computerCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr points){
    pcl::PointXYZ center;
    return center;
}
void seg_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source)
{
	pcl::visualization::PCLVisualizer viewer ("Cluster Viewer"); 
	clock_t start, finish;
	start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	std::cout << "PointCloud before filter has: " << source->points.size() << " data points." << std::endl;
	// Create the filtering object: downsample the dataset using a leaf size of 0.5cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(source);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
   //compute normal																											 
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_filtered);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	// Compute the features
	ne.compute(*normals);
	finish = clock();
	cout << "Time:" << finish - start << endl;

	start = clock();
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.006);
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(normals);
	seg.segment(*inliers, *coefficients);
	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_plane);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_plane,255,255,255);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> single_color(cloud_plane);
	viewer.addPointCloud(cloud_plane,single_color,"plane");
	//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	finish = clock();
	cout << "RANSAC:" << finish - start << endl;
	start = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_plane);
	chull.reconstruct(*cloud_hull);
	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*segmented);
	cloud_filtered.swap(segmented);
	// segment those points that are in the polygonal prism
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> ex;
	ex.setInputCloud(cloud_filtered);
	ex.setInputPlanarHull(cloud_hull);
	pcl::PointIndices::Ptr output(new pcl::PointIndices);
	ex.segment(*output);
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(output);
	extract.setNegative(false);
	extract.filter(*segmented);
	cloud_filtered.swap(segmented);
	//Elucion cluster
	tree->setInputCloud(cloud_filtered);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(300);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);
	finish = clock();
	cout << "cost time"<<finish - start << endl;
	//use largest cluster
	//if(cluster_indices.size()!=0)
	for (int i = 0; i < cluster_indices.size();i++)
	{
		cout << cluster_indices.size() << " clusters found";
		//if (cluster_indices.size() > 1)
			//cout << " Using largest one...";
		//cout << endl;
		pcl::IndicesPtr indices(new std::vector<int>);
		*indices = cluster_indices[i].indices;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(indices);
		extract.setNegative(false);
		extract.filter(*segmented);
		char cloudname[20];
		sprintf(cloudname,"%d",i+1);
		//sprintf(cloudname, "%d", 0);
	        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> clusterColorHandle(segmented,255,0,0);
	        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> clusterColorHandle(segmented);
		//pcl::visualization::PointCloudColorHandler 
		viewer.addPointCloud(segmented, clusterColorHandle,cloudname);
                pcl::PointXYZ center=computeCenter(segmented);
                viewer.addText3D(cloudname,&center);
                
	}
	//viewer.addPointCloud(segmented);
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
	std::cout << "PointCloud representing the Cluster: " << segmented->points.size() << " data points." << std::endl;
	std::stringstream ss;
	ss << "cloud_cluster_";
	
}
int main(int argc, char ** argv) 
{
	if (argc < 2) {
		PCL_ERROR("Please input the PCD file Name.");
		exit(1);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) {
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredX(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.5, 0.5);
	pass.filter(*cloud_filteredX);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXY(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setInputCloud(cloud_filteredX);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.5, 0.5);
	pass.filter(*cloud_filteredXY);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setInputCloud(cloud_filteredXY);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.5);
	pass.filter(*cloud_filteredXYZ);
	//pass.setFilterLimitsNegative (true);
	seg_cloud(cloud_filteredXYZ);
	return 0;
}
