// PCL lib Functions for processing point clouds 
#include <unordered_set>

#include "processPointClouds.h"
#include "kdtree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Also remove points located on the roof of the car
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.setInputCloud(cloudRegion);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());

  for (int index: inliers->indices)
    road->points.push_back(cloud->points[index]);

  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacles);  

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planr component from the remaining cloud 
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0){
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


struct Vector {
    double i, j, k;
};


inline Vector crossProduct(Vector v1, Vector v2) {
  Vector result;
  result.i = v1.j * v2.k - v1.k * v2.j;
  result.j = v1.k * v2.i - v1.i * v2.k;
  result.k = v1.i * v2.j - v1.j * v2.i; 

  return result;
}

template<typename PointT>
inline double distanceFromPlane(double A, double B, double C, double D, PointT p) {
  double numerator = std::abs(A * p.x + B * p.y + C * p.z + D);
  double denominator = std::sqrt(A * A + B * B + C * C);
  return numerator / denominator;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();


    std::unordered_set<int> inliers;
    srand(time(NULL));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distribution(0, cloud->size()-1); 
    
    for (int i = 0; i < maxIterations; ++i) {
      // avoid selecting duplicate indice,
      // we iterate until we have 3 different indices
      std::unordered_set<int> indices;
      while (indices.size() < 3) {
	indices.insert(distribution(gen) % cloud->points.size());
      }
      auto itr = indices.begin();
      const int random_idx_a = *itr++;
      const int random_idx_b = *itr++;
      const int random_idx_c = *itr;
      
      PointT sample_a = cloud->points[random_idx_a];
      PointT sample_b = cloud->points[random_idx_b];
      PointT sample_c = cloud->points[random_idx_c];
      
      Vector v1 = {sample_b.x - sample_a.x, sample_b.y - sample_a.y, sample_b.z - sample_a.z};
      Vector v2 = {sample_c.x - sample_a.x, sample_c.y - sample_a.y, sample_c.z - sample_a.z};
      
      Vector normal = crossProduct(v1, v2);
      double A = normal.i;
      double B = normal.j;
      double C = normal.k;
      double D = -(A * sample_a.x + B * sample_a.y + C * sample_a.z);

      std::unordered_set<int> tempInliers;
      for (int j = 0; j < cloud->size(); ++j){
	auto p = cloud->points[j];
	double distance = distanceFromPlane(A, B, C, D, p);
	if (distance <= distanceTol) {
	  tempInliers.insert(j);
	}
      
      }
      if (tempInliers.size() > inliers.size())
	inliers = tempInliers;

    }
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
    
    for(int index = 0; index < cloud->points.size(); index++)
      {
	PointT point = cloud->points[index];
	if(inliers.count(index))
	  cloudInliers->points.push_back(point);
	else
	  cloudOutliers->points.push_back(point);
      }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
void proximity(int point_idx, std::vector<bool>& processed, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol){
	processed[point_idx] = true;
	cluster.push_back(point_idx);
	auto nearbyPoints = tree->search(cloud->points[point_idx], distanceTol);
	for (auto neighbor : nearbyPoints) {
		if(!processed[neighbor]){
			proximity(neighbor, processed, cloud, cluster, tree, distanceTol);
		}
	}
};

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(), false);

	for (int idx = 0; idx < cloud->points.size(); ++idx){
		if(!processed[idx]) {
			std::vector<int> cluster;
			proximity(idx, processed, cloud, cluster, tree, distanceTol);
			clusters.push_back(cluster);

		} 
	}
 
	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering2(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersPoints;

    KdTree<PointT>* tree = new KdTree<PointT>();
  
    for (int i=0; i<cloud->size(); i++) 
      tree->insert((*cloud)[i],i); 

    std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, clusterTolerance);

    for(std::vector<int> cluster : clusters)
      {
	typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
	for(int indice: cluster)
	  clusterCloud->points.push_back((*cloud)[indice]);

	if (clusterCloud->points.size() >= minSize && clusterCloud->points.size() <= maxSize)
	  clustersPoints.push_back(clusterCloud);
      }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clustersPoints;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //clusterTolerance minSize maxSize
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // in meters
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for(const auto& cluster : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto& idx: cluster.indices){
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        clusters.push_back(cloud_cluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
   


    // Transform the original cloud to the origin where that principal component correspond to the axes
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud
    PointT minPoint, maxPoint; 
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    
    BoxQ box;
    box.bboxTransform = bboxTransform;
	box.bboxQuaternion = bboxQuaternion;
	box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
