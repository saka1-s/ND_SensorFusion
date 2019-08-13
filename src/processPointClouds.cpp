// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                                                              Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud_filtered);
    roi.filter(*cloud_region);

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-3., -2.2, -4., 1.));
    roof.setMax(Eigen::Vector4f( 3.,  2.2,  0., 1.));
    roof.setInputCloud(cloud_region);
    std::vector<int> indices;
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // std::cout << "roof:" << indices.size() << std::endl; 
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // CreateS the filtering object
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_n(new pcl::PointCloud<PointT>);
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    extract.setNegative (true);
    extract.filter (*cloud_n);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_p, cloud_n);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (false);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	for (int i = 0; i < maxIterations; ++i) {	
		// Randomly sample subset and fit line
		PointT point1 = cloud->points[rand()%cloud->points.size()];
		PointT point2 = cloud->points[rand()%cloud->points.size()];
		PointT point3 = cloud->points[rand()%cloud->points.size()];
		float I = (point2.y - point1.y)*(point3.z - point1.z) -
				  (point2.z - point1.z)*(point3.y - point1.y);
		float J = (point2.z - point1.z)*(point3.x - point1.x) -
				  (point2.x - point1.x)*(point3.z - point1.z);
		float K = (point2.x - point1.x)*(point3.y - point1.y) -
				  (point2.y - point1.y)*(point3.x - point1.x);
		float A = I;
		float B = J;
		float C = K;
		float D = -(I*point1.x + J*point1.y + K*point1.z);

	   // Measure distance between every point and fitted line
	    std::unordered_set<int> inliers;
		float norm = sqrt(pow(A, 2)+pow(B, 2)+pow(C, 2));
	    for (int p=0; p < cloud->points.size();++p) {
		   PointT sampled = cloud->points[p];
		   float distance = fabs(A*sampled.x + B*sampled.y + C*sampled.z + D);
		   distance /= norm;
			// If distance is smaller than threshold count it as inlier
		   if (distance < distanceTol) {
			   inliers.insert(p);
		   }
	    }
		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size()) {
			std::cout << "find better result:" << i << "," << inliers.size() << std::endl;
			inliersResult = inliers;
		}
	}
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane2(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back (cloud->points[*pit]);  //*
        }
        clusters.push_back(cloud_cluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(const std::vector<std::vector<float>>& points, int idx,
			   KdTree& tree,
               std::vector<int>& cluster,
			   std::bitset<MAX_PCD_SIZE>& processed,
			   float distanceTol) {
	processed[idx] = true;
	cluster.push_back(idx);
	std::cout << "*";
	std::vector<int> candidate = tree.search(points[idx], distanceTol);
	for (int c = 0; c < candidate.size();++c) {
		if (processed[candidate[c]])continue;
		Proximity(points, candidate[c], tree, cluster, processed, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(std::vector<std::vector<float>>& points,
												KdTree* tree, float distanceTol,
                                                int minSize, int maxSize)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::bitset<MAX_PCD_SIZE> processed;
	std::vector<std::vector<int>> clusters;
	for (int p = 0;p < points.size();++p) {
		if (processed[p])continue;
		std::cout << "New Cluster" << p << ",";
		std::vector<float> pt = points[p];
		std::vector<int> cluster;
		Proximity(points, p, *tree, cluster, processed, distanceTol);
		std::cout << std::endl;
        if(cluster.size() > minSize && cluster.size() < maxSize)
		    clusters.push_back(cluster);
	}
    std::cout << "num of cluster:" << clusters.size() << std::endl;
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering2(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                           float clusterTolerance, int minSize, int maxSize)
{
    std::vector<std::vector<float>> point_cloud;
	KdTree* tree = new KdTree;
    for (int i=0; i < cloud->points.size(); ++i) {
		PointT point = cloud->points[i];
        std::vector<float> v = {point.x, point.y, point.z};
        point_cloud.push_back(v);
    	tree->insert(v,i); 
    }
    std::vector<std::vector<int>> clusters_idx = euclideanCluster(point_cloud, tree, clusterTolerance, minSize, maxSize);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (int c = 0; c < clusters_idx.size(); ++c) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (int i= 0; i< clusters_idx[c].size(); ++i) {
            cloud_cluster->points.push_back(cloud->points[clusters_idx[c][i]]);
        }
        clusters.push_back(cloud_cluster);
    }
    return clusters;
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