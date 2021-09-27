//
// Created by vcr on 2020-12-10.
//

#include "desirable_locations/find_locations_approx.h"


DetectTableApprox::DetectTableApprox(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,
                                     std::vector <desirable_locations::votenetDetection> detections) {
    ros::Time start = ros::Time::now();
    //assign the scene to this class
    this->scene_cloud = scene_cloud;
    this->marker_id = 0;
    //for each bounding box returns a list of intersections
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> intersections = findIntersectingBoundingBoxes(detections);
    pcl::PointCloud<pcl::PointXYZ>::Ptr published_table_tops(new pcl::PointCloud<pcl::PointXYZ>);

    //cout<<(ros::Time::now()-start).toSec()<<endl;

#ifdef VISUALISATIONS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_coloured(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*scene_cloud, *scene_coloured);
    for (int i = 0; i < scene_coloured->size(); i++) {
        scene_coloured->points[i].r = 250;
        scene_coloured->points[i].g = 255;
        scene_coloured->points[i].b = 255;
    }
#endif

    for (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = intersections.begin();
         it != intersections.end(); it++) {
        //extracted+combined the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = it->second;
        // Try to fit a planar model in the plane.
        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        // gets the equation of the plane for table top.
        fitPlanarModel(box_pcd, plane_coefficients);   // returns the table_top in terms of plane
        //find the table top based on euclidean segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd = findTableTop(plane_coefficients, scene_cloud,box_pcd);
        //publish this table top to RVIZ
        *published_table_tops += *table_top_pcd;
        //publishTableTop(table_top_pcd);

#ifdef VISUALISATIONS
        for(int i=0;i<scene_coloured->size();i++) {
            pcl::PointXYZ point = scene_cloud->at(i);
            float plane_value = plane_coefficients->values[0]*point.x + plane_coefficients->values[1]*point.y + plane_coefficients->values[2]*point.z + plane_coefficients->values[3];
            if(abs(plane_value) <= PLANE_DISTANCE_THRESHOLD)
            {
                scene_coloured->points[i].g = 125;
            }
        }
#endif
        //find the Minimum area polygon of this table top
        std::vector <Eigen::Vector3f> polygon_points = findMinimumAreaShape(table_top_pcd);
        std::vector <std::vector<Eigen::Vector3f>> locations = findPossiblePlacements(polygon_points,
                                                                                      PADDING_OFFSET / 2.0,
                                                                                      WHEELCHAIR_SAMPLING_DISTANCE);


#ifdef VISUALISATIONS
        for(int i=0;i<polygon_points.size();i++)
            {
               pcl::PointXYZ sphere_center(polygon_points[i][0],polygon_points[i][1],polygon_points[i][2]);
               viewer_approx->addSphere(sphere_center,0.1,"bbox"+std::to_string(this->marker_id++));
            }

        for(int i=0;i<locations.size();i++)
            {
            for(int j=0;j<locations[i].size();j++)
                {
                    pcl::PointXYZ sphere_center(locations[i][j][0],locations[i][j][1],locations[i][j][2]);
                    //viewer_approx->addSphere(sphere_center,0.04,"bbox"+std::to_string(marker_id++));
                }
            }
#endif
        std::vector <std::vector<Eigen::Vector3f>> locations_filtered = filterForFOV(locations);
        std::vector<double> wheelchairDimensions = {WHEELCHAIR_WIDTH, WHEELCHAIR_LENGTH, WHEELCHAIR_DEPTH};
        std::vector <std::vector<Eigen::Vector3f>> desirable_locations = filterForCollision(locations_filtered,
                                                                                            wheelchairDimensions);
//        std::vector <std::vector<Eigen::Vector3f>> desirable_locations = locations_filtered;
#ifdef VISUALISATIONS
        for(int i=0;i<desirable_locations.size();i++)
            {
              Eigen::Quaternionf wheelchair_rotation;
                double wheelchair_rotation_radian = 0;
            //the wheelchair rotation angle along each edge
            if(desirable_locations[i].size() >= 2){
                Eigen::Vector3f point1 = desirable_locations[i][0], point2 = desirable_locations[i][locations[i].size()-1]; // first and last point
                double slope = (point2[1]-point1[1])/(point2[0]-point1[0]);
                wheelchair_rotation_radian = atan2((point2[1]-point1[1]),(point2[0]-point1[0]));
            }
             wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian,
                                                Eigen::Vector3f::UnitZ());  // rotation along z-axis only
            for(int j=0;j<desirable_locations[i].size();j++)
                {
                    pcl::PointXYZ sphere_center(desirable_locations[i][j][0],desirable_locations[i][j][1],desirable_locations[i][j][2]);
                    //viewer_approx->addSphere(sphere_center,0.04,"bbox"+std::to_string(i+j*1000));
                    viewer_approx->addCube(desirable_locations[i][j],wheelchair_rotation,0.04,0.04,0.04,"cube"+std::to_string(this->marker_id++));
                }
            }
#endif
        // find weights for each desirable location
        std::vector <std::vector<Eigen::Vector3f>> clusters;
        std::vector <std::vector<double>> weights = calculateWeights(desirable_locations, clusters);

        //append these results into main results
        for(std::vector<Eigen::Vector3f> cluster:clusters)
            all_clusters.push_back(cluster);
        for(std::vector<double> weight:weights)
            all_weights.push_back(weight);

    }
    //simpleVis(scene_cloud, "scene");
    //publish this table top to RVIZ
    publishTableTop(published_table_tops);

#ifdef VISUALISATIONS
    viewer_approx->addPointCloud<pcl::PointXYZRGB> (scene_coloured, "sample cloud");
    viewer_approx->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer_approx->addCoordinateSystem (1.0);
    viewer_approx->initCameraParameters ();
    while (!viewer_approx->wasStopped()) {
        viewer_approx->spinOnce(100);
        sleep(0.1);
    }
    exit(0);
#endif

    //publish the desirable locations and weights for cost map
    publishLocations(all_clusters, all_weights);
    ros::Time end = ros::Time::now();
    cout << "Total Time : " << (end - start).toSec() << endl;

}

DetectTableApprox::~DetectTableApprox() {
}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>
DetectTableApprox::findIntersectingBoundingBoxes(std::vector <desirable_locations::votenetDetection> detections) {
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> intersection_list;  // stores the merged extracted pcd index-wise
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pointclouds;
    for (int i = 0; i < detections.size(); i++) {
        //check for threshold - porcess only objects which are above the threshold
        if (detections[i].object_score < OBJECTNESS_THRESHOLD)
            continue;
        //extract the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = extractBBoxPCD(detections[i].bbox_3d);
        extracted_pointclouds[i] = box_pcd;
    }

    intersection_list = computeOverlap(extracted_pointclouds);
    return intersection_list;
}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>
DetectTableApprox::computeOverlap(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pcd) {
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> merged_bbox;
    // find all the matchings as list indexes
    std::map<int, std::vector<int>> intersections;

    //insert empty lists
    std::vector<int> matches;
    for (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = extracted_pcd.begin();
         it != extracted_pcd.end(); it++) {
        intersections[it->first] = matches;
    }

    for (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = extracted_pcd.begin();
         it != extracted_pcd.end(); it++) {
        //create a convec hull of ith extracted pcd
        // general approach - create a convex hull and check if points lie inside the hull or not
        pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pts(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::ConvexHull <pcl::PointXYZ> chull;
        std::vector <pcl::Vertices> hullPolygons;
        chull.setInputCloud(it->second);
        chull.setComputeAreaVolume(true);   // compute the area and volume of the convex hull
        chull.setDimension(2);          // returns 2d convex hull - set it to 3 for XYZ plane
        chull.reconstruct(*convex_hull_pts, hullPolygons);

        pcl::CropHull <pcl::PointXYZ> cropHullFilter;
        //check within convex hull using filter
        cropHullFilter.setHullIndices(hullPolygons);
        cropHullFilter.setHullCloud(convex_hull_pts);
        cropHullFilter.setDim(2); // if you uncomment this, it will work

        //extracted_pcd.begin()
        for (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator ptr = it; ptr != extracted_pcd.end(); ptr++) {
            if (it->first != ptr->first) // dont compare same extracted pcd and skip if already matched
            {
                auto pos = std::find(intersections[it->first].begin(), intersections[it->first].end(), ptr->first);
                if (pos == intersections[it->first].end()) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud <pcl::PointXYZ>);
                    cropHullFilter.setInputCloud(ptr->second);
                    cropHullFilter.filter(*filtered);
                    if (filtered->size() >= 10) {
                        // they intersect we can add them
                        intersections[it->first].push_back(ptr->first);
                        intersections[ptr->first].push_back(it->first);
                    }
                }
            }
        }
    }

    //find the connected components
    std::map<int, std::vector<int>> merged_indexes = findConnectedComponents(intersections);

    //merge the point cloud data
    for (std::map < int, std::vector < int >> ::iterator it = merged_indexes.begin(); it != merged_indexes.end();
    it++)
    {
        merged_bbox[it->first] = extracted_pcd[it->first];
        for (int i = 0; i < it->second.size(); i++) {
            *merged_bbox[it->first] += *extracted_pcd[it->second.at(i)];
        }
    }
    return merged_bbox;

}

void
DetectTableApprox::DFS(std::map<int, std::vector<int>> intersections, std::set<int> &visited, std::vector<int> &path,
                       int node) {
    for (int i = 0; i < intersections[node].size(); i++) {
        if (visited.find(intersections[node][i]) == visited.end()) {
            visited.insert(intersections[node][i]);
            path.push_back(intersections[node][i]);
            DFS(intersections, visited, path, intersections[node][i]);
        }
    }
}

std::map<int, std::vector<int>>
DetectTableApprox::findConnectedComponents(std::map<int, std::vector<int>> intersections) {
    cout << "Merged BBOX: " <<endl;
    std::set<int> visited;
    std::map<int, std::vector<int>> components;
    for (std::map < int, std::vector < int >> ::iterator it = intersections.begin(); it != intersections.end();
    it++)
    {
        if (visited.find(it->first) == visited.end()) {
            std::vector<int> path;
            path.push_back(it->first);
            visited.insert(it->first);
            DFS(intersections, visited, path, it->first);
            components[it->first] = path;
            for (int k:path)
                cout << k << " ----->> ";
        }
        cout << endl;
    }
    return components;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTableApprox::extractBBoxPCD(std::vector <geometry_msgs::Point> bbox_corners) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_corners_pcd(new pcl::PointCloud <pcl::PointXYZ>);

    for (geometry_msgs::Point corner:bbox_corners) {
        pcl::PointXYZ point(corner.x, corner.y, corner.z);
        bbox_corners_pcd->push_back(point);
    }
    // general approach - create a convex hull and check if points lie inside the hull or not
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pts(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::ConvexHull <pcl::PointXYZ> chull;
    std::vector <pcl::Vertices> hullPolygons;
    pcl::CropHull <pcl::PointXYZ> cropHullFilter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud <pcl::PointXYZ>);

    //create the convex hull
    chull.setInputCloud(bbox_corners_pcd);
    chull.setComputeAreaVolume(true);   // compute the area and volume of the convex hull
    chull.setDimension(3);          // returns 2d convex hull - set it to 3 for XYZ plane
    chull.reconstruct(*convex_hull_pts, hullPolygons);

    //check within convex hull using filter
    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(convex_hull_pts);
    cropHullFilter.setDim(3); // if you uncomment this, it will work
    //cropHullFilter.setCropOutside(false); // this will remove points inside the hull
    cropHullFilter.setInputCloud(scene_cloud);
    cropHullFilter.filter(*filtered);
    return filtered;
}

void
DetectTableApprox::fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr table_bbox_cloud,
                                  pcl::ModelCoefficients::Ptr coefficients) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation <pcl::PointXYZ> seg;    // Create the segmentation object
    seg.setOptimizeCoefficients(true);     // Optional
    seg.setModelType(pcl::SACMODEL_PLANE);  // Fitting a plane on this point cloud
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(PLANE_DISTANCE_THRESHOLD);     // Minimum distance to be considered
    seg.setInputCloud(table_bbox_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }
    // equation of the plane
    std::cout << "Model coefficients (a,b,c,d): " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    std::cout << "Number of inliers " << inliers->indices.size() << " points." << std::endl;

    // CAN BE REMOVED FROM CODE NOW

//    //Create table top using inlier indices
//    pcl::PointCloud<pcl::PointXYZ>::Ptr table_plane(new pcl::PointCloud <pcl::PointXYZ>);
//    pcl::copyPointCloud(*table_bbox_cloud, *inliers, *table_plane);
//    return table_plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTableApprox::findTableTop(pcl::ModelCoefficients::Ptr plane_coefficients,
                                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cluster(new pcl::PointCloud <pcl::PointXYZ>); //returned object

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pcd(new pcl::PointCloud <pcl::PointXYZ>);
    for (int i = 0; i < scene_cloud->size(); i++) {
        pcl::PointXYZ point = scene_cloud->at(i);
        float plane_value = plane_coefficients->values[0] * point.x + plane_coefficients->values[1] * point.y +
                            plane_coefficients->values[2] * point.z + plane_coefficients->values[3];
        if (abs(plane_value) <= PLANE_DISTANCE_THRESHOLD) {
            plane_pcd->push_back(point);
        }
    }
    plane_pcd->width = plane_pcd->size();
    plane_pcd->height = 1;
    plane_pcd->is_dense = true;
    //copy the points to pcd
    //simpleVis(plane_pcd,"plane_pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (plane_pcd);
    sor.setMeanK (50);
    sor.setStddevMulThresh (2.0);
    sor.filter (*cloud_filtered);
    //simpleVis(cloud_filtered,"statistical");

    // Region Growing segmentation
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud_filtered);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50);
    reg.setInputCloud (cloud_filtered);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//    while (!viewer.wasStopped ())
//    {
//    }

    // for each cluster check which cluster has overlap with original box and merge it into returned result.
    // general approach - create a convex hull and check if points lie inside the hull or not
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pts(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::ConvexHull <pcl::PointXYZ> chull;
    std::vector <pcl::Vertices> hullPolygons;
    //create the convex hull
    chull.setInputCloud(box_pcd);
    chull.setComputeAreaVolume(true);   // compute the area and volume of the convex hull
    chull.setDimension(3);          // returns 2d convex hull - set it to 3 for XYZ plane
    chull.reconstruct(*convex_hull_pts, hullPolygons);

    pcl::CropHull <pcl::PointXYZ> cropHullFilter;
    //check within convex hull using filter
    cropHullFilter.setHullIndices(hullPolygons);
    cropHullFilter.setHullCloud(convex_hull_pts);
    cropHullFilter.setDim(3); // if you uncomment this, it will work
    //cropHullFilter.setCropOutside(false); // this will remove points inside the hull

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin();  it != clusters.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud_filtered)[*pit]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        //cloud_cluster->is_dense = true;
        //simpleVis(cloud_cluster,"cluster"+std::to_string(j));
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud <pcl::PointXYZ>);
        cropHullFilter.setInputCloud(cloud_cluster);   // taken from class which is the scene cloud
        cropHullFilter.filter(*filtered);
        if (filtered->size() > 100) {
            std::cout<<filtered->size()<<" Selected Cluster " << j <<std::endl;
            *merged_cluster += *cloud_cluster;
        }

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        j++;
    }

/*

    //https://pointclouds.org/documentation/tutorials/cluster_extraction.html
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    tree->setInputCloud(plane_pcd);

    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction <pcl::PointXYZ> ec;
    ec.setClusterTolerance(EUCLIDEAN_DISTANCE_THRESHOLD); // 10cm
    ec.setMinClusterSize(EUCLIDEAN_CLUSTER_MIN_SIZE);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(plane_pcd);
    ec.extract(cluster_indices);

    //find the
    //assert(cluster_indices.size() == 1 && "Euclidean Clustering more than one cluster detected");

    //merge all the remainig clusters

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*plane_pcd)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        *merged_cluster += *cloud_cluster;
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        j++;
    }*/
    //simpleVis(merged_cluster,"test");

    return merged_cluster;

}

void DetectTableApprox::publishTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd) {
    sensor_msgs::PointCloud2 pc2_msg_;
    pcl::toROSMsg(*table_top_pcd, pc2_msg_);
    pc2_msg_.header.frame_id = "odom";
    pc2_msg_.header.stamp = ros::Time::now();
    pub_table_approx.publish(pc2_msg_);
    ros::spinOnce();

}


std::vector <Eigen::Vector3f>
DetectTableApprox::findMinimumAreaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // project the 3D point onto a 2D plane but removing the height component - assumption evey point in plane has roughly sample height
    std::vector <cv::Point2f> points;
    double height = 0.0;
    for (unsigned int ii = 0; ii < cloud->points.size(); ii++) {
        cv::Point2f p2d(cloud->points[ii].x, cloud->points[ii].y);
        height += cloud->points[ii].z;
        points.push_back(p2d);
    }
    height /= cloud->points.size();

    // test for circularity of the table top
    // convex hull of the table top
    std::vector <cv::Point2f> hull_points;
    std::vector <cv::Point2f> contour;  // Convex hull contour points
    cv::convexHull(points, hull_points,
                   false);  //https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656
    // Approximating polygonal curve to convex hull
    cv::approxPolyDP(hull_points, contour, 0.1,
                     true);   //https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html

    std::vector <Eigen::Vector3f> locations;
    // convert this to eigen vector
    for (cv::Point2f point:contour) {
        Eigen::Vector3f p(point.x, point.y, height);
        locations.push_back(p);
        //std::cout<<point<<std::endl;
    }
    std::cout << "Convex Polygon Vertices : " << locations.size() << std::endl;
    return locations;

}

std::vector <std::vector<Eigen::Vector3f>>
DetectTableApprox::findPossiblePlacements(std::vector <Eigen::Vector3f> approx_polygon,
                                          double padding_offset, double location_sampling_distance) {
    int total_collsion_points = 0;
    std::vector <std::vector<Eigen::Vector3f>> collsion_points;
    // add the first point back again so that we have a complete loop
    approx_polygon.push_back(approx_polygon[0]);

    std::vector <Eigen::Vector3f> padded_approx_polygon = calculatePaddedLines(approx_polygon, padding_offset);

    for (int i = 0; i < padded_approx_polygon.size(); i = i + 2)  // last is repeated loop point so dont include it
    {

        std::vector <Eigen::Vector3f> points_on_line;
        float ratio = location_sampling_distance / sqrt((padded_approx_polygon[i] - padded_approx_polygon[i + 1]).dot(
                padded_approx_polygon[i] - padded_approx_polygon[i + 1]));
        float proportion = ratio;
        while (proportion < 1.0) {
            Eigen::Vector3f point =
                    padded_approx_polygon[i] + (padded_approx_polygon[i + 1] - padded_approx_polygon[i]) * proportion;
            points_on_line.push_back(point);
            proportion += ratio;
        }
        total_collsion_points += points_on_line.size();
        collsion_points.push_back(points_on_line);
    }

    std::cout << "Found " << total_collsion_points << " collision points along "
              << collsion_points.size() << " edges" << endl;
    return collsion_points;
}

std::vector <Eigen::Vector3f>
DetectTableApprox::calculatePaddedLines(std::vector <Eigen::Vector3f> approx_polygon, double padding_offset) {
    std::vector <Eigen::Vector3f> padded_edges;
    std::vector <cv::Point2f> contour;
    for (Eigen::Vector3f point:approx_polygon)
        contour.push_back(cv::Point2f(point[0], point[1]));
    contour.pop_back();

    for (int i = 0; i < approx_polygon.size() - 1; i++) {
        cv::Point2f p1 = cv::Point2f(approx_polygon[i][0],
                                     approx_polygon[i][1]); // "start"  // z point is not required for our case
        cv::Point2f p2 = cv::Point2f(approx_polygon[i + 1][0], approx_polygon[i + 1][1]); // "end"

        // take care with division by zero caused by vertical lines
        double slope = (p2.y - p1.y) / (double) (p2.x - p1.x);
        double perpendicular_slope = -1.0 / (slope);
        cv::Point2f padded_point1, padded_point2, padded_point3, padded_point4;
        cv::Point2f padded_point1_smaller, padded_point2_smaller, padded_point3_smaller, padded_point4_smaller;

        padded_point1.x = p1.x + sqrt(pow(padding_offset, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point1.y = perpendicular_slope * (padded_point1.x - p1.x) + p1.y;
        padded_point2.x = p1.x - sqrt(pow(padding_offset, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point2.y = perpendicular_slope * (padded_point2.x - p1.x) + p1.y;
        padded_point3.x = p2.x + sqrt(pow(padding_offset, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point3.y = perpendicular_slope * (padded_point3.x - p2.x) + p2.y;
        padded_point4.x = p2.x - sqrt(pow(padding_offset, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point4.y = perpendicular_slope * (padded_point4.x - p2.x) + p2.y;

        double padding_offset_smaller = 0.02; // 2cm
        padded_point1_smaller.x = p1.x + sqrt(pow(padding_offset_smaller, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point1_smaller.y = perpendicular_slope * (padded_point1_smaller.x - p1.x) + p1.y;
        padded_point2_smaller.x = p1.x - sqrt(pow(padding_offset_smaller, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point2_smaller.y = perpendicular_slope * (padded_point2_smaller.x - p1.x) + p1.y;
        padded_point3_smaller.x = p2.x + sqrt(pow(padding_offset_smaller, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point3_smaller.y = perpendicular_slope * (padded_point3_smaller.x - p2.x) + p2.y;
        padded_point4_smaller.x = p2.x - sqrt(pow(padding_offset_smaller, 2.0) / (1 + pow(pow(slope, 2.0), -1.0)));
        padded_point4_smaller.y = perpendicular_slope * (padded_point4_smaller.x - p2.x) + p2.y;

        cv::Point2f mid_point1 = (padded_point1_smaller + padded_point3_smaller) / 2.0;
        cv::Point2f mid_point2 = (padded_point2_smaller + padded_point4_smaller) / 2.0;

        if (cv::pointPolygonTest(contour, mid_point1, false) == -1) {
            padded_edges.push_back(Eigen::Vector3f(padded_point1.x, padded_point1.y, approx_polygon[i][2]));
            padded_edges.push_back(Eigen::Vector3f(padded_point3.x, padded_point3.y, approx_polygon[i][2]));
        } else {
            padded_edges.push_back(Eigen::Vector3f(padded_point2.x, padded_point2.y, approx_polygon[i][2]));
            padded_edges.push_back(Eigen::Vector3f(padded_point4.x, padded_point4.y, approx_polygon[i][2]));
        }

        /*  std::cout<<cv::pointPolygonTest(contour , mid_point1, false )<<std::endl;
          std::cout<<cv::pointPolygonTest(contour , mid_point2, false )<<std::endl;
          //std::cout<<cv::pointPolygonTest(contour , padded_point3, false )<<std::endl;
          //std::cout<<cv::pointPolygonTest(contour , padded_point4, false )<<std::endl;
          std::cout<<"--------------"<<std::endl;
          pcl::PointXYZ sphere_center1(padded_point1.x,padded_point1.y,approx_polygon[i][2]);
          viewer_approx->addSphere(sphere_center1,0.05,"bbox"+std::to_string(i+10));
          pcl::PointXYZ sphere_center2(padded_point2.x,padded_point2.y,approx_polygon[i][2]);
          viewer_approx->addSphere(sphere_center2,0.05,"bbox"+std::to_string(i+50));
          pcl::PointXYZ sphere_center3(padded_point3.x,padded_point3.y,approx_polygon[i][2]);
          viewer_approx->addSphere(sphere_center3,0.05,"bbox"+std::to_string(i+30));
          pcl::PointXYZ sphere_center4(padded_point4.x,padded_point4.y,approx_polygon[i][2]);
          viewer_approx->addSphere(sphere_center4,0.05,"bbox"+std::to_string(i+40));
          pcl::PointXYZ smid_point1(mid_point1.x,mid_point1.y,approx_polygon[i][2]);
          viewer_approx->addSphere(smid_point1,0.05,"bbox"+std::to_string(i+60));
          pcl::PointXYZ smid_point2(mid_point2.x,mid_point2.y,approx_polygon[i][2]);
          viewer_approx->addSphere(smid_point2,0.05,"bbox"+std::to_string(i+70));*/
    }
    cout << "Points in Padded Edges : " << padded_edges.size() << endl;
    return padded_edges;
}


std::vector <std::vector<Eigen::Vector3f>>
DetectTableApprox::filterForFOV(std::vector <std::vector<Eigen::Vector3f>> locations) {
    std::vector <std::vector<Eigen::Vector3f>> locations_FOV_filtered;
    //find the instantaeous transform between the "odom/world" and the camera_depth_optical_frame
    // trsnformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    try {
        transformStamped = tfBuffer.lookupTransform("camera_depth_optical_frame", "odom",
                                                    ros::Time(0),
                                                    ros::Duration(3.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform camera_depth_optical_frame to odom: %s", ex.what());
    }

    image_geometry::PinholeCameraModel cam_model; // init cam_model
    sensor_msgs::CameraInfo cam_info;
    cam_info.height = 480, cam_info.width = 640;
    cam_info.distortion_model = "plumb_bob";
    cam_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info.K = {554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.P = {554.254691191187, 0.0, 320.5, -0.0, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_model.fromCameraInfo(cam_info); // fill cam_model with CameraInfo

    int total = 0;
    for (int i = 0; i < locations.size(); i++) {
        std::vector <Eigen::Vector3f> filter_FOV;
        // do this for each edge
        for (int j = 0; j < locations[i].size(); j++) {
            geometry_msgs::PointStamped transformed_pt, initial_pt;
            initial_pt.point.x = locations[i][j][0], initial_pt.point.y = locations[i][j][1], initial_pt.point.z = locations[i][j][2];
            //tf2_geometry_msgs::do_transform(initial_pt, transformed_pt, transformStamped);
            tf2::doTransform(initial_pt, transformed_pt, transformStamped);
            // projection_matrix is the matrix you should use if you don't want to use project3dToPixel() and want to use opencv API
            cv::Matx34d projection_matrix = cam_model.fullProjectionMatrix();
            cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(transformed_pt.point.x, transformed_pt.point.y,
                                                                    transformed_pt.point.z)); // project 3d point to 2d point
            if (uv.x >= 0 && uv.x <= cam_info.width && uv.y >= 0 && uv.y <= cam_info.height)
                filter_FOV.push_back(locations[i][j]);
        }
        locations_FOV_filtered.push_back(filter_FOV);
        total += filter_FOV.size();
    }
    cout << "FOV filtered locations are : " << total << endl;
    return locations_FOV_filtered;
}

// This can be clearly optimized if needed
std::vector <std::vector<Eigen::Vector3f>>
DetectTableApprox::filterForCollision(std::vector <std::vector<Eigen::Vector3f>> &locations,
                                      std::vector<double> &wheelchair_dimensions) {

    std::vector <std::vector<Eigen::Vector3f>> desirable_locations;
    double wheelchair_width = wheelchair_dimensions[0], wheelchair_length = wheelchair_dimensions[1], wheelchair_depth = wheelchair_dimensions[2];


    int total_points = 0;
    for (int ii = 0; ii < locations.size(); ii++) {

        double wheelchair_rotation_radian = 0;
        Eigen::Quaternionf wheelchair_rotation;

        //the wheelchair rotation angle along each edge
        if (locations[ii].size() >= 2) {
            Eigen::Vector3f point1 = locations[ii][0], point2 = locations[ii][locations[ii].size() -
                                                                              1]; // first and last point
            double slope = (point2[1] - point1[1]) / (point2[0] - point1[0]);
            wheelchair_rotation_radian = atan2((point2[1] - point1[1]), (point2[0] - point1[0]));
        }
        wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian,
                                                Eigen::Vector3f::UnitZ());  // rotation along z-axis only

       // std::cout << wheelchair_rotation_radian * 180 / M_PI << std::endl;
        std::vector <Eigen::Vector3f> filtered_points;
        for (int j = 0; j < locations[ii].size(); j++) {




            //https://vtk.org/doc/nightly/html/classvtkDataSet.html
            //https://vtk.org/doc/nightly/html/classvtkPolyData.html
            //https://github.com/PointCloudLibrary/pcl/blob/master/visualization/src/common/shapes.cpp
            vtkSmartPointer <vtkDataSet> data = pcl::visualization::createCube(locations[ii][j],
                                                                               wheelchair_rotation,
                                                                               wheelchair_width, wheelchair_length,
                                                                               wheelchair_depth);

            std::set <std::vector<double>> cube_corners;
            for (int i = 0;
                 i < data->GetNumberOfPoints(); i++)            // returns all the edges 12*2 = 24 bidirectional

            {
                std::vector<double> edges{data->GetPoint(i)[0], data->GetPoint(i)[1], data->GetPoint(i)[2]};
                cube_corners.insert(edges);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cube_pcd(new pcl::PointCloud <pcl::PointXYZ>);
            cube_pcd->width = 4; //8 corners
            cube_pcd->height = 2;
            for (std::vector<double> corner:cube_corners) {
                pcl::PointXYZ point(corner[0], corner[1], corner[2]);
                cube_pcd->push_back(point);
            }

            // general approach - create a convex hull and check if points lie inside the hull or not
            pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pts(new pcl::PointCloud <pcl::PointXYZ>);
            pcl::ConvexHull <pcl::PointXYZ> chull;
            std::vector <pcl::Vertices> hullPolygons;
            //create the convex hull
            chull.setInputCloud(cube_pcd);
            chull.setComputeAreaVolume(true);   // compute the area and volume of the convex hull
            chull.setDimension(3);          // returns 2d convex hull - set it to 3 for XYZ plane
            chull.reconstruct(*convex_hull_pts, hullPolygons);

            pcl::CropHull <pcl::PointXYZ> cropHullFilter;
            //check within convex hull using filter
            cropHullFilter.setHullIndices(hullPolygons);
            cropHullFilter.setHullCloud(convex_hull_pts);
            cropHullFilter.setDim(3); // if you uncomment this, it will work
            //cropHullFilter.setCropOutside(false); // this will remove points inside the hull

            //optimised-  check for each point invidually
            pcl::PointCloud<pcl::PointXYZ>::Ptr check_cloud(new pcl::PointCloud <pcl::PointXYZ>);
            // Fill in the cloud data
            check_cloud->width = 1;
            check_cloud->height = 1;
            check_cloud->points.resize(check_cloud->width * check_cloud->height);
            bool point_inside = false;
            for (int nIndex = 0; nIndex < scene_cloud->points.size() && !point_inside; nIndex++) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud <pcl::PointXYZ>);
                check_cloud->points[0] = scene_cloud->points[nIndex];
                cropHullFilter.setInputCloud(check_cloud);   // taken from class which is the scene cloud
                cropHullFilter.filter(*filtered);
                if (filtered->size() > 0) {
                    point_inside = true;
                }
            }
            if (!point_inside) {
                filtered_points.push_back(locations[ii][j]);      // if no point inside the convex hull
            }

            // check 2 by shivam: https://math.stackexchange.com/questions/1472049/check-if-a-point-is-inside-a-rectangular-shaped-area-3d

/*
            std::set <std::vector<double>> cube_corners;
            for (int i = 0;
                 i < data->GetNumberOfPoints(); i++)            // returns all the edges 12*2 = 24 bidirectional

            {
                std::vector<double> edges{data->GetPoint(i)[0], data->GetPoint(i)[1], data->GetPoint(i)[2]};
                cube_corners.insert(edges);
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cube_pcd(new pcl::PointCloud <pcl::PointXYZ>);
            cube_pcd->width = 4; //8 corners
            cube_pcd->height = 2;
            for (std::vector<double> corner:cube_corners) {
                pcl::PointXYZ point(corner[0], corner[1], corner[2]);
                cube_pcd->push_back(point);
            }



            // general approach - create a convex hull and check if points lie inside the hull or not
            pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pts(new pcl::PointCloud <pcl::PointXYZ>);
            pcl::ConvexHull <pcl::PointXYZ> chull;
            std::vector <pcl::Vertices> hullPolygons;
            pcl::CropHull <pcl::PointXYZ> cropHullFilter;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud <pcl::PointXYZ>);

            //create the convex hull
            chull.setInputCloud(cube_pcd);
            chull.setComputeAreaVolume(true);   // compute the area and volume of the convex hull
            chull.setDimension(3);          // returns 2d convex hull - set it to 3 for XYZ plane
            chull.reconstruct(*convex_hull_pts, hullPolygons);

            //check within convex hull using filter
            cropHullFilter.setHullIndices(hullPolygons);
            cropHullFilter.setHullCloud(convex_hull_pts);
            cropHullFilter.setDim(3); // if you uncomment this, it will work
            //cropHullFilter.setCropOutside(false); // this will remove points inside the hull
            cropHullFilter.setInputCloud(scene_cloud);   // taken from class which is the scene cloud
            cropHullFilter.filter(*filtered);
            if (filtered->size() == 0) {
                filtered_points.push_back(locations[ii][j]);      // if no point inside the convex hull
            }
*/
        }
        total_points += filtered_points.size();
        desirable_locations.push_back(filtered_points);
    }
    std::cout << "Found " << total_points << " non colliding  points along " << desirable_locations.size()
              << " edges" << endl;
    return desirable_locations;
}

std::vector <std::vector<double>>
DetectTableApprox::calculateWeights(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,
                                    std::vector <std::vector<Eigen::Vector3f>> &clusters) {
    std::vector <std::vector<double>> weights; // final value to be returned
    std::vector<int> along_edge;               // tells about the edge of the point
    clusters = makeClusters(desirable_locations, along_edge);     // clusters formed by the points

    for (std::vector <Eigen::Vector3f> cluster:clusters) {

        Eigen::Vector3f mid_point(0, 0, 0);
        for (Eigen::Vector3f point:cluster)
            mid_point += point;
        mid_point /= cluster.size();

        // the sampling is such that the two extremes are our max values
        std::vector <Eigen::Vector3f> extremes;
        extremes.push_back(cluster[0]);
        extremes.push_back(cluster[cluster.size() - 1]);
        std::vector<double> weight_in_cluster;
        std::vector<int> partitions(2, 0);
        for (int i = 0; i < cluster.size(); i++) {
            double distance = sqrt((extremes[0] - cluster[i]).dot(extremes[0] - cluster[i])); // distnace from left
            int index = 0;
            if (sqrt((extremes[1] - cluster[i]).dot(extremes[1] - cluster[i])) < distance) {
                distance = sqrt((extremes[1] - cluster[i]).dot(
                        extremes[1] - cluster[i]));  // update to right if this is smaller
                index = 1;
            }
            if (distance >= DECREASE_RANGE) {
                weight_in_cluster.push_back(1);  // if the distance is more add 1
            } else {
                partitions[index]++;
            }
        }
        //assert(partitions[0] == partitions[1]);  // check for trapezium function
        double ratio_left = 1.0 / (partitions[0] + 1.0), ratio_right =
                1.0 / (partitions[1] + 1.0);   // add 1 extra for bounds since 1 is our max
        //double ratio = 1.0/(partitions[0]+1.0);
        std::vector<double> side_weights_left, side_weights_right;
        for (int i = 0; i < partitions[0]; i++) {
            side_weights_left.push_back((i + 1.0) * ratio_left);   // we should not take 0 as a weight
        }
        for (int i = 0; i < partitions[1]; i++) {
            side_weights_right.push_back((i + 1.0) * ratio_right);   // we should not take 0 as a weight
        }
        //right weight should be reversed
        reverse(side_weights_right.begin(), side_weights_right.end());
        weight_in_cluster.insert(weight_in_cluster.begin(), side_weights_left.begin(),
                                 side_weights_left.end()); // add the left side
        weight_in_cluster.insert(weight_in_cluster.end(), side_weights_right.begin(),
                                 side_weights_right.end()); // add the right side

        cout << "Cluster Size : " << weight_in_cluster.size() << " ( L,C,R ) = " << partitions[0] << " , "
             << (weight_in_cluster.size() - partitions[1] - partitions[0]) << " , " << partitions[1] << endl;
//        cout << "Left Partition has : " << partitions[0] << " Right Partition has : " << partitions[1]
//             << " Weight Size : " << weight_in_cluster.size() << endl;
        weights.push_back(weight_in_cluster);
        assert(cluster.size() == weight_in_cluster.size());
    }
    std::cout << "Weight calculated for each point" << std::endl;
    cout << "--------------------------------------------------------------------------" << endl;
    return weights;
}

std::vector <std::vector<Eigen::Vector3f>>
DetectTableApprox::makeClusters(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
                                std::vector<int> &along_edge) {
    cout << "--------------------------------------------------------------------------" << endl;
    std::vector <std::vector<Eigen::Vector3f>> clusters;
    for (int i = 0; i < desirable_locations.size(); i++) {

        //create the pcd for clustering for a edge
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
        cloud->width = desirable_locations[i].size();
        cloud->height = 1;
        for (Eigen::Vector3f corner:desirable_locations[i]) {
            pcl::PointXYZ point(corner.x(), corner.y(), corner.z());
            cloud->push_back(point);
        }

        //incase no possible placement along the edge don't cluster and return
        if (cloud->size() == 0)
            continue;

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        std::vector <pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction <pcl::PointXYZ> ec;
        ec.setClusterTolerance(CLUSTER_TOLERANCE); // wheelchair length/2 cm
        ec.setMinClusterSize(MIN_CLUSTER_SIZE); // at min 1 point in this cluster
        ec.setMaxClusterSize(cloud->size());   // at max all the points in the same cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        cout << "Edge : " << i << " Points : " << desirable_locations[i].size() << " Clusters : "
             << cluster_indices.size() << " - ";
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            std::vector <Eigen::Vector3f> cluster;
            Eigen::Vector3f pt;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                pt.x() = (*cloud)[*pit].x;
                pt.y() = (*cloud)[*pit].y;
                pt.z() = (*cloud)[*pit].z;
                cluster.push_back(pt);
            }
            along_edge.push_back(i);
            clusters.push_back(cluster);
            cout << " " << cluster.size() << " , ";
            //std::cout << "PointCloud representing the Cluster: " << cluster.size() << " data points." << std::endl;
        }
        std::cout << endl;
    }
    cout << "Total Number of Clusters formed are : " << clusters.size() << endl;
    return clusters;
}


/*
 * Publish locations for cost map..
 */
void DetectTableApprox::publishLocations(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
                                         std::vector <std::vector<double>> &weights) {
    assert(desirable_locations.size() == weights.size());
    desirable_locations::locationArray all_locations;
    for (int i = 0; i < desirable_locations.size(); i++) {
        for (int j = 0; j < desirable_locations[i].size(); j++) {
            assert(desirable_locations[i].size() == weights[i].size());
            desirable_locations::desiredLocation table_location;
            table_location.location.x = desirable_locations[i][j][0], table_location.location.y = desirable_locations[i][j][1], table_location.location.z = desirable_locations[i][j][2];
            table_location.location_weight = weights[i][j];
            all_locations.desired_locations.push_back(table_location);
        }
    }
    //publish the locations for cost map calculations...
    pub_approx.publish(all_locations);
}


void callbackDesirableLocations(const desirable_locations::detectionArray::ConstPtr &message) {

    sensor_msgs::PointCloud2 cloud = message->scene_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromROSMsg(cloud, *scene_cloud);  // assgin the scene cloud
    cout << "Scene Cloud : width " << scene_cloud->width << " Height : " << scene_cloud->height << endl;

    // Create the filtering object - voxelise the pcd
    std::vector<float> leaf_size = {0.02f, 0.02f, 0.02f};
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr scene_voxelised_pointcloud2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr scene_pointCloud2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*scene_cloud, *scene_pointCloud2);
    sor.setInputCloud(scene_pointCloud2);
    sor.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    sor.filter(*scene_voxelised_pointcloud2);
    printf("Scene Voxelised: width = %d, height = %d\n", scene_voxelised_pointcloud2->width,
           scene_voxelised_pointcloud2->height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_voxelised(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*scene_voxelised_pointcloud2, *scene_voxelised);

    // for each input pcd detect table tops
    DetectTableApprox detectTable(scene_voxelised, message->detections);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "find_location_approx");
    ros::NodeHandle n_pub, nh_;
    //Topic to publish
    pub_approx = n_pub.advertise<desirable_locations::locationArray>("/pcl_processing/desirable_locations", 1);
    pub_table_approx = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_processing/filtered_table_top", 1);
    //subscribe to the votenet detection topic
    ros::NodeHandle nh;
    // create the subscriber
    ros::Subscriber sub = nh.subscribe("/votenet/detections", 1, callbackDesirableLocations);
    // Run the subscriber until someone stops the program with Ctrl + C
    ros::spin();
    return 0;
}
