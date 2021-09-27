//
// Created by vcr on 2021-01-08.
//

#include "desirable_locations/find_locations_approx.h"

void Open3DViz::publishData()
{
    sensor_msgs::PointCloud2 scene_pc;
    pcl::toROSMsg(*this->scene_cloud, scene_pc);

    sensor_msgs::PointCloud2 object_top_pc;
    pcl::toROSMsg(*this->object_top_cloud, object_top_pc);

    std::vector<geometry_msgs::Point> poly_pts;
    for(auto pt:this->polygon_points)
    {
        geometry_msgs::Point p;
        p.x = pt[0], p.y = pt[1], p.z = pt[2];
        poly_pts.push_back(p);
    }
    std::vector<geometry_msgs::Point> pot_pls;
    for(auto edge:this->potential_placements)
    {
        for(auto v:edge)
        {
            geometry_msgs::Point p;
            p.x = v[0], p.y = v[1], p.z = v[2];
            pot_pls.push_back(p);
        }
    }
    std::vector<geometry_msgs::Quaternion> heading;
    std::vector<double> weight;
    std::vector<geometry_msgs::Point> loc;
    for(int i=0;i<this->all_clusters.size();i++)
    {
        for(int j=0;j<this->all_clusters[i].size();j++)
        {
            geometry_msgs::Point p;
            p.x = all_clusters[i][j][0];
            p.y = all_clusters[i][j][1];
            p.z = all_clusters[i][j][2];
            loc.push_back(p);
            weight.push_back(all_weights[i][j]);
            geometry_msgs::Quaternion q;
            tf::quaternionEigenToMsg(all_heading[i][j].cast<double>(), q);
            heading.push_back(q);
        }
    }

    sensor_msgs::Image img_msg; // >> message to be sent

    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header
    img_bridge = cv_bridge::CvImage(header, "64FC1", this->depth_image);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

    desirable_locations::o3dViz viz_msg;
    viz_msg.scene_pc = scene_pc, viz_msg.obj_top_pc = object_top_pc;
    viz_msg.poly_pts = poly_pts;
    viz_msg.pot_pts = pot_pls;
    viz_msg.loc = loc;
    viz_msg.heading = heading, viz_msg.location_weight = weight;
    pub_o3d.publish(viz_msg);



}

DetectTableApprox::DetectTableApprox(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, cv::Mat &d_image,
                                     std::vector <desirable_locations::votenetDetection> detections) {
    ros::Time start = ros::Time::now();
    //assign the scene and depth image to this class
    this->scene_cloud = scene_cloud;
    this->depth_image = d_image;
    this->o3dviz = new Open3DViz();

    //this->sensor_depth_image = depth_image;
    // generate point cloud from depth image and back
    //this->camera_cloud = convertDepthToPCD(this->sensor_depth_image);
    //cv::Mat my_depth = convertPCDToDepth(scene_cloud);

    this->marker_id = 0;            // unique marker id to each visualisations

    //finds intersections between the bounding boxes and returns as map
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> intersections = findIntersectingBoundingBoxes(detections);
    pcl::PointCloud<pcl::PointXYZ>::Ptr published_table_tops(new pcl::PointCloud <pcl::PointXYZ>);

#ifdef VISUALISATIONS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_coloured(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*scene_cloud, *scene_coloured);
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd = findTableTop(plane_coefficients, box_pcd);
        //too small table pcds can be ignored.
        if(table_top_pcd->size() <= 10)
            continue;
        //publish this table top to RVIZ
        //*published_table_tops += *table_top_pcd;

#ifdef VISUALISATIONS
//        for(int i=0;i<scene_coloured->size();i++) {
//            pcl::PointXYZ point = scene_cloud->at(i);
//            scene_coloured->points[i].r = 170;
//            scene_coloured->points[i].g = 175;
//            scene_coloured->points[i].b = 175;
//            float plane_value = plane_coefficients->values[0]*point.x + plane_coefficients->values[1]*point.y + plane_coefficients->values[2]*point.z + plane_coefficients->values[3];
//
//            if(abs(plane_value) <= PLANE_DISTANCE_THRESHOLD)
//            {
//                scene_coloured->points[i].g = 125;
//            }
//        }
//        // paint the table top separately - THIS IS A BUG IN THE CODE
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_top_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
//        pcl::copyPointCloud(*table_top_pcd, *table_top_colored);
//        *scene_coloured += *table_top_colored;

        //for open3d visualisations
        this->o3dviz->scene_cloud = scene_cloud;
        this->o3dviz->object_top_cloud = table_top_pcd;
        this->o3dviz->depth_image = this->depth_image;
#endif
        //find the Minimum area polygon of this table top
        std::vector <Eigen::Vector3f> polygon_points = findMinimumAreaShape(table_top_pcd);
        std::vector <std::vector<Eigen::Vector3f>> locations = findPossiblePlacements(polygon_points,
                                                                                      PADDING_OFFSET / 2.0,
                                                                                      WHEELCHAIR_SAMPLING_DISTANCE);



#ifdef VISUALISATIONS
//    for(int i=0;i<polygon_points.size();i++)
//        {
//           pcl::PointXYZ sphere_center(polygon_points[i][0],polygon_points[i][1],polygon_points[i][2]);
//           viewer_approx->addSphere(sphere_center,0.1,"bbox"+std::to_string(this->marker_id++));
//        }
//
//    for(int i=0;i<locations.size();i++)
//        {
//        for(int j=0;j<locations[i].size();j++)
//            {
//                pcl::PointXYZ sphere_center(locations[i][j][0],locations[i][j][1],locations[i][j][2]);
//                //viewer_approx->addSphere(sphere_center,0.04,"bbox"+std::to_string(marker_id++));
//            }
//        }
    //open3d visualisations
    this->o3dviz->polygon_points = polygon_points;
    this->o3dviz->potential_placements = locations;
#endif
        //find the instantaneous transform between the "odom/world" and the camera_depth_optical_frame
        geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());

        std::vector <std::vector<Eigen::Vector3f>> locations_filtered = filterForFOV(locations, transformStamped);
        std::vector<double> wheelchairDimensions = {WHEELCHAIR_WIDTH, WHEELCHAIR_LENGTH, WHEELCHAIR_DEPTH};
        //ros::Time new_start = ros::Time::now();
        std::vector <std::vector<Eigen::Vector3f>> desirable_locations = filterForCollision(locations_filtered, wheelchairDimensions);
        std::vector<std::vector<Eigen::Quaternionf>> heading = calculateHeading(desirable_locations);
        //ros::Time new_end = ros::Time::now();
        //cout<<"TIME"<<(new_end-new_start).toSec()<<endl;
        //std::vector <std::vector<Eigen::Vector3f>> desirable_locations = locations_filtered;
        //desirable_locations = locations;
#ifdef VISUALISATIONS
//        for(int i=0;i<desirable_locations.size();i++)
//            {
//              Eigen::Quaternionf wheelchair_rotation;
//                double wheelchair_rotation_radian = -0;
//            //the wheelchair rotation angle along each edge
//            if(desirable_locations[i].size() >= 2){
//                Eigen::Vector3f point1 = desirable_locations[i].front(), point2 = desirable_locations[i].back(); // first and last point
//                double slope = (point2[1]-point1[1])/(point2[0]-point1[0]);
//                wheelchair_rotation_radian = atan2((point2[1]-point1[1]),(point2[0]-point1[0]));
//                //cout<<i<<"\t"<<wheelchair_rotation_radian<<endl;
//                wheelchair_rotation_radian += -1.57;  // to make the array perpendicular
//            }
//             wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian,
//                                                Eigen::Vector3f::UnitZ());  // rotation along z-axis only
//            for(int j=0;j<desirable_locations[i].size();j++)
//                {
//                    pcl::PointXYZ sphere_center(desirable_locations[i][j][0],desirable_locations[i][j][1],desirable_locations[i][j][2]);
//                    viewer_approx->addSphere(sphere_center,0.04,"bbox"+std::to_string(marker_id++));
//                    pcl::PointXYZ arrow_end(sphere_center.x - 0.5*cos(wheelchair_rotation_radian),sphere_center.y - 0.5*sin(wheelchair_rotation_radian),sphere_center.z);
//                    viewer_approx->addArrow(arrow_end,sphere_center,1,0,0,false,"arrow"+std::to_string(marker_id++));
//                 //   viewer_approx->addCube(desirable_locations[i][j],wheelchair_rotation,0.04,0.04,0.04,"cube"+std::to_string(this->marker_id++));
//                }
//            }

#endif

        // find weights for each desirable location
        std::vector <std::vector<Eigen::Vector3f>> clusters;
        //std::vector <std::vector<double>> position_weights = calculatePositionalWeights(desirable_locations, clusters);
        std::vector <std::vector<double>> position_weights = findPositionalWeights(desirable_locations, clusters);
        std::vector <std::vector<double>> visibility_weights = calculateVisibilityWeights(desirable_locations, heading, wheelchairDimensions, transformStamped);

        //std::vector <std::vector<double>> visibility_weights = position_weights;
        //std::vector<std::vector<std::pair<double,double>>> distances = projectOnDepthImage(clusters,wheelchairDimensions);

        //append these results into main results
        for(std::vector <Eigen::Vector3f> &salient_location:desirable_locations)
            all_clusters.push_back(salient_location);
//        for (std::vector <Eigen::Vector3f> cluster:clusters)
//            all_clusters.push_back(cluster);


        for (int i = 0; i < position_weights.size(); i++) {
            ASSERT(position_weights[i].size() == visibility_weights[i].size(),
                   "Mismatch Visibility and Position Weights size");
            for(int j = 0;j<position_weights[i].size();j++) {
                position_weights[i][j] *= visibility_weights[i][j];
            }
            all_weights.push_back(position_weights[i]);
        }

        //add the heading

        for(std::vector<Eigen::Quaternionf> &head:heading)
            all_heading.push_back(head);

//#ifdef VISUALISATIONS
//        for(int x = 0;x<distances.size();x++)
//        {
//            for(int y = 0;y<distances[x].size();y++)
//                {
//                    pcl::PointXYZ position(clusters[x][y][0],clusters[x][y][1],clusters[x][y][2]);
//                    viewer_approx->addText3D("    "+std::to_string(distances[x][y].first)+" - "+ std::to_string(distances[x][y].second), position,0.05,0,1,0,std::to_string(marker_id++),0);
//                }
//        }
//#endif

    }

    //publish this table top to RVIZ
    publishTableTop(published_table_tops);

#ifdef VISUALISATIONS
//    viewer_approx->setBackgroundColor(255, 255, 255);
//    viewer_approx->addPointCloud<pcl::PointXYZRGB> (scene_coloured, "3D Viewer");
//    viewer_approx->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
//    viewer_approx->addCoordinateSystem (1.0);
//    viewer_approx->initCameraParameters ();
//    viewer_approx->removeCoordinateSystem();
//    while (!viewer_approx->wasStopped()) {
//        viewer_approx->spinOnce(100);
//        sleep(0.1);
//    }
    //exit(0);
    this->o3dviz->all_clusters = all_clusters;
    this->o3dviz->all_weights = all_weights;
    this->o3dviz->all_heading = all_heading;
    this->o3dviz->publishData();
#endif

    //publish the desirable locations and weights for cost map
    publishLocations(all_clusters, all_weights, all_heading);
    ros::Time end = ros::Time::now();
    cout << "Total Time : " << (end - start).toSec() << endl;
}

DetectTableApprox::~DetectTableApprox() {

}

std::vector<Eigen::Vector3f> DetectTableApprox::addInbetweenPaddedLines(std::vector<Eigen::Vector3f> padded_approx_polygon)
{
    std::vector<Eigen::Vector3f> padded_polygon(padded_approx_polygon.size()*2);
    for(int i=0;i<padded_approx_polygon.size();i+=2)
    {
        //cout<<padded_approx_polygon[i][0]<<"\t"<<padded_approx_polygon[i][1]<<" ---> "<<padded_approx_polygon[i+1][0]<<"\t"<<padded_approx_polygon[i+1][1]<<endl;
        padded_polygon[i*2] = padded_approx_polygon[i];
        padded_polygon[i*2 + 1] = padded_approx_polygon[i+1];
        padded_polygon[i*2 + 2] = padded_approx_polygon[i+1];
        if(i!=0)
            padded_polygon[i*2-1] = padded_approx_polygon[i];
    }

    // join the last edge
    padded_polygon[padded_polygon.size()-1] = padded_approx_polygon[0];
    return padded_polygon;
}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>
DetectTableApprox::findIntersectingBoundingBoxes(std::vector <desirable_locations::votenetDetection> &detections) {
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> intersection_list;                   // stores the merged pcds by index
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pointclouds;               //Stores pcds inside the bounding boxes with index
    for (int i = 0; i < detections.size(); i++) {
        //check for threshold - process only objects which are above the threshold
        if (detections[i].object_score < OBJECTNESS_THRESHOLD)
            continue;
        //extract the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = extractBBoxPCD(detections[i].bbox_3d);
        extracted_pointclouds[i] = box_pcd;
    }
    //finds overlapping point clouds
    intersection_list = computeOverlap(extracted_pointclouds);
    return intersection_list;
}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>
DetectTableApprox::computeOverlap(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> &extracted_pcd) {

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
        //create a convex hull of ith extracted pcd
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
                    if (filtered->size() >= NUM_OF_POINTS_FOR_OVERLAPP) {
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
            //OPTIMISATION: THIS WILL HAVE MANY DUPLICATE POINTS AS WELL.
            *merged_bbox[it->first] += *extracted_pcd[it->second.at(i)];
        }
    }
    return merged_bbox;
}

void
DetectTableApprox::DFS(std::map<int, std::vector<int>> &intersections, std::set<int> &visited, std::vector<int> &path,
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
DetectTableApprox::findConnectedComponents(std::map<int, std::vector<int>> &intersections) {
    cout << "Merged BBOX: " << endl;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTableApprox::extractBBoxPCD(std::vector <geometry_msgs::Point> &bbox_corners) {
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
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTableApprox::findTableTop(pcl::ModelCoefficients::Ptr plane_coefficients,
                                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cluster(new pcl::PointCloud <pcl::PointXYZ>); //returned object

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pcd(new pcl::PointCloud <pcl::PointXYZ>);
    for (int i = 0; i < this->scene_cloud->size(); i++) {
        pcl::PointXYZ point = this->scene_cloud->at(i);
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);
    // Apply statistical outlier for removing noise and smaller points from plane pcd
    // link https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html
    pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
    sor.setInputCloud(plane_pcd);
    sor.setMeanK(STATISTICAL_OUTLIER_NEIGHBOURS);
    sor.setStddevMulThresh(STATISTICAL_OUTLIER_STD);
    sor.filter(*cloud_filtered);
    //simpleVis(cloud_filtered,"statistical");

    // Region Growing segmentation
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_filtered);
    normal_estimator.setKSearch(STATISTICAL_OUTLIER_NEIGHBOURS);
    normal_estimator.compute(*normals);

    //Region growing segmentation to find clusters in the plane pcd.
    pcl::RegionGrowing <pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(25000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(REGION_GROWING_NEIGHBOURS);
    reg.setInputCloud(cloud_filtered);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(REGION_GROWING_SMOOTHNESS);
    reg.setCurvatureThreshold(REGION_GROWING_CURVATURE);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
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
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
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
        if (filtered->size() >= REGION_GROWING_OVERLAPPING) {
            std::cout << filtered->size() << " Selected Cluster " << j << std::endl;
            *merged_cluster += *cloud_cluster;
        }
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        j++;
    }
    //simpleVis(merged_cluster,"test");
    return merged_cluster;
}

/*
 *  Publish the table top pcd for error checking
 *  Status: Testing code - not optimised
 */
void DetectTableApprox::publishTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd) {
    sensor_msgs::PointCloud2 pc2_msg_;
    pcl::toROSMsg(*table_top_pcd, pc2_msg_);
    pc2_msg_.header.frame_id = src_frame.c_str();
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
    }
    std::cout << "Convex Polygon Vertices : " << locations.size() << std::endl;
    return locations;

}

std::vector <std::vector<Eigen::Vector3f>>
DetectTableApprox::findPossiblePlacements(std::vector <Eigen::Vector3f> &approx_polygon,
                                          double padding_offset, double location_sampling_distance) {
    int total_collsion_points = 0;
    std::vector <std::vector<Eigen::Vector3f>> collsion_points;
    // add the first point back again so that we have a complete loop
    approx_polygon.push_back(approx_polygon[0]);

    std::vector <Eigen::Vector3f> padded_gapped_approx_polygon = calculatePaddedLines(approx_polygon, padding_offset);
    std::vector <Eigen::Vector3f> padded_approx_polygon = addInbetweenPaddedLines(padded_gapped_approx_polygon);

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
DetectTableApprox::calculatePaddedLines(std::vector <Eigen::Vector3f> &approx_polygon, double padding_offset) {
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
        double height = std::max((double) approx_polygon[i][2],WHEELCHAIR_DEPTH/2.0);
        //the z height is max of table height or wheelchair height
        if (cv::pointPolygonTest(contour, mid_point1, false) == -1) {
            padded_edges.push_back(Eigen::Vector3f(padded_point1.x, padded_point1.y, height));
            padded_edges.push_back(Eigen::Vector3f(padded_point3.x, padded_point3.y, height));
        } else {
            padded_edges.push_back(Eigen::Vector3f(padded_point2.x, padded_point2.y, height));
            padded_edges.push_back(Eigen::Vector3f(padded_point4.x, padded_point4.y, height));
        }
    }
    cout << "Points in Padded Edges : " << padded_edges.size() << endl;
    return padded_edges;
}


std::vector <std::vector<Eigen::Vector3f>>
DetectTableApprox::filterForFOV(std::vector <std::vector<Eigen::Vector3f>> &locations, geometry_msgs::TransformStamped transformStamped) {
    std::vector <std::vector<Eigen::Vector3f>> locations_FOV_filtered;
    //find the instantaneous transform between the "odom/world" and the camera_depth_optical_frame
    //geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
    boost::mutex::scoped_lock lock(access_guard_);
    //camera info subscribed topic
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info); // fill cam_model with CameraInfo

    /*image_geometry::PinholeCameraModel cam_model; // init cam_model
    sensor_msgs::CameraInfo cam_info;
    cam_info.height = 480, cam_info.width = 640;
    cam_info.distortion_model = "plumb_bob";
    cam_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info.K = {554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.P = {554.254691191187, 0.0, 320.5, -0.0, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_model.fromCameraInfo(cam_info); // fill cam_model with CameraInfo
    */
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
        //add edge only if we have some valid locations
        if(filter_FOV.size() >= 2)   //atleast tow since we need tow points in the next step
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


 /*   std::vector <std::vector<Eigen::Vector3f>> desirable_locations;
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

        }
        total_points += filtered_points.size();
        desirable_locations.push_back(filtered_points);
    }
    std::cout << "Found " << total_points << " non colliding  points along " << desirable_locations.size()
              << " edges" << endl;
    return desirable_locations;*/

    std::vector <std::vector<Eigen::Vector3f>> desirable_locations;
    double wheelchair_width = wheelchair_dimensions[0], wheelchair_length = wheelchair_dimensions[1], wheelchair_depth = wheelchair_dimensions[2];

    int total_points = 0;
    for (int ii = 0; ii < locations.size(); ii++) {

        double wheelchair_rotation_radian = 0;
        Eigen::Quaternionf wheelchair_rotation;

        //the wheelchair rotation angle along each edge
        if (locations[ii].size() >= 2) {
            Eigen::Vector3f point1 = locations[ii].front(), point2 = locations[ii].back();  // first and last point
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
            for (int i = 0;i < data->GetNumberOfPoints(); i++)            // returns all the edges 12*2 = 24 bidirectional
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
            bool point_inside = false;
            for (int nIndex = 0; nIndex < scene_cloud->points.size() && !point_inside; nIndex++) {
                if(pcl::isXYPointIn2DXYPolygon(scene_cloud->points[nIndex], *cube_pcd))
                    point_inside = true;
            }

            if(!point_inside)
                filtered_points.push_back(locations[ii][j]);      // if no point inside the convex hull

        }
        total_points += filtered_points.size();
        if(filtered_points.size() >=2 )
            desirable_locations.push_back(filtered_points);
    }
    std::cout << "Found " << total_points << " non colliding  points along " << desirable_locations.size()
              << " edges" << endl;
    return desirable_locations;
}

std::vector <std::vector<double>>
DetectTableApprox::findPositionalWeights(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
                                         std::vector <std::vector<Eigen::Vector3f>> &clusters) {
    /*
    * Points which are close to corners and other chairs should be weighted less
    */
    std::vector <std::vector<double>> weights; // final value to be returned
    for(std::vector<Eigen::Vector3f> &locations:desirable_locations)
    {
        std::vector<double> weight_in_cluster(locations.size(), 1);
        weights.push_back(weight_in_cluster);
    }

/*
    std::vector<int> along_edge;               // tells about the edge of the point
    clusters = makeClusters(desirable_locations, along_edge);     // clusters formed by the points
    for (std::vector <Eigen::Vector3f> cluster:clusters) {
        //initially assign weight 1 to every point in the cluster
        std::vector<double> weight_in_cluster(cluster.size(), 1);
        //the sampling is such that the two extremes are our max values
        Eigen::Vector3f left_extreme = cluster[0], right_extreme = cluster[cluster.size() - 1];

        weights.push_back(weight_in_cluster);
        assert(cluster.size() == weight_in_cluster.size());
    }
*/

    std::cout << "Weight calculated for each point" << std::endl;
    cout << "--------------------------------------------------------------------------" << endl;
    return weights;
}


std::vector <std::vector<double>>
DetectTableApprox::calculatePositionalWeights(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,
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

geometry_msgs::TransformStamped DetectTableApprox::listenTransform(std::string des_frame, std::string src_frame) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(des_frame.c_str(), src_frame.c_str(),
                                                    ros::Time(0),
                                                    ros::Duration(3.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform %s to %s: %s", des_frame.c_str(), src_frame.c_str(), ex.what());
    }
    return transformStamped;
}

/*
 * Calculate the visibility weights -  8 corners - 0.1 and centre -0.2
 * Add the weight if the point is visible using depth image.
 */
std::vector <std::vector<double>>
DetectTableApprox::calculateVisibilityWeights(std::vector <std::vector<Eigen::Vector3f>> &locations, std::vector<std::vector<Eigen::Quaternionf>> &heading,
                                              std::vector<double> &wheelchair_dimensions, geometry_msgs::TransformStamped transformStamped) {
    boost::mutex::scoped_lock lock(access_guard_);
    //returned value
    std::vector <std::vector<double>> visibility_weights;
    //camera parameters
    float centre_x = cam_info.K[2];
    float centre_y = cam_info.K[5];
    float focal_x = cam_info.K[0];
    float focal_y = cam_info.K[4];

    double wheelchair_width = wheelchair_dimensions[0], wheelchair_length = wheelchair_dimensions[1], wheelchair_depth = wheelchair_dimensions[2];

    //for each location
    for (int ii = 0; ii < locations.size(); ii++) {
//        double wheelchair_rotation_radian = 0;
//        Eigen::Quaternionf wheelchair_rotation;
//        //the wheelchair rotation angle along each edge
//        if (locations[ii].size() >= 2) {
//            Eigen::Vector3f point1 = locations[ii][0], point2 = locations[ii][locations[ii].size() -1]; // first and last point
//            wheelchair_rotation_radian = atan2((point2[1] - point1[1]), (point2[0] - point1[0]));
//        }
//        wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian, Eigen::Vector3f::UnitZ());  // rotation along z-axis only

        std::vector<double> cluster_weight;
        geometry_msgs::PointStamped transformed_pt, initial_pt;

        cout << locations[ii].size() << " : ";
        for (int j = 0; j < locations[ii].size(); j++) {

            double current_weight = 0.0;
            //calculate for center of the cube
            //transform the center
            initial_pt.point.x = locations[ii][j][0], initial_pt.point.y = locations[ii][j][1], initial_pt.point.z = locations[ii][j][2];
            tf2::doTransform(initial_pt, transformed_pt, transformStamped);
            float z = transformed_pt.point.z;                      //to mm
            float u = (transformed_pt.point.x * focal_x) / z;        //to mm transformed - parking
            float v = (transformed_pt.point.y * focal_y) / z;        // to mm
            int pixel_pos_x = (int) (u + centre_x);
            int pixel_pos_y = (int) (v + centre_y);

            //center is always visible
            if (pixel_pos_x >= 0 && pixel_pos_x <= cam_info.width && pixel_pos_y >= 0 && pixel_pos_y <= cam_info.height) {
                //cv::circle( normalized, uv, 5, cv::Scalar( 255, 255, 255 ), CV_FILLED);
                float raw = depth_image.at<float>(pixel_pos_y, pixel_pos_x);
                // if the point is above the depth point then its visible
                if (std::isnan(raw) || depth_image.at<float>(pixel_pos_y, pixel_pos_x) >= transformed_pt.point.z) {
                    current_weight += 0.2;
                    //cv::circle( normalized, uv, 2, cv::Scalar( 0, 255, 255 ), CV_FILLED);
                }
            }

            vtkSmartPointer <vtkDataSet> data = pcl::visualization::createCube(locations[ii][j], heading[ii][j],
                                                                               wheelchair_width, wheelchair_length,
                                                                               wheelchair_depth);
            std::set <std::vector<double>> cube_corners;
            for (int i = 0;i < data->GetNumberOfPoints(); i++) {            // returns all the edges 12*2 = 24 bidirectional
                std::vector<double> edges{data->GetPoint(i)[0], data->GetPoint(i)[1], data->GetPoint(i)[2]};
                cube_corners.insert(edges);
            }

            for (std::set < std::vector < double >> ::iterator it = cube_corners.begin(); it != cube_corners.end(); it++)
            {
                initial_pt.point.x = (*it)[0], initial_pt.point.y = (*it)[1], initial_pt.point.z = (*it)[2];
                tf2::doTransform(initial_pt, transformed_pt, transformStamped);
                float z = transformed_pt.point.z;                      //to mm
                float u = (transformed_pt.point.x * focal_x) / z;        //to mm transformed - parking
                float v = (transformed_pt.point.y * focal_y) / z;        // to mm
                int pixel_pos_x = (int) (u + centre_x);
                int pixel_pos_y = (int) (v + centre_y);
                float raw = depth_image.at<float>(pixel_pos_y, pixel_pos_x);
                if (pixel_pos_x >= 0 && pixel_pos_x <= cam_info.width && pixel_pos_y >= 0 && pixel_pos_y <= cam_info.height) {
                    // if the point is above the depth point then its visible
                    if (std::isnan(raw) || depth_image.at<float>(pixel_pos_y, pixel_pos_x) >= transformed_pt.point.z)
                        current_weight += 0.1;
                }
            }
            cout<<current_weight << " , ";
            cluster_weight.push_back(current_weight);
        }
        cout<<endl;
        visibility_weights.push_back(cluster_weight);
    }
    //cv::imshow("foo", normalized);
    //cv::waitKey(0);
    return visibility_weights;
}

//donot pass by reference here.
std::vector <std::vector<std::pair < double, double>>> DetectTableApprox::projectOnDepthImage (std::vector<std::vector < Eigen::Vector3f>> locations,std::vector<double> wheelchair_dimensions)
{
    float centre_x = cam_info.K[2];
    float centre_y = cam_info.K[5];
    float focal_x = cam_info.K[0];
    float focal_y = cam_info.K[4];

    std::vector <std::vector<std::pair < double, double>>> camera_distance;

    //find the instantaeous transform between the "odom/world" and the camera_depth_optical_frame
    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    tf::transformMsgToEigen (transformStamped.transform, transform);
    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*this->scene_cloud, *camera_cloud, transform);
   // tf2::fromMsg(transformStamped, transform);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("sample cloud"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (camera_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    geometry_msgs::PointStamped transformed_pt, initial_pt;
    for(int i = 0; i<locations.size(); i++)
    {
        double wheelchair_rotation_radian = 0;
        Eigen::Quaternionf wheelchair_rotation;
        //the wheelchair rotation angle along each edge
        if (locations[i].size()>= 2)
        {
            Eigen::Vector3f point1 = locations[i][0], point2 = locations[i][locations[i].size() - 1]; // first and last point
            wheelchair_rotation_radian = atan2((point2[1] - point1[1]), (point2[0] - point1[0]));
        }
        wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian, Eigen::Vector3f::UnitZ());  // rotation along z-axis only

        std::vector <std::pair<double, double>> distances;
        std::pair<double, double> p;
        for( int j = 0; j<locations[i].size(); j++)
        {
            vtkSmartPointer <vtkDataSet> data = pcl::visualization::createCube(locations[i][j], wheelchair_rotation,
                                                                       WHEELCHAIR_WIDTH, WHEELCHAIR_LENGTH,
                                                                       WHEELCHAIR_DEPTH);
            std::set <std::vector<double>> cube_corners;
            for (int ii = 0; ii<data->GetNumberOfPoints();ii++)            // returns all the edges 12*2 = 24 bidirectional
            {
                std::vector<double> edges{data->GetPoint(ii)[0], data->GetPoint(ii)[1], data->GetPoint(ii)[2]};
                cube_corners.insert(edges);
            }
            //trnasform each corner of the cube

            for(std::set<std::vector < double>>::iterator it = cube_corners.begin();it!=cube_corners.end();it++)
            {
                initial_pt.point.x = (*it)[0], initial_pt.point.y = (*it)[1], initial_pt.point.z = (*it)[2];
                tf2::doTransform(initial_pt, transformed_pt, transformStamped);
            }
            //transform the center as well.
            initial_pt.point.x = locations[i][j][0], initial_pt.point.y = locations[i][j][1], initial_pt.point.z = locations[i][j][2];
            tf2::doTransform(initial_pt, transformed_pt, transformStamped);

            p.second = -999999; //default value if no match found
            double curr_min = INT_MAX;
            Eigen::Vector4f line_pt(transformed_pt.point.x, transformed_pt.point.y, transformed_pt.point.z, 1);
            Eigen::Vector4f origin(0, 0, 0, 1);
            Eigen::Vector4f line_dir = origin - line_pt;
            pcl::PointXYZ closest_point = camera_cloud->points[0];
            //search for the closest point in the point cloud
            for(int x = 0; x < camera_cloud->points.size(); x++)
            {
                // Calculate the distance from the point to the line
                // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
                Eigen::Vector4f pt(camera_cloud->points[x].x, camera_cloud->points[x].y, camera_cloud->points[x].z,1);
                double dist = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
                if(!std::isnan(dist) && dist <= 0.01 &&  dist<curr_min )
                {
                    p.second = camera_cloud->points[x].z;
                    curr_min = dist;
                    closest_point = camera_cloud->points[x];
                }
            }



        float z = transformed_pt.point.z;                      //to mm
        float u = (transformed_pt.point.x * focal_x) / z;        //to mm transformed - parking
        float v = (transformed_pt.point.y * focal_y) / z;        // to mm
        int pixel_pos_x = (int) (u + centre_x);
        int pixel_pos_y = (int) (v + centre_y);

        z = closest_point.z;                      //to mm
        u = (closest_point.x * focal_x) / z;        //to mm // closest - ray point
        v = (closest_point.y * focal_y) / z;        // to mm
        int pixel_pos_x_d = (int) (u + centre_x);
        int pixel_pos_y_d = (int) (v + centre_y);
        if((i+j) % 1 == 0)
        {
            pcl::PointXYZ sphere_center(transformed_pt.point.x, transformed_pt.point.y, transformed_pt.point.z);
            viewer->addSphere(sphere_center,0.02,"bbox"+std::to_string(this->marker_id++));
            viewer->addSphere(closest_point, 0.05,"bbox"+std::to_string(this->marker_id++));
            viewer->addLine(sphere_center, closest_point, 1,1,1,"arrow"+std::to_string(this->marker_id++));
            viewer->addLine(sphere_center, pcl::PointXYZ(0, 0, 0),1,1,0,"arrow"+std::to_string(this->marker_id++));
            cout<<"Parking Spot : " << pixel_pos_x<<" " << pixel_pos_y << " " <<  transformed_pt.point.z << " Ray Point : " << pixel_pos_x_d<<" " << pixel_pos_y_d << " Depth Image : " <<  depth_image.at<float>(pixel_pos_y, pixel_pos_x) << " "<< closest_point.z<<endl;
        }

//      cv::circle( normalized, uv, 5, cv::Scalar( 255, 255, 255 ), CV_FILLED);

        p.second = depth_image.at<float> (pixel_pos_y,pixel_pos_x);
        p.first = transformed_pt.point.z;
        distances.push_back(p);
    }
        camera_distance.
        push_back(distances);
    }
//    cv::imshow("foo", normalized);
//    cv::waitKey(4000);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    sleep(0.1);
    }
    return camera_distance;
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
 * Status: Finalised
 */
void DetectTableApprox::publishLocations(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
                                         std::vector <std::vector<double>> &weights, std::vector<std::vector<Eigen::Quaternionf>> &heading) {
    assert(desirable_locations.size() == weights.size());
    if (desirable_locations.size() == 0)
    {
        cout<<"No Locations found. Nothing to publish."<<endl;
        return;
    }
    desirable_locations::locationArray all_locations;
    for (int i = 0; i < desirable_locations.size(); i++) {
        for (int j = 0; j < desirable_locations[i].size(); j++) {
            assert(desirable_locations[i].size() == weights[i].size());
            assert(desirable_locations[i].size() == heading[i].size());

            desirable_locations::desiredLocation table_location;
            //assign the parking spots
            table_location.location.x = desirable_locations[i][j][0];
            table_location.location.y = desirable_locations[i][j][1];
            table_location.location.z = desirable_locations[i][j][2];
            //assign the weight
            table_location.location_weight = weights[i][j];
            //heading
            tf::quaternionEigenToMsg(heading[i][j].cast<double>(), table_location.heading);
            all_locations.desired_locations.push_back(table_location);
        }
    }
    //publish the locations for cost map calculations...
    pub_approx.publish(all_locations);
}

std::vector<std::vector<Eigen::Quaternionf>> DetectTableApprox::calculateHeading(std::vector<std::vector<Eigen::Vector3f>> &locations){
    std::vector<std::vector<Eigen::Quaternionf>> heading;
    for(int i=0;i<locations.size();i++)
    {
        Eigen::Quaternionf wheelchair_rotation;
        double wheelchair_rotation_radian = -0;
        //the wheelchair rotation angle along each edge
        if(locations[i].size() >= 2){
            Eigen::Vector3f point1 = locations[i].front(), point2 = locations[i].back(); // first and last point
            double slope = (point2[1]-point1[1])/(point2[0]-point1[0]);
            wheelchair_rotation_radian = atan2((point2[1]-point1[1]),(point2[0]-point1[0]));
            wheelchair_rotation_radian += -1.57;  // to make the array perpendicular
        }
        wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian,
                                                Eigen::Vector3f::UnitZ());  // rotation along z-axis only

        std::vector<Eigen::Quaternionf> edge_heading(locations[i].size(), wheelchair_rotation);
        heading.push_back(edge_heading);
    }
    return heading;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg) {
    //std::cout<<"===============================cameraInfoCallback===============================" <<std::endl;
    //image_geometry::PinholeCameraModel cam_model_;
    // cam_model_.fromCameraInfo(info_msg);
    boost::mutex::scoped_lock lock(access_guard_);
    cam_info = *info_msg;
    //cam_model.fromCameraInfo(info_msg); // fill cam_model with CameraInfo
    // projection_matrix is the matrix you should use if you don't want to use project3dToPixel() and want to use opencv API
    // cv::Matx34d projection_matrix=cam_model_.fullProjectionMatrix();
    // std::cout<<cam_model_.project3dToPixel(cv::Point3d(-0.1392072,-0.02571392, 2.50376511) )<<std::endl;
}

cv::Mat convertPCDToDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud) {
    float centre_x = cam_info.K[2];
    float centre_y = cam_info.K[5];
    float focal_x = cam_info.K[0];
    float focal_y = cam_info.K[4];
    cv::Mat cv_image = cv::Mat(cam_info.height, cam_info.width, CV_32FC1);


    for (int i = 0; i < scene_cloud->points.size(); i++) {
        if (scene_cloud->points[i].z == scene_cloud->points[i].z) {
            float z = scene_cloud->points[i].z;                      //to mm
            float u = (scene_cloud->points[i].x * focal_x) / z;        //to mm
            float v = (scene_cloud->points[i].y * focal_y) / z;        // to mm
            int pixel_pos_x = (int) (u + centre_x);
            int pixel_pos_y = (int) (v + centre_y);
            if (pixel_pos_x > (cam_info.width - 1)) {
                pixel_pos_x = cam_info.width - 1;
            }
            if (pixel_pos_y > (cam_info.height - 1)) {
                pixel_pos_y = cam_info.height - 1;
            }
            cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
        }
    }

//    double max = 0.0;
//    cv::Mat temp;
//    cv::minMaxLoc(cv_image, 0, &max, 0, 0);
//    cv_image.convertTo(temp, CV_32F, 1.0/max, 0)  ;

    //cv_image.convertTo(cv_image,CV_32F);
//    cv::imshow("depth_image", temp);
//    cv::waitKey(0);
    return cv_image;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTableApprox::convertDepthToPCD(sensor_msgs::Image depth_msg) {
    float center_x = cam_info.K[2];
    float center_y = cam_info.K[5];
    float camera_fx = cam_info.K[0];
    float camera_fy = cam_info.K[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    cloud->height = depth_msg.height;
    cloud->width = depth_msg.width;
    cloud->is_dense = false;

    cloud->points.resize(cloud->height * cloud->width);

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 1.0;
    float constant_x = unit_scaling / camera_fx;
    float constant_y = unit_scaling / camera_fy;
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
    const float *depth_row = reinterpret_cast<const float *>(&depth_msg.data[0]);
    int row_step = depth_msg.step / sizeof(float);

    for (int v = 0; v < (int) cloud->height; ++v, depth_row += row_step) {
        for (int u = 0; u < (int) cloud->width; ++u) {
            pcl::PointXYZ &pt = *pt_iter++;
            float depth = depth_row[u];

            // Missing points denoted by NaNs
            if (!std::isfinite(depth)) {
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            // Fill in XYZ
            pt.x = (u - center_x) * depth * constant_x;
            pt.y = (v - center_y) * depth * constant_y;
            pt.z = depth;
        }
    }

    pcl::PCLPointCloud2::Ptr camera_scene(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud, *camera_scene);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    // Create the filtering object
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    sor.setInputCloud(camera_scene);
    sor.setLeafSize(0.015f, 0.015f, 0.015f);
    sor.filter(*cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_filtered(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_filtered, *camera_filtered);



//    // paint the table top separately
////    pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_color(new pcl::PointCloud<pcl::PointXYZRGB>());
////    pcl::copyPointCloud(*, *s_color);
//        *cloud += *scene_cloud;
////    cout<<"Point Cloud Size:\t" << cloud->size()<<endl;
////    // Display point cloud map
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("sample cloud"));
//    viewer->setBackgroundColor (0, 0, 0);
//   // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//    viewer->addPointCloud<pcl::PointXYZ> (camera_filtered, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(100);
//        sleep(0.1);
//      }

    return camera_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropBounds(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_cloud, double z_min, double z_max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (unfiltered_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*cloud_filtered_xyz);
    return cloud_filtered_xyz;
}


void callbackDesirableLocations(const desirable_locations::detectionArray::ConstPtr &message) {
//    ros::Time start_time = ros::Time::now();
    //convert the depth image in float values
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message->depth_image, message);
    cv::Mat d_image;
    cv_ptr->image.convertTo(d_image, CV_32F);

//   uncomment this for depth image visualisations
/*    double max = 0.0;
    cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
    cv_ptr->image.convertTo(normalized, CV_32F, 1.0 / max, 0);
    cv::imshow("foo", normalized);
    cv::waitKey(20);
*/
    sensor_msgs::PointCloud2 cloud = message->scene_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromROSMsg(cloud, *scene_cloud);  // assgin the scene cloud
//    cout << "Scene Cloud : width " << scene_cloud->width << " Height : " << scene_cloud->height << endl;

//    Create the filtering object - voxelize again for faster processing the pcd
    std::vector<float> leaf_size = {0.01f, 0.01f, 0.01f};
    pcl::VoxelGrid <pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2::Ptr scene_voxelised_pointcloud2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr scene_pointCloud2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*scene_cloud, *scene_pointCloud2);
    sor.setInputCloud(scene_pointCloud2);
    sor.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    sor.filter(*scene_voxelised_pointcloud2);
//    printf("Scene Voxelised: width = %d, height = %d\n", scene_voxelised_pointcloud2->width, scene_voxelised_pointcloud2->height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_voxelised(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*scene_voxelised_pointcloud2, *scene_voxelised);

    pcl::PointCloud <pcl::PointXYZ>::Ptr cropped_voxelised(new pcl::PointCloud<pcl::PointXYZ>);
    cropped_voxelised = cropBounds(scene_voxelised,0.04,2.0);
    // for each input pcd detect table tops
    DetectTableApprox detectTable(cropped_voxelised, d_image, message->detections);
//    ros::Time end_time = ros::Time::now();
//    cout<<(end_time-start_time).toSec()<<endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "find_location_approx");
    ros::NodeHandle nh;
    //Topic to publish
    pub_approx = nh.advertise<desirable_locations::locationArray>("/pcl_processing/desirable_locations", 1);
    pub_table_approx = nh.advertise<sensor_msgs::PointCloud2>("/pcl_processing/filtered_table_top", 1);
    pub_o3d = nh.advertise<desirable_locations::o3dViz>("/o3d/viz",1);
    //subscribe to the votenet detection topic
    ros::Subscriber sub = nh.subscribe("/votenet/detections", 1, callbackDesirableLocations);
    //camera info subscriber
    ros::Subscriber sub_cam_info = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);
    // Run the subscriber until someone stops the program with Ctrl + C
    ros::spin();
    return 0;
}


