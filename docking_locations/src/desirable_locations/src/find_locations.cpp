//
// Created by vcr on 2020-10-29.
//

#include "desirable_locations/find_locations.h"

DetectTable::DetectTable(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,
                         std::vector <desirable_locations::votenetDetection> detections) {
    ros::Time start = ros::Time::now();
    this->scene_cloud = scene_cloud;

    //for each bounding box returns a list of intersections
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> intersections = findIntersectingBoundingBoxes(detections);
    cout<<(ros::Time::now()-start).toSec()<<endl;

    //RVIZ visualisations
    /*visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint","/desirable_locations"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting
    visual_tools_->setLifetime(1.0);*/

#ifdef VISUALISATIONS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_coloured(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*scene_cloud, *scene_coloured);
    for (int i = 0; i < scene_coloured->size(); i++) {
        scene_coloured->points[i].r = 250;
        scene_coloured->points[i].g = 255;
        scene_coloured->points[i].b = 255;
    }

//     // Clear the view
//       viewer->removeAllShapes();
//       viewer->removeAllPointClouds();
#endif
/*    // for each detection of table run the desirable function
    for (int i = 0; i < detections.size(); i++) {
        desirable_locations::votenetDetection detect_object = detections[i];
        //check for threshold - porcess only objects which are above the threshold
        if (detect_object.object_score < OBJECTNESS_THRESHOLD)
            continue;
        //extract the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = extractBBoxPCD(detect_object.bbox_3d);
        //simpleVis(box_pcd, "table" + std::to_string(i));*/
    for( std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it=intersections.begin();it!=intersections.end();it++)
    {
        //extracted+combined the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = it->second;
        // Try to fit a planar model in the plane.
        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        // gets the equation of the plane for table top.
        fitPlanarModel(box_pcd, plane_coefficients);   // returns the table_top in terms of plane
        //find the table top based on euclidean segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd = findTableTop(plane_coefficients,scene_cloud);
        //publish this table top to RVIZ
        publishTableTop(table_top_pcd);

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
        //find the Minimum area rectangle for this table top
        std::vector <Eigen::Vector3f> table_top_bbx, padded_table_top_bbx;
        bool isRectangle = false;
        double rotation_radian;
        float radius;
        Eigen::Vector3f circle_center;
        findMinimumAreaShape(table_top_pcd, PADDING_OFFSET, table_top_bbx, padded_table_top_bbx,rotation_radian, isRectangle, circle_center, radius);

#ifdef VISUALISATIONS
        //visualise the table top corners
        Eigen::Quaternionf bboxQuaternion;
        bboxQuaternion = Eigen::AngleAxisf(rotation_radian, Eigen::Vector3f::UnitZ());  // rotation along z-axis only
        if(isRectangle)
            {
            double length = sqrt((table_top_bbx[0]-table_top_bbx[1]).dot(table_top_bbx[0]-table_top_bbx[1]));
            double width = sqrt((table_top_bbx[1]-table_top_bbx[2]).dot(table_top_bbx[1]-table_top_bbx[2]));
            viewer->addCube(table_top_bbx[4], bboxQuaternion, width, length, 0.01, "bbox"+std::to_string(100));
            }
        else
            {
            pcl::PointXYZ sphere_center(circle_center[0],circle_center[1],circle_center[2]);
            viewer->addSphere(sphere_center,radius,"bbox"+std::to_string(100));
            }

#endif
        //find the desirable locations for wheelchair placement
        std::vector <std::vector<Eigen::Vector3f>> locations;
        if(isRectangle)
            locations= findPossiblePlacementsRectangle(padded_table_top_bbx, WHEELCHAIR_SAMPLING_DISTANCE);
        else
            locations = findPossiblePlacementsCircle(circle_center,radius+(PADDING_OFFSET/2.0),WHEELCHAIR_SAMPLING_DISTANCE);

        std::vector<std::vector<Eigen::Vector3f>> locations_filtered = filterForFOV(locations);
        std::vector<double> wheelchairDimensions = {WHEELCHAIR_WIDTH, WHEELCHAIR_LENGTH, WHEELCHAIR_DEPTH,
                                                    rotation_radian};
        std::vector <std::vector<Eigen::Vector3f>> desirable_locations = filterForCollision(locations_filtered,
                                                                                            wheelchairDimensions);
#ifdef VISUALISATIONS
        Eigen::Quaternionf wheelchairQuaternion;
        wheelchairQuaternion = Eigen::AngleAxisf(rotation_radian,Eigen::Vector3f::UnitZ());
        for(int i=0;i<desirable_locations.size();i++)  // along each edge
        {
            for(int j=0;j<desirable_locations[i].size();j=j+4)   // skip every 4 placements
            {
                //visualise the sphere at these locations
                //viewer->addCube(desirable_locations[i][j],wheelchairQuaternion,WHEELCHAIR_WIDTH,WHEELCHAIR_LENGTH,WHEELCHAIR_DEPTH,"cube"+std::to_string(100*i+j));
                viewer->addCube(desirable_locations[i][j],wheelchairQuaternion,0.04,0.04,0.04,"cube"+std::to_string(100*i+j));
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


        // publish the desirable locations for visualisations
        //publishDesirableLocations(desirable_locations,rotation_radian);
    }
    //simpleVis(scene_cloud, "scene");
#ifdef VISUALISATIONS
    viewer->addPointCloud<pcl::PointXYZRGB> (scene_coloured, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            sleep(0.1);
        }
    exit(0);
#endif

    //publish the desirable locations and weights for cost map
    publishLocations(all_clusters, all_weights);
    ros::Time end = ros::Time::now();
    cout<<"Total Time : "<< (end-start).toSec()<<endl;

}

DetectTable::~DetectTable() {
//    // Clear messages
//    visual_tools_->deleteAllMarkers();
//    visual_tools_->enableBatchPublishing();

}

std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> DetectTable::findIntersectingBoundingBoxes(std::vector <desirable_locations::votenetDetection> detections)
{
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> intersection_list;  // stores the merged extracted pcd index-wise
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pointclouds;
    for(int i=0;i<detections.size();i++)
    {
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

//Shivam Can be optimised more if needed.
std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> DetectTable::computeOverlap(std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pcd)
{
    std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> merged_bbox;
    // find all the matchings as list indexes
    std::map<int,std::vector<int>> intersections;

    //insert empty lists
    std::vector<int> matches;
    for(std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = extracted_pcd.begin();it!=extracted_pcd.end();it++)
    {
        intersections[it->first] = matches;
    }

    for(std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = extracted_pcd.begin();it!=extracted_pcd.end();it++)
    {
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
        for(std::map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator ptr = it;ptr!=extracted_pcd.end();ptr++)
        {
            if(it->first != ptr->first) // dont compare same extracted pcd and skip if already matched
            {
                    auto pos = std::find (intersections[it->first].begin(), intersections[it->first].end(), ptr->first);
                    if(pos==intersections[it->first].end())
                    {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud <pcl::PointXYZ>);
                        cropHullFilter.setInputCloud(ptr->second);
                        cropHullFilter.filter(*filtered);
                        if(filtered->size() >= 10) {
                            // they intersect we can add them
                            intersections[it->first].push_back(ptr->first);
                            intersections[ptr->first].push_back(it->first);
                        }
                    }
            }
        }
    }

    //find the connected components
    std::map<int,std::vector<int>> merged_indexes = findConnectedComponents(intersections);

    //merge the point cloud data
    for(std::map<int,std::vector<int>>::iterator it=merged_indexes.begin();it!=merged_indexes.end();it++)
    {
        merged_bbox[it->first] = extracted_pcd[it->first];
        for(int i=0;i<it->second.size();i++)
        {
            *merged_bbox[it->first] += *extracted_pcd[it->second.at(i)];
        }
    }
    return merged_bbox;

}

void DetectTable::DFS(std::map<int,std::vector<int>>intersections, std::set<int> &visited,std::vector<int> &path,int node)
{
    for(int i=0;i<intersections[node].size();i++)
    {
        if(visited.find(intersections[node][i]) == visited.end())
        {
            visited.insert(intersections[node][i]);
            path.push_back(intersections[node][i]);
            DFS(intersections,visited,path,intersections[node][i]);
        }
    }
}

std::map<int,std::vector<int>> DetectTable::findConnectedComponents(std::map<int,std::vector<int>>intersections)
{
    cout<<"Merged BBOX: "<<endl;
    std::set<int> visited;
    std::map<int,std::vector<int>> components;
    for(std::map<int,std::vector<int>>::iterator it= intersections.begin();it!=intersections.end();it++)
    {
        if(visited.find(it->first)==visited.end())
        {
            std::vector<int> path;
            path.push_back(it->first);
            visited.insert(it->first);
            DFS(intersections,visited,path,it->first);
            components[it->first] = path;
            for(int k:path)
                cout<<k<<" ----->> ";
        }
        cout<<endl;
    }
    return components;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTable::extractBBoxPCD(std::vector <geometry_msgs::Point> bbox_corners) {
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
DetectTable::fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr table_bbox_cloud,
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

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectTable::findTableTop(pcl::ModelCoefficients::Ptr plane_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pcd(new pcl::PointCloud <pcl::PointXYZ>);
    for(int i=0;i<scene_cloud->size();i++) {
        pcl::PointXYZ point = scene_cloud->at(i);
        float plane_value = plane_coefficients->values[0]*point.x + plane_coefficients->values[1]*point.y + plane_coefficients->values[2]*point.z + plane_coefficients->values[3];
        if(abs(plane_value) <= PLANE_DISTANCE_THRESHOLD)
        {
            plane_pcd->push_back(point);
        }
    }
    plane_pcd->width = plane_pcd->size ();
    plane_pcd->height = 1;
    plane_pcd->is_dense = true;
    //copy the points to pcd

    //simpleVis(plane_pcd,"plane_pcd");

    //https://pointclouds.org/documentation/tutorials/cluster_extraction.html
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (plane_pcd);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.15); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (plane_pcd);
    ec.extract (cluster_indices);

    //find the
    //assert(cluster_indices.size() == 1 && "Euclidean Clustering more than one cluster detected");

    //merge the cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*plane_pcd)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        *merged_cluster += *cloud_cluster;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        j++;
    }
    return merged_cluster;

}

void DetectTable::publishTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd){
    sensor_msgs::PointCloud2 pc2_msg_;
    pcl::toROSMsg(*table_top_pcd,pc2_msg_ );
    pc2_msg_.header.frame_id = "odom";
    pc2_msg_.header.stamp = ros::Time::now();
    pub_table.publish(pc2_msg_);
    ros::spinOnce();

}



void
DetectTable::findMinimumAreaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double padding_offset,
                     std::vector <Eigen::Vector3f> &table_top_bbx, std::vector <Eigen::Vector3f> &padded_table_top_bbx, double &rotation_radian,
                     bool &isRectangle,Eigen::Vector3f &circle_center, float &radius) {
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
    std::vector<cv::Point2f> hull_points;
    std::vector<cv::Point2f> contour;  // Convex hull contour points
    cv::convexHull(points,hull_points,false);  //https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga014b28e56cb8854c0de4a211cb2be656
    // Approximating polygonal curve to convex hull
    cv::approxPolyDP(hull_points, contour, 0.1, true);   //https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
    double contourArea = cv::contourArea(cv::Mat(contour));
    double contourPerimeter = cv::arcLength(cv::Mat(contour),true);

    cout<<"Contour Area " << contourArea << " \t Contour Perimeter "<< contourPerimeter<<endl;
    double circularity = (4.0*M_PI*contourArea)/(contourPerimeter*contourPerimeter);
    cout<<"Contour Area " << contourArea << " \t Contour Perimeter "<< contourPerimeter<< " \t Circularity : " << circularity << endl;

    // find the rectangle
    cv::Mat points_mat(points);
    cv::RotatedRect rrect = cv::minAreaRect(points_mat);
    //find the circle
    cv::Point2f circle_center_2d;
    float radius_2d;
    cv::minEnclosingCircle(points,circle_center_2d,radius_2d);

    //check for minimum area between two shapes
    if(circularity <= 0.8 || circularity >= 1.2 )
    {
        isRectangle = true;
        cout<<"Rectangular Table"<<endl;
        //create the padded rectangle
        cv::Size2f padded_size(rrect.size.width + padding_offset, rrect.size.height + padding_offset);
        cv::RotatedRect paddedRect(rrect.center, padded_size, rrect.angle);

        cv::Point2f vertices[4], padded_vertices[4];
        rrect.points(vertices); // this is original reactangle
        paddedRect.points(padded_vertices); // this is padded rectangle

        //store the table top bounding points in a vector
        for (unsigned int ii = 0; ii < 4; ii++) {
            Eigen::Vector3f bbx(vertices[ii].x, vertices[ii].y, height);
            Eigen::Vector3f pbbx(padded_vertices[ii].x, padded_vertices[ii].y, height);
            table_top_bbx.push_back(bbx);
            padded_table_top_bbx.push_back(pbbx);
        }
        // add the centers in the last
        Eigen::Vector3f center(rrect.center.x, rrect.center.y, height);
        table_top_bbx.push_back(center);
        padded_table_top_bbx.push_back(center);
        std::cout << "Found bounding rectangle with 2d angle " << rrect.angle
                  << endl;  // This angle is wrt to the center of the rectangle
        rotation_radian = rrect.angle * PI / 180.0;

    }
    else
    {
        ROS_WARN("CIRCULAR TABLE");
        // in this case just return the circle center, we'll do the padding when we generate the points
        circle_center[0] = circle_center_2d.x;
        circle_center[1] = circle_center_2d.y;
        circle_center[2] = height;
        radius = radius_2d;
    }
}

std::vector <std::vector<Eigen::Vector3f>>
DetectTable::findPossiblePlacementsRectangle(std::vector <Eigen::Vector3f> padded_table_bbox,
                                    double location_sampling_distance) {
    int total_collsion_points = 0;
     std::vector <std::vector<Eigen::Vector3f>> points_on_rectangle_collsion;
    // remove the center since we don't need it
    padded_table_bbox.pop_back();
    // add the first point back again so that we have a complete loop
    padded_table_bbox.push_back(padded_table_bbox[0]);
    for (int i = 0; i < padded_table_bbox.size() - 1; i++)  // last is repeated loop point so dont include it
    {
        std::vector <Eigen::Vector3f> points_on_line;
        float ratio = location_sampling_distance / sqrt((padded_table_bbox[i] - padded_table_bbox[i + 1]).dot(
                padded_table_bbox[i] - padded_table_bbox[i + 1]));
        float proportion = ratio;
        while (proportion < 1.0) {
            Eigen::Vector3f point =
                    padded_table_bbox[i] + (padded_table_bbox[i + 1] - padded_table_bbox[i]) * proportion;
            points_on_line.push_back(point);
            proportion += ratio;
        }
        total_collsion_points += points_on_line.size();
        points_on_rectangle_collsion.push_back(points_on_line);
    }

        std::cout << "Found " << total_collsion_points << " collision points along "
                  << points_on_rectangle_collsion.size() << " edges" << endl;
    return points_on_rectangle_collsion;
}

std::vector <std::vector<Eigen::Vector3f>>
DetectTable::findPossiblePlacementsCircle(Eigen::Vector3f circle_center, float radius,
                             double location_sampling_distance)
{
    std::vector <std::vector<Eigen::Vector3f>> points_on_circle_collsion;
    //calculate the circumferece of the circle
    double perimeter = 2.0*M_PI*radius;
    double angle_offset = (360.0/perimeter)*location_sampling_distance;
    cout<<angle_offset<<endl;
    std::vector<Eigen::Vector3f> points_collision;
    for(int angle = 0;angle<=360;angle =angle+angle_offset)
    {
        double x_new = circle_center[0] + radius * cos(angle * PI / 180);
        double y_new = circle_center[1] + radius * sin(angle * PI / 180);
        Eigen::Vector3f point(x_new,y_new,circle_center[2]);
        points_collision.push_back(point);
    }
    points_on_circle_collsion.push_back(points_collision);
    return points_on_circle_collsion;

}

std::vector<std::vector<Eigen::Vector3f>> DetectTable::filterForFOV(std::vector <std::vector<Eigen::Vector3f>> locations)
{
    std::vector<std::vector<Eigen::Vector3f>> locations_FOV_filtered;
    //find the instantaeous transform between the "odom/world" and the camera_depth_optical_frame
    // trsnformations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    try {
        transformStamped = tfBuffer.lookupTransform( "camera_depth_optical_frame", "odom",
                                                    ros::Time(0),
                                                    ros::Duration(3.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform camera_depth_optical_frame to odom: %s", ex.what());
    }

    image_geometry::PinholeCameraModel cam_model; // init cam_model
    sensor_msgs::CameraInfo cam_info;
    cam_info.height =  480, cam_info.width = 640;
    cam_info.distortion_model =  "plumb_bob";
    cam_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info.K = {554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0};
    cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cam_info.P = {554.254691191187, 0.0, 320.5, -0.0, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};
    cam_model.fromCameraInfo(cam_info); // fill cam_model with CameraInfo

    int total = 0;
    for(int i=0;i<locations.size();i++)
    {
        std::vector<Eigen::Vector3f> filter_FOV;
        // do this for each edge
        for(int j=0;j<locations[i].size();j++)
        {
            geometry_msgs::PointStamped transformed_pt,initial_pt;
            initial_pt.point.x = locations[i][j][0],initial_pt.point.y = locations[i][j][1],initial_pt.point.z = locations[i][j][2];
            //tf2_geometry_msgs::do_transform(initial_pt, transformed_pt, transformStamped);
            tf2::doTransform(initial_pt, transformed_pt, transformStamped);
            // projection_matrix is the matrix you should use if you don't want to use project3dToPixel() and want to use opencv API
            cv::Matx34d projection_matrix=cam_model.fullProjectionMatrix();
            cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(transformed_pt.point.x,transformed_pt.point.y, transformed_pt.point.z)); // project 3d point to 2d point
            if(uv.x >= 0 && uv.x <= cam_info.width && uv.y >= 0 && uv.y <= cam_info.height )
                filter_FOV.push_back(locations[i][j]);
        }
        locations_FOV_filtered.push_back(filter_FOV);
        total+= filter_FOV.size();
    }
    cout<<"FOV filtered locations are : " <<total<<endl;
    return locations_FOV_filtered;
}

// This can be clearly optimized if needed
std::vector <std::vector<Eigen::Vector3f>>
DetectTable::filterForCollision(std::vector <std::vector<Eigen::Vector3f>> &locations,
                                std::vector<double> &wheelchair_dimensions) {
    std::vector <std::vector<Eigen::Vector3f>> desirable_locations;
    double wheelchair_width = wheelchair_dimensions[0], wheelchair_length = wheelchair_dimensions[1], wheelchair_depth = wheelchair_dimensions[2];
    double wheelchair_rotation_radian = wheelchair_dimensions[3];
    Eigen::Quaternionf wheelchair_rotation;
    wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian,
                                            Eigen::Vector3f::UnitZ());  // rotation along z-axis only

    int total_points = 0;
    for (int ii = 0; ii < locations.size(); ii++) {
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
DetectTable::calculateWeights(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,
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

        cout<<"Cluster Size : " << weight_in_cluster.size() << " ( L,C,R ) = " << partitions[0] << " , "<< (weight_in_cluster.size() - partitions[1] - partitions[0]) << " , "<< partitions[1] <<endl;
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
DetectTable::makeClusters(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
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
        ec.setMinClusterSize(MIN_CLUSTER_SIZE); // at min 3 point in this cluster
        ec.setMaxClusterSize(cloud->size());   // at max all the points in the same cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        cout << "Edge : " << i << " Points : " << desirable_locations[i].size() <<" Clusters : " << cluster_indices.size() << " - ";
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
            cout<<" " << cluster.size()<<" , ";
            //std::cout << "PointCloud representing the Cluster: " << cluster.size() << " data points." << std::endl;
        }
        std::cout<<endl;
    }
    cout << "Total Number of Clusters formed are : " << clusters.size() << endl;
    return clusters;
}


/*
 * Publish locations for cost map..
 */
void DetectTable::publishLocations(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
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
    pub_.publish(all_locations);
}
//void DetectTable::publishDesirableLocations(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,double rotation_radian)
//{
//    for(int i=0;i<desirable_locations.size();i++)
//    {
//        for(int j=0;j<desirable_locations[i].size();j++)
//        {
//            // Create pose
//            Eigen::Isometry3d pose;
//            pose = Eigen::AngleAxisd(rotation_radian, Eigen::Vector3d::UnitZ()); // rotate along X axis by 45 degrees
//            pose.translation() = Eigen::Vector3d(desirable_locations[i][j][0], desirable_locations[i][j][1], desirable_locations[i][j][2] ); // translate x,y,z
//            visual_tools_->publishWireframeCuboid(pose, WHEELCHAIR_LENGTH, WHEELCHAIR_WIDTH, WHEELCHAIR_HEIGHT, rviz_visual_tools::RAND);
//            visual_tools_->trigger();
//        }
//    }
//
//}

void callback(const desirable_locations::detectionArray::ConstPtr &message) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud = message->scene_pc;
    pcl::fromROSMsg(cloud, *scene_cloud);  // assgin the scene cloud
    cout << "Scene Cloud : width " << scene_cloud->width << " Height : " << scene_cloud->height << endl;

    // Create the filtering object
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

    // for each input pcd detect tables
    DetectTable detectTable(scene_voxelised, message->detections);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "find_location");
    ros::NodeHandle n_pub,nh_;
    //Topic to publish
    pub_ = n_pub.advertise<desirable_locations::locationArray>("/pcl_processing/desirable_locations", 1);
    pub_table = nh_.advertise<sensor_msgs::PointCloud2>("/table_top", 1);
//#ifdef VISUALISATIONS
//    // This viewer has 4 windows, but is only showing images in one of them as written here.
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//#endif
    //subscribe to the votenet detection topic
    ros::NodeHandle nh;
    // create the subscriber
    ros::Subscriber sub = nh.subscribe("/votenet/detections", 1, callback);
    // Run the subscriber until someone stops the program with Ctrl + C
    ros::spin();

    return 0;

}