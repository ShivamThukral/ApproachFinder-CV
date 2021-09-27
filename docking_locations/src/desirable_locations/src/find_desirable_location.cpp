//
// Created by vcr on 2021-03-16.
//

#include "desirable_locations/find_desirable_location.h"

/*
 * Publish locations for cost map..
 * Status: Finalised
 */
void DesirableLocations::publishLocations(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
                                         std::vector <std::vector<double>> &weights) {
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
            desirable_locations::desiredLocation table_location;
            //assign the parking spots
            table_location.location.x = desirable_locations[i][j][0];
            table_location.location.y = desirable_locations[i][j][1];
            table_location.location.z = desirable_locations[i][j][2];
            //asign the weight
            table_location.location_weight = weights[i][j];
            all_locations.desired_locations.push_back(table_location);
        }
    }
    //publish the locations for cost map calculations...
    pub_approx.publish(all_locations);
}

std::vector <std::vector<Eigen::Vector3f>>
DesirableLocations::makeClusters(std::vector <std::vector<Eigen::Vector3f>> &desirable_locations,
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

std::vector <std::vector<double>>
DesirableLocations::calculateVisibilityWeights(std::vector <std::vector<Eigen::Vector3f>> locations,
                                              std::vector<double> wheelchair_dimensions) {
    //returned value
    std::vector <std::vector<double>> visibility_weights;
    //camera parameters
    float centre_x = cam_info.K[2];
    float centre_y = cam_info.K[5];
    float focal_x = cam_info.K[0];
    float focal_y = cam_info.K[4];

    double wheelchair_width = wheelchair_dimensions[0], wheelchair_length = wheelchair_dimensions[1], wheelchair_depth = wheelchair_dimensions[2];
    //find the instantaeous transform between the "odom/world" and the camera_depth_optical_frame
    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());

    //for each location
    for (int ii = 0; ii < locations.size(); ii++) {
        double wheelchair_rotation_radian = 0;
        Eigen::Quaternionf wheelchair_rotation;
        //the wheelchair rotation angle along each edge
        if (locations[ii].size() >= 2) {
            Eigen::Vector3f point1 = locations[ii][0], point2 = locations[ii][locations[ii].size() -1]; // first and last point
            wheelchair_rotation_radian = atan2((point2[1] - point1[1]), (point2[0] - point1[0]));
        }
        wheelchair_rotation = Eigen::AngleAxisf(wheelchair_rotation_radian, Eigen::Vector3f::UnitZ());  // rotation along z-axis only

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

            vtkSmartPointer <vtkDataSet> data = pcl::visualization::createCube(locations[ii][j], wheelchair_rotation,
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

std::vector <std::vector<double>>
DesirableLocations::findPositionalWeights(std::vector <std::vector<Eigen::Vector3f>> desirable_locations,
                                         std::vector <std::vector<Eigen::Vector3f>> &clusters) {
    /*
    * Points which are close to corners and other chairs should be weighted less
    */
    std::vector <std::vector<double>> weights; // final value to be returned
    std::vector<int> along_edge;               // tells about the edge of the point
    clusters = makeClusters(desirable_locations, along_edge);     // clusters formed by the points
    for (std::vector <Eigen::Vector3f> cluster:clusters) {
        //initially assign weight 1 to every point in the cluster
        std::vector<double> weight_in_cluster(cluster.size(), 1);

        //the sampling is such that the two extremes are our max values
        Eigen::Vector3f left_extreme = cluster[0], right_extreme = cluster[cluster.size() - 1];

        //for each point in cluster check if they are close to corner o chair
        for (int i = 0; i < cluster.size(); i++) {
            Eigen::Vector3f point = cluster[i];
            double distance_left = sqrt((left_extreme - point).dot(left_extreme - point)); // distance from left
            double distance_right = sqrt((right_extreme - point).dot(right_extreme - point)); // distance from right
/*
            //point is clear from both the sides
            if(distance_left >= CLUSTER_TOLERANCE && distance_right <= CLUSTER_TOLERANCE)
                continue;
            double W = 0.0;

            //add penality-left side
            if(distance_left < distance_right)
                W += (distance_left / CLUSTER_TOLERANCE);
            else
                W += (distance_right / CLUSTER_TOLERANCE);

            weight_in_cluster[i] = W;
            cout<<weight_in_cluster[i]<<",";
            */
        }
        weights.push_back(weight_in_cluster);
        assert(cluster.size() == weight_in_cluster.size());
    }
    std::cout << "Weight calculated for each point" << std::endl;
    cout << "--------------------------------------------------------------------------" << endl;
    return weights;
}


geometry_msgs::TransformStamped DesirableLocations::listenTransform(std::string des_frame, std::string src_frame) {
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

std::vector <std::vector<Eigen::Vector3f>>
DesirableLocations::filterForCollision(std::vector <std::vector<Eigen::Vector3f>> &locations,
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

        }
        total_points += filtered_points.size();
        desirable_locations.push_back(filtered_points);
    }
    std::cout << "Found " << total_points << " non colliding  points along " << desirable_locations.size()
              << " edges" << endl;
    return desirable_locations;
}

std::vector <std::vector<Eigen::Vector3f>>
DesirableLocations::filterForFOV(std::vector <std::vector<Eigen::Vector3f>> &locations) {
    std::vector <std::vector<Eigen::Vector3f>> locations_FOV_filtered;
    //find the instantaeous transform between the "odom/world" and the camera_depth_optical_frame
    geometry_msgs::TransformStamped transformStamped = listenTransform(des_frame.c_str(), src_frame.c_str());

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
        locations_FOV_filtered.push_back(filter_FOV);
        total += filter_FOV.size();
    }
    cout << "FOV filtered locations are : " << total << endl;
    return locations_FOV_filtered;
}

std::vector<Eigen::Vector3f> DesirableLocations::addInbetweenPaddedLines(std::vector<Eigen::Vector3f> padded_approx_polygon)
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

std::vector <Eigen::Vector3f>
DesirableLocations::calculatePaddedLines(std::vector <Eigen::Vector3f> &approx_polygon, double padding_offset) {
    std::vector <Eigen::Vector3f> padded_edges;
    std::vector <cv::Point2f> contour;
    for (Eigen::Vector3f point:approx_polygon)
        contour.push_back(cv::Point2f(point[0], point[1]));
    contour.pop_back();

    for (int i = 0; i < approx_polygon.size() - 1; i++) {
        cv::Point2f p1 = cv::Point2f(approx_polygon[i][0], approx_polygon[i][1]); // "start"  // z point is not required for our case
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

std::vector <Eigen::Vector3f>
DesirableLocations::findMinimumAreaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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
DesirableLocations::findPossiblePlacements(std::vector <Eigen::Vector3f> &approx_polygon,
                                          double padding_offset, double location_sampling_distance) {
    int total_collsion_points = 0;
    std::vector <std::vector<Eigen::Vector3f>> collsion_points;
    // add the first point back again so that we have a complete loop
    approx_polygon.push_back(approx_polygon[0]);

    std::vector <Eigen::Vector3f> padded_approx_polygon = calculatePaddedLines(approx_polygon, padding_offset);

    // join the edges inbetween as well.
    std::vector<Eigen::Vector3f> padded_polygon = addInbetweenPaddedLines(padded_approx_polygon);
    padded_approx_polygon = padded_polygon;

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


pcl::PointCloud<pcl::PointXYZ>::Ptr DesirableLocations::findTableTop(pcl::ModelCoefficients::Ptr plane_coefficients,
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
    reg.setNumberOfNeighbours(STATISTICAL_OUTLIER_NEIGHBOURS);
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


void
DesirableLocations::fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr table_bbox_cloud,
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

//TODO: need to optimise for both tables and toilets
void
DesirableLocations::fitModelToilets(pcl::PointCloud<pcl::PointXYZ>::Ptr toilet_pcd,
                                  pcl::ModelCoefficients::Ptr coefficients) {

    //apply pass through filter with height constraints
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (toilet_pcd);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.20, 0.55);
    pass.filter (*cloud_filtered_z);

/*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (table_bbox_cloud);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_NORMAL_SPHERE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setNormalDistanceWeight ( 0.1 );
    seg.setMaxIterations ( 10000 );
    seg.setDistanceThreshold ( 0.2 );
    seg.setRadiusLimits ( 0, 0.4 );
    seg.setInputCloud ( table_bbox_cloud );
    seg.setInputNormals ( cloud_normals );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    seg.segment ( *inliers, *coefficients );
    std::cerr << "Cylinder coefficients: " << *coefficients << std::endl;
*/

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation <pcl::PointXYZ> seg;    // Create the segmentation object
    seg.setOptimizeCoefficients(true);     // Optional
    seg.setModelType(pcl::SACMODEL_PLANE);  // Fitting a plane on this point cloud
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    //seg.setRadiusLimits (0, 0.4);
    seg.setDistanceThreshold(PLANE_DISTANCE_THRESHOLD);     // Minimum distance to be considered
    seg.setInputCloud(cloud_filtered_z);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }
    std::cout << "coefficients: " << *coefficients << std::endl;
//    equation of the plane
//    std::cout << "Model coefficients (a,b,c,d): " << coefficients->values[0] << " "
//              << coefficients->values[1] << " "
//              << coefficients->values[2] << " "
//              << coefficients->values[3] << std::endl;
    std::cout << "Number of inliers " << inliers->indices.size() << " points." << std::endl;

/*    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted (new pcl::PointCloud<pcl::PointXYZ>);
    // Extract the inliers
    extract.setInputCloud (cloud_filtered_z);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_extracted);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_extracted, *new_cloud);
    for(auto &p: new_cloud->points) p.r=255;
    extract.setNegative (true);
    extract.filter (*cloud_extracted);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_extracted, *final_cloud);
    for(auto &p: final_cloud->points) p.g=255;

    *new_cloud += *final_cloud;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (255.0, 255.0, 255.0);
    viewer.addPointCloud<pcl::PointXYZRGB>(new_cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //pcl::PointXYZ center(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    //viewer.addSphere(center, coefficients->values[3], "sphere");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        sleep(0.1);
    }*/
}


std::vector <Eigen::Vector3f> DesirableLocations::findToiletPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr box_corners)
{
    std::vector<Eigen::Vector3f> polygon_points;
    //the bbox should have 8 corners
    ASSERT(box_corners->points.size()==8,"BBOX corners should have 8 points");
    for(int i=0;i<4;i++)
        polygon_points.push_back(Eigen::Vector3f(box_corners->points[i].x,box_corners->points[i].y, WHEELCHAIR_DEPTH/2.0 + 0.1));
    return polygon_points;
}

void
DesirableLocations::DFS(std::map<int, std::vector<int>> &intersections, std::set<int> &visited, std::vector<int> &path,
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
DesirableLocations::findConnectedComponents(std::map<int, std::vector<int>> &intersections) {
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

void DesirableLocations::findIntersectingBBoxToilets(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> &extracted_bbox, std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> &merged_bbox, std::string label_name)
{
    std::map<int, cv::RotatedRect> all_rectangles;
    //convert the point cloud in cv::rectangles
    for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = extracted_bbox.begin();it!=extracted_bbox.end();it++) {
        std::vector<cv::Point2f> rect_points;

        //need only three points
        for(int i=0;i<3;i++)
            rect_points.push_back(cv::Point2f(it->second->points[i].x,it->second->points[i].y ));
        //https://github.com/opencv/opencv/blob/master/modules/core/src/types.cpp
        cv::Point2f _center = 0.5f * (rect_points[0] + rect_points[2]);
        cv::Vec2f vecs[2];
        vecs[0] = cv::Vec2f(rect_points[0] - rect_points[1]);
        vecs[1] = cv::Vec2f(rect_points[1] - rect_points[2]);

        // wd_i stores which vector (0,1) or (1,2) will make the width
        // One of them will definitely have slope within -1 to 1
        int wd_i = 0;
        if( std::fabs(vecs[1][1]) < std::fabs(vecs[1][0]) ) wd_i = 1;
        int ht_i = (wd_i + 1) % 2;

        float _angle = std::atan(vecs[wd_i][1] / vecs[wd_i][0]) * 180.0f / (float) CV_PI;
        float _width = (float) norm(vecs[wd_i]);
        float _height = (float) norm(vecs[ht_i]);

        cv::RotatedRect rectangle(_center,cv::Size2f(_width,_height),_angle);
        all_rectangles[it->first] = rectangle;
    }

    // find all the matchings as list indexes
    std::map<int, std::vector<int>> intersections;

    //insert empty lists
    std::vector<int> matches;
    for (std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = extracted_bbox.begin();
         it != extracted_bbox.end(); it++) {
        intersections[it->first] = matches;
    }


    for(std::map<int, cv::RotatedRect>::iterator it = all_rectangles.begin();it!=all_rectangles.end();it++)
    {
        for(std::map<int, cv::RotatedRect>::iterator ptr = it; ptr!=all_rectangles.end();ptr++)
        {
            if (it->first != ptr->first) // dont compare same extracted pcd and skip if already matched
            {
                //if not matched already
                auto pos = std::find(intersections[it->first].begin(), intersections[it->first].end(), ptr->first);
                if (pos == intersections[it->first].end()) {
                    //check overlap between bboxes
                    std::vector<cv::Point2f> intersection_points;
                    if(cv::rotatedRectangleIntersection(it->second, ptr->second, intersection_points) > 0)
                    {
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
        std::string name = label_name + std::to_string(it->first);
        cv::RotatedRect rectangle = all_rectangles[it->first];
        double area = rectangle.size.width * rectangle.size.height;
        int index = it->first;
        for (int i = 0; i < it->second.size(); i++) {
            cv::RotatedRect rect = all_rectangles[it->second[i]];
            if(area < rect.size.height * rect.size.width) {
                area = rect.size.height * rect.size.width;
                rectangle = rect;
                index = it->second[i];
            }
        }
        merged_bbox[name] = extracted_bbox[index];
    }
}

void DesirableLocations::findIntersectingBBoxTables(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> &extracted_pcd, std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> &merged_bbox, std::string label_name)
{
    //std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> merged_bbox;
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
                    if (filtered->size() >= OVERLAPPING_POINTS) {
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
        std::string name = label_name + std::to_string(it->first);
        merged_bbox[name] = extracted_pcd[it->first];
        for (int i = 0; i < it->second.size(); i++) {
            *merged_bbox[name] += *extracted_pcd[it->second.at(i)];
        }
    }
    //return merged_bbox;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DesirableLocations::extractBBoxPCD(std::vector <geometry_msgs::Point> &bbox_corners) {
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

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> DesirableLocations::extract_pointcloud(std::string label_name, std::vector <desirable_locations::votenetDetection> &detections)
{
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_pointclouds;
    for (int i = 0; i < detections.size(); i++) {
        //check for threshold - process only objects which are above the threshold and extract only required semantic classes
        if (detections[i].object_score < OBJECTNESS_THRESHOLD || detections[i].semantic_class.compare(label_name) != 0)
            continue;
        //extract the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = extractBBoxPCD(detections[i].bbox_3d);
        extracted_pointclouds[i] = box_pcd;
    }
    return extracted_pointclouds;
}

std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> DesirableLocations::extract_toilet_bbox(std::string label_name, std::vector <desirable_locations::votenetDetection> &detections)
{
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_bbox;
    for (int i = 0; i < detections.size(); i++) {
        //check for threshold - process only objects which are above the threshold and extract only required semantic classes
        if (detections[i].object_score < OBJECTNESS_THRESHOLD || detections[i].semantic_class.compare(label_name) != 0)
            continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_corners_pcd(new pcl::PointCloud <pcl::PointXYZ>);
        for (geometry_msgs::Point corner:detections[i].bbox_3d) {
            pcl::PointXYZ point(corner.x, corner.y, corner.z);
            bbox_corners_pcd->push_back(point);
        }
        extracted_bbox[i] = bbox_corners_pcd;
    }
    return extracted_bbox;
}

DesirableLocations::~DesirableLocations() {

}

DesirableLocations::DesirableLocations(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, cv::Mat &d_image,
                                     std::vector <desirable_locations::votenetDetection> detections) {
    //track the time of execution
    ros::Time start = ros::Time::now();
    //assign the scene and depth image to this class
    this->scene_cloud = scene_cloud;
    this->depth_image = d_image;
    this->marker_id = 0;            // unique marker id to each visualisations

    //separate out table and toilets detections
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_tables = extract_pointcloud("table", detections);
    //std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_toilets = extract_pointcloud("chair", detections);
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> extracted_toilets_bbox = extract_toilet_bbox("toilet", detections);


    //finds intersections between the bounding boxes and returns as map
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> merged_object_pcd;
    findIntersectingBBoxTables(extracted_tables, merged_object_pcd,"table");
    findIntersectingBBoxToilets(extracted_toilets_bbox, merged_object_pcd,"toilet");


    //std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> merged_table_pcd = findIntersecingBBox(extracted_tables);
    //std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> merged_toilet_pcd = findIntersecingBBox(extracted_toilets);

    //clear some memory
    extracted_tables.clear();
    extracted_toilets_bbox.clear();

#ifdef VISUALISATIONS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_coloured(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*scene_cloud, *scene_coloured);
#endif

    for (std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = merged_object_pcd.begin();
         it != merged_object_pcd.end(); it++){

        //extracted+combined the pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_pcd = it->second;
        std::vector <Eigen::Vector3f> polygon_points;
        if(it->first.rfind("table",0) == 0)
        {
            // Try to fit a planar model in the plane.
            pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
            // gets the equation of the plane for table top.
            fitPlanarModel(box_pcd, plane_coefficients);   // returns the table_top in terms of plane
            //find the table top based on euclidean segmentation
            pcl::PointCloud<pcl::PointXYZ>::Ptr table_top_pcd = findTableTop(plane_coefficients, box_pcd);
            //find the Minimum area polygon of this table top
            polygon_points = findMinimumAreaShape(table_top_pcd);

#ifdef VISUALISATIONS
            for(int i=0;i<scene_coloured->size();i++) {
            pcl::PointXYZ point = scene_cloud->at(i);
            scene_coloured->points[i].r = 170;
            scene_coloured->points[i].g = 175;
            scene_coloured->points[i].b = 175;
            float plane_value = plane_coefficients->values[0]*point.x + plane_coefficients->values[1]*point.y + plane_coefficients->values[2]*point.z + plane_coefficients->values[3];

            if(abs(plane_value) <= PLANE_DISTANCE_THRESHOLD)
            {
                scene_coloured->points[i].g = 125;
            }
        }
        // paint the table top separately
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_top_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*table_top_pcd, *table_top_colored);
        *scene_coloured += *table_top_colored;
#endif

        }
        else if(it->first.rfind("toilet",0) == 0)
        {
            polygon_points = findToiletPolygon(box_pcd);
            // gets the equation of the plane for table top.
            //fitModelToilets(box_pcd, plane_coefficients);   // returns the table_top in terms of plane
        }


        //find possible placement locations
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
                    viewer_approx->addSphere(sphere_center,0.04,"bbox"+std::to_string(marker_id++));
                }
            }
#endif

        std::vector <std::vector<Eigen::Vector3f>> locations_filtered = filterForFOV(locations);
        std::vector<double> wheelchairDimensions = {WHEELCHAIR_WIDTH, WHEELCHAIR_LENGTH, WHEELCHAIR_DEPTH};
        std::vector <std::vector<Eigen::Vector3f>> desirable_locations = filterForCollision(locations_filtered,
                                                                                            wheelchairDimensions);

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
                   // viewer_approx->addSphere(sphere_center,0.04,"bbox"+std::to_string(marker_id++));
                 //   viewer_approx->addCube(desirable_locations[i][j],wheelchair_rotation,0.04,0.04,0.04,"cube"+std::to_string(this->marker_id++));
                }
            }
#endif
        std::vector <std::vector<Eigen::Vector3f>> clusters;
        std::vector <std::vector<double>> position_weights = findPositionalWeights(desirable_locations, clusters);
        std::vector <std::vector<double>> visibility_weights = calculateVisibilityWeights(clusters, wheelchairDimensions);

        //append these results into main results
        for (std::vector <Eigen::Vector3f> cluster:clusters)
            all_clusters.push_back(cluster);

        for (int i = 0; i < position_weights.size(); i++) {
            ASSERT(position_weights[i].size() == visibility_weights[i].size(),
                   "Mismatch Visibility and Position Weights size");
            for(int j = 0;j<position_weights[i].size();j++)
            {
                position_weights[i][j] *= visibility_weights[i][j];
            }
            all_weights.push_back(position_weights[i]);
        }


        //simpleVis(it->second, "toilet");

    }

#ifdef VISUALISATIONS
    viewer_approx->setBackgroundColor(255, 255, 255);
    viewer_approx->addPointCloud<pcl::PointXYZRGB> (scene_coloured, "3D Viewer");
    viewer_approx->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
    viewer_approx->addCoordinateSystem (1.0);
    viewer_approx->initCameraParameters ();
    viewer_approx->removeCoordinateSystem();
    while (!viewer_approx->wasStopped()) {
        viewer_approx->spinOnce(100);
        sleep(0.1);
    }
   // exit(0);
#endif


    //publish the desirable locations and weights for cost map
    publishLocations(all_clusters, all_weights);
    ros::Time end = ros::Time::now();
    cout << "Total Time : " << (end - start).toSec() << endl;

}



//reassign the camera-info subscribed from the topic
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info_msg) {
    cam_info = *info_msg;
}

void callbackDesirableLocations(const desirable_locations::detectionArray::ConstPtr &message) {

    //convert depth image into cv image
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message->depth_image, message);
    cv::Mat depth_image;
    cv_ptr->image.convertTo(depth_image, CV_32F);
    //convert point cloud to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud = message->scene_pc;
    pcl::fromROSMsg(cloud, *scene_cloud);  // assgin the scene cloud

    cout << "Recieved Scene Cloud : width " << scene_cloud->width << " Height : " << scene_cloud->height << endl;

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

    //convert back to pcl::PointCloudXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_voxelised(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*scene_voxelised_pointcloud2, *scene_voxelised);

    // for each input pcd detect desirable locations
    DesirableLocations desired_locations(scene_voxelised, depth_image, message->detections);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "find_desirable_locations");
    ros::NodeHandle nh;
    //Topic to publish
    pub_approx = nh.advertise<desirable_locations::locationArray>("/pcl_processing/desirable_locations", 1);
    //pub_table_approx = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_processing/filtered_table_top", 1);
    //subscribe to the votenet detection topic
    ros::Subscriber sub = nh.subscribe("/votenet/detections", 1, callbackDesirableLocations);
    //camera info subscriber
    ros::Subscriber sub_cam_info = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);
    // Run the subscriber until someone stops the program with Ctrl + C
    ros::spin();
    return 0;
}
