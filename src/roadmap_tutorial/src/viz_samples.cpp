#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

void  SampleDenseMarkers(visualization_msgs::Marker& markers,
                         visualization_msgs::MarkerArray& edge_markers){
  markers.header.frame_id = "/base_link";
  markers.header.stamp = ros::Time::now();
  markers.action = visualization_msgs::Marker::ADD;
  markers.pose.orientation.w = 1.0;
  markers.id = 0;
  markers.type = visualization_msgs::Marker::SPHERE_LIST;
  markers.scale.x = 0.008;
  markers.scale.y = 0.008;
  markers.scale.z = 0.008;
  markers.color.r = 1.0f;
  markers.color.g = 1.0f;
  markers.color.b = 0.1f;
  markers.color.a = 1.0;
  markers.lifetime = ros::Duration();

  visualization_msgs::Marker edge_marker;
  edge_marker.header.frame_id = "/base_link";
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
  edge_marker.scale.x = 0.0003;
  edge_marker.scale.y = 0.0003;
  edge_marker.scale.z = 0.0003;
  edge_marker.color.r = 1.0f;
  edge_marker.color.g = 0.0f;
  edge_marker.color.b = 0.0f;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration();

  size_t edge_count = 0;
  //Get vertex coordinates
  for(size_t i = 0; i < 26; i++){
    for(size_t j = 0; j < 41; j++){
      for(size_t k = 0; k < 6; k++){
        geometry_msgs::Point point;
        point.x = 0.2 + i * 0.02;
        point.y = -0.4 + j * 0.02;
        point.z = 0.2 + k * 0.03 - 0.147 - 0.014;
        markers.points.push_back(point);

        geometry_msgs::Point sparse_point;
        sparse_point.x = 0.2 + round((point.x - 0.2) / 0.1) * 0.1;
        sparse_point.y = -0.4 + round((point.y - -0.4) / 0.1) * 0.1;
        sparse_point.z = 0.4 + 2 * 0.05 - 0.147 - 0.014;
        edge_marker.id = edge_count; edge_count++;
        edge_marker.points.clear();
        edge_marker.points.push_back(point);
        edge_marker.points.push_back(sparse_point);
        edge_markers.markers.push_back(edge_marker);
        edge_marker.id = edge_count; edge_count++;
        edge_marker.points[1].z = 0.4 + 1 * 0.05 - 0.147 - 0.014;
        edge_markers.markers.push_back(edge_marker);
        edge_marker.id = edge_count; edge_count++;
        edge_marker.points[1].z = 0.4 + 0 * 0.05 - 0.147 - 0.014;
        edge_markers.markers.push_back(edge_marker);


        if(i != 0){
          geometry_msgs::Point prev_point;
          prev_point.x = 0.2 + (i - 1) * 0.02;
          prev_point.y = -0.4 + (j) * 0.02;
          prev_point.z = 0.2 + (k) * 0.03 - 0.147 - 0.014;
          edge_marker.id = edge_count; edge_count++;
          edge_marker.points.clear();
          edge_marker.points.push_back(point);
          edge_marker.points.push_back(prev_point);
          edge_markers.markers.push_back(edge_marker);
        }
        if(j != 0){
          geometry_msgs::Point prev_point;
          prev_point.x = 0.2 + (i) * 0.02;
          prev_point.y = -0.4 + (j-1) * 0.02;
          prev_point.z = 0.2 + (k) * 0.03 - 0.147 - 0.014;
          edge_marker.id = edge_count; edge_count++;
          edge_marker.points.clear();
          edge_marker.points.push_back(point);
          edge_marker.points.push_back(prev_point);
          edge_markers.markers.push_back(edge_marker);
        }
        if(k != 0){
          geometry_msgs::Point prev_point;
          prev_point.x = 0.2 + (i) * 0.02;
          prev_point.y = -0.4 + (j) * 0.02;
          prev_point.z = 0.2 + (k-1) * 0.03 - 0.147 - 0.014;
          edge_marker.id = edge_count; edge_count++;
          edge_marker.points.clear();
          edge_marker.points.push_back(point);
          edge_marker.points.push_back(prev_point);
          edge_markers.markers.push_back(edge_marker);
        }
      }
    }
  }
}

void  SampleSparseMarkers(visualization_msgs::Marker& markers,
                          visualization_msgs::MarkerArray& edge_markers){
  markers.header.frame_id = "/base_link";
  markers.header.stamp = ros::Time::now();
  markers.action = visualization_msgs::Marker::ADD;
  markers.pose.orientation.w = 1.0;
  markers.id = 0;
  markers.type = visualization_msgs::Marker::SPHERE_LIST;
  markers.scale.x = 0.01;
  markers.scale.y = 0.01;
  markers.scale.z = 0.01;
  markers.color.r = 1.0f;
  markers.color.g = 0.5f;
  markers.color.b = 0.0f;
  markers.color.a = 1.0;
  markers.lifetime = ros::Duration();
  size_t edge_count = 0;
  //Get vertex coordinates
  for(size_t i = 0; i < 6; i++){
    for(size_t j = 0; j < 9; j++){
      for(size_t k = 0; k < 3; k++){
        geometry_msgs::Point point;
        point.x = 0.2 + i * 0.1;
        point.y = -0.4 + j * 0.1;
        point.z = 0.4 + k * 0.05 - 0.147 - 0.014;
        markers.points.push_back(point);
        if(i != 0){
          geometry_msgs::Point prev_point;
          prev_point.x = 0.2 + (i - 1) * 0.1;
          prev_point.y = -0.4 + (j) * 0.1;
          prev_point.z = 0.4 + (k) * 0.05 - 0.147 - 0.014;
          visualization_msgs::Marker edge_marker;
          edge_marker.header.frame_id = "/base_link";
          edge_marker.header.stamp = ros::Time::now();
          edge_marker.action = visualization_msgs::Marker::ADD;
          edge_marker.pose.orientation.w = 1.0;
          edge_marker.id = edge_count; edge_count++;
          edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
          edge_marker.scale.x = 0.001;
          edge_marker.scale.y = 0.001;
          edge_marker.scale.z = 0.001;
          edge_marker.color.r = 1.0f;
          edge_marker.color.g = 0.0f;
          edge_marker.color.b = 0.0f;
          edge_marker.color.a = 1.0;
          edge_marker.lifetime = ros::Duration();
          edge_marker.points.push_back(point);
          edge_marker.points.push_back(prev_point);
          edge_markers.markers.push_back(edge_marker);
        }
        if(j != 0){
          geometry_msgs::Point prev_point;
          prev_point.x = 0.2 + (i) * 0.1;
          prev_point.y = -0.4 + (j-1) * 0.1;
          prev_point.z = 0.4 + (k) * 0.05 - 0.147 - 0.014;
          visualization_msgs::Marker edge_marker;
          edge_marker.header.frame_id = "/base_link";
          edge_marker.header.stamp = ros::Time::now();
          edge_marker.action = visualization_msgs::Marker::ADD;
          edge_marker.pose.orientation.w = 1.0;
          edge_marker.id = edge_count; edge_count++;
          edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
          edge_marker.scale.x = 0.001;
          edge_marker.scale.y = 0.001;
          edge_marker.scale.z = 0.001;
          edge_marker.color.r = 1.0f;
          edge_marker.color.g = 0.0f;
          edge_marker.color.b = 0.0f;
          edge_marker.color.a = 1.0;
          edge_marker.lifetime = ros::Duration();
          edge_marker.points.push_back(point);
          edge_marker.points.push_back(prev_point);
          edge_markers.markers.push_back(edge_marker);
        }
        if(k != 0){
          geometry_msgs::Point prev_point;
          prev_point.x = 0.2 + (i) * 0.1;
          prev_point.y = -0.4 + (j) * 0.1;
          prev_point.z = 0.4 + (k-1) * 0.05 - 0.147 - 0.014;
          visualization_msgs::Marker edge_marker;
          edge_marker.header.frame_id = "/base_link";
          edge_marker.header.stamp = ros::Time::now();
          edge_marker.action = visualization_msgs::Marker::ADD;
          edge_marker.pose.orientation.w = 1.0;
          edge_marker.id = edge_count; edge_count++;
          edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
          edge_marker.scale.x = 0.001;
          edge_marker.scale.y = 0.001;
          edge_marker.scale.z = 0.001;
          edge_marker.color.r = 1.0f;
          edge_marker.color.g = 0.0f;
          edge_marker.color.b = 0.0f;
          edge_marker.color.a = 1.0;
          edge_marker.lifetime = ros::Duration();
          edge_marker.points.push_back(point);
          edge_marker.points.push_back(prev_point);
          edge_markers.markers.push_back(edge_marker);
        }
      }
    }
  }
}

void  SampleHomeMarkers(visualization_msgs::Marker& markers,
                        visualization_msgs::MarkerArray& edge_markers){
  markers.header.frame_id = "/base_link";
  markers.header.stamp = ros::Time::now();
  markers.action = visualization_msgs::Marker::ADD;
  markers.pose.orientation.w = 1.0;
  markers.id = 0;
  markers.type = visualization_msgs::Marker::SPHERE_LIST;
  markers.scale.x = 0.03;
  markers.scale.y = 0.03;
  markers.scale.z = 0.03;
  markers.color.r = 1.0f;
  markers.color.g = 0.8f;
  markers.color.b = 0.5f;
  markers.color.a = 1.0;
  markers.lifetime = ros::Duration();
  geometry_msgs::Point home_1, home_2;
  home_1.x = 0.3;
  home_1.y = 0;
  home_1.z = 0.55 - 0.147 - 0.014;
  home_2.x = 0.45;
  home_2.y = 0;
  home_2.z = 0.55 - 0.147 - 0.014;
  markers.points.push_back(home_1);
  markers.points.push_back(home_2);

  visualization_msgs::Marker edge_marker;
  edge_marker.header.frame_id = "/base_link";
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
  edge_marker.scale.x = 0.001;
  edge_marker.scale.y = 0.001;
  edge_marker.scale.z = 0.001;
  edge_marker.color.r = 0.0f;
  edge_marker.color.g = 1.0f;
  edge_marker.color.b = 0.0f;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration();



  size_t edge_count = 0;
  //Get vertex coordinates
  for(size_t i = 0; i < 6; i++){
    for(size_t j = 0; j < 9; j++){
      for(size_t k = 0; k < 3; k++){
        geometry_msgs::Point point;
        point.x = 0.2 + i * 0.1;
        point.y = -0.4 + j * 0.1;
        point.z = 0.4 + k * 0.05 - 0.147 - 0.014;

        edge_marker.id = edge_count; edge_count++;
        edge_marker.points.clear();
        edge_marker.points.push_back(point);
        edge_marker.points.push_back(home_1);
        edge_markers.markers.push_back(edge_marker);

        edge_marker.id = edge_count; edge_count++;
        edge_marker.points.clear();
        edge_marker.points.push_back(point);
        edge_marker.points.push_back(home_2);
        edge_markers.markers.push_back(edge_marker);
      }
    }
  }
}






int main(int argc, char** argv){

  ros::init(argc, argv, "samples_viewer");
  ros::NodeHandle n;
  ros::Rate rate(1);


  ros::Publisher dense_pub = n.advertise<visualization_msgs::Marker>("/dense_markers", 5);
  ros::Publisher dense_edge_pub = n.advertise<visualization_msgs::MarkerArray>("/dense_edges", 5);
  ros::Publisher sparse_pub = n.advertise<visualization_msgs::Marker>("/sparse_markers", 5);
  ros::Publisher sparse_edge_pub = n.advertise<visualization_msgs::MarkerArray>("/sparse_edges", 5);
  ros::Publisher home_pub = n.advertise<visualization_msgs::Marker>("/home_markers", 5);
  ros::Publisher home_edge_pub = n.advertise<visualization_msgs::MarkerArray>("/home_edges", 5);
  visualization_msgs::Marker dense_markers;
  visualization_msgs::MarkerArray dense_edge_markers;
  visualization_msgs::MarkerArray sparse_edge_markers;
  visualization_msgs::Marker sparse_markers;
  visualization_msgs::MarkerArray home_edge_markers;
  visualization_msgs::Marker home_markers;
  SampleDenseMarkers(dense_markers, dense_edge_markers);
  std::cout << dense_edge_markers.markers.size() << " dense edges drawn" << std::endl;

  SampleSparseMarkers(sparse_markers, sparse_edge_markers);
  SampleHomeMarkers(home_markers, home_edge_markers);

  std::cout << sparse_edge_markers.markers.size() << " sparse edges drawn" << std::endl;
  std::cout << home_edge_markers.markers.size() << " home edges drawn" << std::endl;
  
  while (ros::ok()) {
    dense_pub.publish(dense_markers);
    dense_edge_pub.publish(dense_edge_markers);
    sparse_pub.publish(sparse_markers);
    sparse_edge_pub.publish(sparse_edge_markers);
    
    home_pub.publish(home_markers);
    home_edge_pub.publish(home_edge_markers);
    ros::spinOnce();
    rate.sleep();
  }
}
