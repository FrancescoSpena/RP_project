#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>

#include <nav_msgs/OccupancyGrid.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rp_stuff/dmap.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap_localizer.h>


DMap dmap(0,0);
DMapLocalizer localizer;
GridMap grid_map(0.1,1,1);

bool map_received = false;
bool init_pose_received = false; 
bool first_scan = true; 

std::vector<Vector2f> obstacles;
Canvas canvas; 


void mapCallBack(const nav_msgs::OccupancyGrid& map){
    if(map_received == false){
        std::cerr << "Map received" << std::endl;
        //extract info to map_server
        uint32_t width = map.info.width;
        uint32_t height = map.info.height;
        float resolution = map.info.resolution;
        
        //set grid map and localizer
        grid_map.resize(height,width);
        grid_map.cells = map.data;
        localizer.setMap(grid_map,1,100);
        localizer.distances.draw(canvas,true);
        localizer.grid_mapping.reset(Vector2f(0,0),resolution);
        
        map_received = true;
    }
    std::cerr << "Dmap ready" << std::endl;
}

void initCallBack(const geometry_msgs::PoseWithCovarianceStamped& init){
    if(map_received == true && init_pose_received == false){
        std::cerr << "Init pose received" << std::endl;
        
        //coordinates [frame = map]
        //Extract information from initpose topic [rviz]
        Isometry2f X=Eigen::Isometry2f::Identity();
        Eigen::Quaternionf q(init.pose.pose.orientation.w,init.pose.pose.orientation.x,init.pose.pose.orientation.y,init.pose.pose.orientation.z); 
        float x = init.pose.pose.position.x;
        float y = init.pose.pose.position.y;
        X.translation() << x, y;
        X.linear()=q.toRotationMatrix().topLeftCorner<2,2>();
        localizer.X=X;
        
        init_pose_received = true;
    }
}

void laserCallBack(const sensor_msgs::LaserScan& scan){
    if(map_received == true && init_pose_received == true){
        std::vector<Vector2f> scan_endpoints;
        //extract point from laser 
        for (size_t i=0; i<scan.ranges.size(); ++i) {
            float alpha=scan.angle_min+i*scan.angle_increment;
            float r=scan.ranges[i];
            if (r< scan.range_min || r> scan.range_max)
            continue;
            scan_endpoints.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
        }

        //Compute localize and print odometry
        localizer.localize(scan_endpoints,10);
        std::cerr << "t: " << localizer.X.translation().transpose() << std::endl;
    }
    return;
}

int main(int argc, char** argv){
    //create topic name
    std::string map_topic_name = "/map";
    std::string init_topic_name = "/initialpose";
    std::string laser_topic_name = "/base_scan";

    //create handle node
    ros::init(argc,argv,"main_node");
    ros::NodeHandle n_map; 
    ros::NodeHandle n_init_pose;
    ros::NodeHandle n_laser_scan;

    //subscribe to topic from topic name
    ros::Subscriber sub = n_map.subscribe<const nav_msgs::OccupancyGrid&>(map_topic_name, 10, mapCallBack);
    ros::Subscriber sub_init_pose = n_init_pose.subscribe<const geometry_msgs::PoseWithCovarianceStamped&>(init_topic_name, 10, initCallBack);
    ros::Subscriber sub_laser_scan = n_laser_scan.subscribe<const sensor_msgs::LaserScan&>(laser_topic_name, 10, laserCallBack);

    ros::spin();
    return 0;
} 