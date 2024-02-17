#include <ros/ros.h>
#include <iostream>

#include <nav_msgs/OccupancyGrid.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rp_stuff/dmap.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap_localizer.h>


DMap dmap(0,0);
DMapLocalizer localizer;

bool map_received = false;
bool init_pose_received = false; 


void mapCallBack(const nav_msgs::OccupancyGrid& map){
    if(map_received == false){
        std::cerr << "Map received" << std::endl;
        std::vector<Vector2i> obstacles;
        
        auto& width = map.info.width;
        auto& height = map.info.height;
        auto& resolution = map.info.resolution;

        dmap.resize(height,width);
        
        Grid_<int8_t> grid_map(height,width);
        grid_map.cells = map.data;
        
        //Extract a obstacles points --> points < 127
        //add to obstacles with function ptr2idx
        for(const auto& o: grid_map.cells){
            if(o < 127){
                obstacles.push_back(grid_map.ptr2idx(&o));
            }
        }
        
        //Compute a dmap
        int dmax2 = pow(1 / resolution, 2);
        int ops = dmap.compute(obstacles,dmax2);
        map_received = true;    
    }
    std::cerr << "Dmap ready" << std::endl;
}

void initCallBack(const geometry_msgs::PoseWithCovarianceStamped& init){
    if(map_received == true && init_pose_received == false){
        std::cerr << "Init pose received" << std::endl;

        init_pose_received = true;
    }
}

void laserCallBack(const sensor_msgs::LaserScan& scan){
    if(map_received == true && init_pose_received == true){
        std::cerr << "Scan received" << std::endl;
    }
    return;
}

int main(int argc, char** argv){
    std::string map_topic_name = "/map";
    std::string init_topic_name = "/initialpose";
    std::string laser_topic_name = "/laser";

    ros::init(argc,argv,"publisher_node");
    ros::NodeHandle n_map; 
    ros::NodeHandle n_init_pose;
    ros::NodeHandle n_laser_scan;

    ros::Subscriber sub = n_map.subscribe<const nav_msgs::OccupancyGrid&>(map_topic_name, 10, mapCallBack);
    ros::Subscriber sub_init_pose = n_init_pose.subscribe<const geometry_msgs::PoseWithCovarianceStamped&>(init_topic_name, 10, initCallBack);
    ros::Subscriber sub_laser_scan = n_laser_scan.subscribe<const sensor_msgs::LaserScan&>(laser_topic_name, 10, laserCallBack);


    ros::spin();
    return 0;
} 