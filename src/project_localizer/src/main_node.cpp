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

bool map_received = false;
bool init_pose_received = false; 


void mapCallBack(const nav_msgs::OccupancyGrid& map){
    if(map_received == false){
        std::cerr << "Map received" << std::endl;
        std::vector<Vector2f> obstacles;
        
        uint32_t width = map.info.width;
        uint32_t height = map.info.height;
        float resolution = map.info.resolution;

        std::cerr << "    width = " << width << std::endl << "    height = " << height << std::endl;
        
        GridMap grid_map(resolution,height,width);
        grid_map.cells = map.data;

        // Canvas canvas;
        // grid_map.draw(canvas);
        // showCanvas(canvas,0);

        //Extract a obstacles points
        for(const auto& o: grid_map.cells){
            if(o == 100){
                //Convert the occupated cell in grid coordinates 
                Vector2i index = grid_map.ptr2idx(&o);
                //Extract the value from the cell
                int8_t val = grid_map(index(0),index(1));
                Vector2f obs(static_cast<float>(val), static_cast<float>(val));
                //transform point from world to grid
                obs = grid_map.world2grid(obs);
                obstacles.push_back(obs);
            }
        }

        std::cerr << "num obstacles = " << obstacles.size() << std::endl;

        if(obstacles.size() <= 0){
            std::cerr << "Not obstacles" << std::endl;
            return; 
        }

        localizer.setMap(obstacles,resolution,1);
        std::cerr << "localizer ready" << std::endl;
        std::cerr << "  rows:  " << localizer.distances.rows << std::endl << "  cols: " << localizer.distances.cols << std::endl;

        // // Canvas canvas;
        // const auto& distances = localizer.distances;
        // Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
        
        // float f_min=1e9;
        // float f_max=0;
        // for(auto& f: distances.cells) {
        //     f_min=std::min(f, f_min);
        //     f_max=std::max(f, f_max);
        // }
        // float scale=255./(f_max-f_min);

        // // 2. copy the (normalized) distances
        // for (size_t i=0; i<distances.cells.size(); ++i) {
        //     obstacle_image.cells[i]=scale  * (distances.cells[i] - f_min);
        // }
        
        
        // obstacle_image.draw(canvas);
        
        // showCanvas(canvas,0);
        
        map_received = true;
    }
    std::cerr << "Dmap ready" << std::endl;
}

void initCallBack(const geometry_msgs::PoseWithCovarianceStamped& init){
    if(map_received == true && init_pose_received == false){
        std::cerr << "Init pose received" << std::endl;

        //Extract values from msg and convert
        //from world to grid
        
        Isometry2f X=Eigen::Isometry2f::Identity();
        float x = init.pose.pose.position.x;
        float y = init.pose.pose.position.y;
        
        X.translation() << x, y;
        
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